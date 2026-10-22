// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <crypto/engine.h>
#include <crypto/hmac.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#define HASH_CONFIG_REG				0x00
#define HASH_PAD_LENGTH_REG			0x04
#define HASH_INT_ENABLE_REG			0x08
#define HASH_INT_STATUS_REG			0x0c
#define HASH_IV_REG				0x50
#define HASH_OUTPUT_REG				0x70
#define HASH_INPUT_REG				0x90
#define HASH_DMA_SOURCE_REG			0x94
#define HASH_DMA_LENGTH_REG			0x9c
#define HASH_DMA_AXI_CONTROL_REG		0xb0

/* HASH_CONFIG bit fields */
#define HASH_CONFIG_RESET			BIT(0)
#define HASH_CONFIG_ENABLE			BIT(1)
#define HASH_CONFIG_MODE			GENMASK(5, 4)
#define HASH_CONFIG_MODE_SHA1			0x0
#define HASH_CONFIG_MODE_SHA256			0x1
#define HASH_CONFIG_IV				BIT(8)
#define HASH_CONFIG_DMA				BIT(12)
#define HASH_CONFIG_INIT			BIT(20)
#define HASH_CONFIG_DISABLED \
	(FIELD_PREP_CONST(HASH_CONFIG_ENABLE, 0))

/* HASH_INT_ENABLE bit fields */
#define HASH_INT_ENABLE_DONE			BIT(0)
#define HASH_INT_ENABLE_DISABLE_ALL \
	(FIELD_PREP_CONST(HASH_INT_ENABLE_DONE, 0))

/* HASH_INT_STATUS bit fields */
#define HASH_INT_STATUS_DONE			BIT(0)
#define HASH_INT_STATUS_ALL			GENMASK(1, 0)

/* HASH_DMA_AXI_CONTROL bit fields */
#define HASH_DMA_AXI_CONTROL_BIT16		BIT(16)

#define HASH_STATE_STARTED			BIT(0)
#define HASH_STATE_FAILED			BIT(1)
#define HASH_STATE_FINAL			BIT(2)

#define HASH_DMA_BUFFER_SIZE			SZ_4K

#define HASH_NUM_ALGS				4
#define HASH_PRIORITY				300

#define HASH_RESET_TIMEOUT_US			100
#define HASH_RESET_DELAY_US			10
#define HASH_TIMEOUT_US				3000000
#define HASH_DMA_TIMEOUT_MS			3000
#define HASH_PM_TIMEOUT_MS			10000
#define HASH_RESULT_DELAY_US			3

struct novatek_hash_dev;

struct novatek_hash_alg {
	struct ahash_engine_alg alg;
	struct novatek_hash_dev *dev;
	bool sha256;
	bool hmac;
	bool released;
};

struct novatek_hash_dev {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct crypto_engine *engine;
	struct kref refcount;
	struct completion dma_complete;
	u8 *dma_buffer;
	dma_addr_t dma_address;
	uint dma_length;
	int irq;
	atomic_t pending;
	wait_queue_head_t idle;
	spinlock_t lock;
	bool suspended;
	bool removing;
	bool powered;
	bool dma_mapped;
	bool irq_requested;
	bool irq_disabled;
	bool faulted;
	struct novatek_hash_alg algs[HASH_NUM_ALGS];
};

struct novatek_hash_ctx {
	struct novatek_hash_dev *dev;
	struct crypto_shash *fallback;
	u8 key[SHA256_BLOCK_SIZE];
	u32 mode;
	bool hmac;
};

struct novatek_hash_state {
	u64 count;
	u32 digest[SHA256_DIGEST_SIZE / sizeof(u32)];
	u8 buffer[SHA256_BLOCK_SIZE];
	u8 opad[SHA256_BLOCK_SIZE];
	u32 buffered;
	u32 flags;
};

struct novatek_hash_reqctx {
	struct novatek_hash_state state;
	uint nbytes;
	bool final;
};

static void novatek_hash_release(struct kref *refcount)
{
	struct novatek_hash_dev *hash;

	hash = container_of(refcount, struct novatek_hash_dev, refcount);
	if (!hash->dma_mapped)
		kfree_sensitive(hash->dma_buffer);
	put_device(hash->dev);
	kfree(hash);
}

static void novatek_hash_destroy_alg(struct crypto_alg *base)
{
	struct novatek_hash_alg *alg;

	alg = container_of(base, struct novatek_hash_alg, alg.base.halg.base);
	WRITE_ONCE(alg->released, true);
	kref_put(&alg->dev->refcount, novatek_hash_release);
}

static int novatek_hash_reset(struct novatek_hash_dev *hash)
{
	u32 value;
	int ret;

	writel(HASH_INT_ENABLE_DISABLE_ALL, hash->base + HASH_INT_ENABLE_REG);

	writel(HASH_CONFIG_DISABLED, hash->base + HASH_CONFIG_REG);
	value = HASH_CONFIG_RESET;
	writel(value, hash->base + HASH_CONFIG_REG);
	ret = readl_poll_timeout(hash->base + HASH_CONFIG_REG,
				 value, !(value & HASH_CONFIG_RESET),
				 1, HASH_RESET_TIMEOUT_US);
	if (ret)
		return ret;

	value = HASH_INT_STATUS_ALL;
	writel(value, hash->base + HASH_INT_STATUS_REG);
	return 0;
}

static void novatek_hash_unmap(struct novatek_hash_dev *hash)
{
	if (!hash->dma_mapped)
		return;

	dma_unmap_single(hash->dev, hash->dma_address, hash->dma_length, DMA_TO_DEVICE);
	hash->dma_mapped = false;

	memzero_explicit(hash->dma_buffer, hash->dma_length);
}

static int novatek_hash_dma_stop(struct novatek_hash_dev *hash)
{
	int ret;

	writel(HASH_INT_ENABLE_DISABLE_ALL, hash->base + HASH_INT_ENABLE_REG);
	synchronize_irq(hash->irq);

	ret = novatek_hash_reset(hash);
	if (!ret) {
		novatek_hash_unmap(hash);
		return 0;
	}

	ret = reset_control_assert(hash->reset);
	if (ret) {
		WRITE_ONCE(hash->faulted, true);
		return ret;
	}
	reset_control_status(hash->reset);
	udelay(HASH_RESET_DELAY_US);

	novatek_hash_unmap(hash);

	ret = reset_control_deassert(hash->reset);
	if (!ret)
		ret = novatek_hash_reset(hash);
	WRITE_ONCE(hash->faulted, !!ret);
	return ret;
}

static irqreturn_t novatek_hash_irq(int irq, void *data)
{
	struct novatek_hash_dev *hash = data;
	u32 status;

	status = readl(hash->base + HASH_INT_STATUS_REG);
	if (!(status & HASH_INT_STATUS_DONE))
		return IRQ_NONE;

	writel(HASH_INT_ENABLE_DISABLE_ALL, hash->base + HASH_INT_ENABLE_REG);
	writel(HASH_INT_STATUS_DONE, hash->base + HASH_INT_STATUS_REG);

	complete(&hash->dma_complete);
	return IRQ_HANDLED;
}

static void novatek_hash_disable(struct novatek_hash_dev *hash)
{
	int ret;

	if (hash->irq_requested) {
		if (hash->powered)
			writel(HASH_INT_ENABLE_DISABLE_ALL, hash->base + HASH_INT_ENABLE_REG);
		synchronize_irq(hash->irq);
		devm_free_irq(hash->dev, hash->irq, hash);
		hash->irq_requested = false;
	}

	if (!hash->powered)
		return;

	ret = reset_control_assert(hash->reset);
	if (!ret) {
		reset_control_status(hash->reset);
		udelay(HASH_RESET_DELAY_US);
		novatek_hash_unmap(hash);
	}
	if (hash->dma_mapped) {
		dev_err(hash->dev, "DMA is still active; retaining its resources\n");
		return;
	}

	clk_disable_unprepare(hash->clk);
	hash->powered = false;
}

static u32 novatek_hash_config(struct novatek_hash_dev *hash,
			       struct novatek_hash_state *state, u32 mode)
{
	u32 config;
	uint i;

	config = FIELD_PREP(HASH_CONFIG_MODE, mode);
	if (state->flags & HASH_STATE_STARTED) {
		for (i = 0; i < ARRAY_SIZE(state->digest); i++)
			writel(state->digest[i], hash->base + HASH_IV_REG + i * sizeof(u32));
		config |= HASH_CONFIG_IV;
	}
	return config;
}

static void novatek_hash_read_digest(struct novatek_hash_dev *hash,
				     struct novatek_hash_state *state, u32 mode)
{
	uint words;
	uint i;

	udelay(HASH_RESULT_DELAY_US);
	words = mode == HASH_CONFIG_MODE_SHA1 ?
		SHA1_DIGEST_SIZE / sizeof(u32) : ARRAY_SIZE(state->digest);
	for (i = 0; i < words; i++)
		state->digest[i] = readl(hash->base + HASH_OUTPUT_REG + i * sizeof(u32));
	memset(state->digest + words, 0, sizeof(state->digest) - words * sizeof(u32));
	state->flags |= HASH_STATE_STARTED;
	cond_resched();
}

static int novatek_hash_pio(struct novatek_hash_dev *hash,
			    struct novatek_hash_state *state, u32 mode, const u8 *data)
{
	u32 config, status, value;
	uint i;
	int ret;

	if (READ_ONCE(hash->removing))
		return -ENODEV;

	config = novatek_hash_config(hash, state, mode);
	writel(config, hash->base + HASH_CONFIG_REG);

	value = HASH_INT_STATUS_DONE;
	writel(value, hash->base + HASH_INT_STATUS_REG);

	config |= HASH_CONFIG_ENABLE | HASH_CONFIG_INIT;
	writel(config, hash->base + HASH_CONFIG_REG);

	for (i = 0; i < SHA256_BLOCK_SIZE; i += sizeof(u32))
		writel(get_unaligned_le32(data + i), hash->base + HASH_INPUT_REG);
	ret = readl_poll_timeout(hash->base + HASH_INT_STATUS_REG,
				 status, status & HASH_INT_STATUS_DONE,
				 1, HASH_TIMEOUT_US);
	if (ret)
		return ret;

	novatek_hash_read_digest(hash, state, mode);
	return 0;
}

static int novatek_hash_dma(struct novatek_hash_dev *hash,
			    struct novatek_hash_state *state, u32 mode)
{
	u32 config, value;

	config = novatek_hash_config(hash, state, mode);

	writel(0, hash->base + HASH_PAD_LENGTH_REG);
	writel(lower_32_bits(hash->dma_address), hash->base + HASH_DMA_SOURCE_REG);
	writel(hash->dma_length, hash->base + HASH_DMA_LENGTH_REG);

	value = HASH_INT_STATUS_DONE;
	writel(value, hash->base + HASH_INT_STATUS_REG);

	reinit_completion(&hash->dma_complete);
	value = HASH_INT_ENABLE_DONE;
	writel(value, hash->base + HASH_INT_ENABLE_REG);

	value = readl(hash->base + HASH_DMA_AXI_CONTROL_REG);
	value &= ~HASH_DMA_AXI_CONTROL_BIT16;
	writel(value, hash->base + HASH_DMA_AXI_CONTROL_REG);

	config |= HASH_CONFIG_ENABLE | HASH_CONFIG_DMA;
	writel(config, hash->base + HASH_CONFIG_REG);
	if (!wait_for_completion_timeout(&hash->dma_complete,
					 msecs_to_jiffies(HASH_DMA_TIMEOUT_MS))) {
		if (novatek_hash_dma_stop(hash))
			dev_err_ratelimited(hash->dev, "Failed to stop hash DMA\n");
		return -ETIMEDOUT;
	}

	novatek_hash_read_digest(hash, state, mode);

	novatek_hash_unmap(hash);
	return 0;
}

static int novatek_hash_blocks(struct novatek_hash_dev *hash, struct novatek_hash_state *state,
			       u32 mode, const u8 *data, uint length)
{
	uint offset;
	int ret;

	if (hash->dma_buffer) {
		if (data != hash->dma_buffer)
			memcpy(hash->dma_buffer, data, length);

		hash->dma_address = dma_map_single(hash->dev,
						   hash->dma_buffer, length,
						   DMA_TO_DEVICE);
		if (dma_mapping_error(hash->dev, hash->dma_address)) {
			memzero_explicit(hash->dma_buffer, length);
			return -ENOMEM;
		}
		hash->dma_length = length;
		hash->dma_mapped = true;

		return novatek_hash_dma(hash, state, mode);
	}

	ret = 0;
	for (offset = 0; offset < length; offset += SHA256_BLOCK_SIZE) {
		ret = novatek_hash_pio(hash, state, mode, data + offset);
		if (ret)
			break;
	}

	return ret;
}

static int novatek_hash_finish(struct novatek_hash_dev *hash,
			       struct novatek_hash_state *state, u32 mode)
{
	u64 bits = state->count << 3;
	int ret;

	state->buffer[state->buffered++] = 0x80;
	if (state->buffered > SHA256_BLOCK_SIZE - sizeof(bits)) {
		memset(state->buffer + state->buffered, 0, SHA256_BLOCK_SIZE - state->buffered);
		ret = novatek_hash_blocks(hash, state, mode, state->buffer, SHA256_BLOCK_SIZE);
		if (ret)
			return ret;
		state->buffered = 0;
	}

	memset(state->buffer + state->buffered,
	       0, SHA256_BLOCK_SIZE - sizeof(bits) - state->buffered);
	put_unaligned_be64(bits, state->buffer + SHA256_BLOCK_SIZE - sizeof(bits));
	return novatek_hash_blocks(hash, state, mode, state->buffer, SHA256_BLOCK_SIZE);
}

static int novatek_hash_do_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = ahash_request_cast(areq);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct novatek_hash_ctx *ctx = crypto_ahash_ctx(tfm);
	struct novatek_hash_reqctx *rctx = ahash_request_ctx(req);
	struct novatek_hash_state *state = &rctx->state;
	struct novatek_hash_dev *hash = ctx->dev;
	uint digestsize = crypto_ahash_digestsize(tfm);
	struct scatter_walk walk;
	u8 digest[SHA256_DIGEST_SIZE];
	uint offset = 0;
	uint count, i;
	int ret;

	if (READ_ONCE(hash->removing)) {
		ret = -ENODEV;
		goto out_complete_request;
	}

	if (READ_ONCE(hash->faulted)) {
		ret = -EIO;
		goto out_wipe_digest;
	}

	ret = novatek_hash_reset(hash);
	if (ret)
		goto out_wipe_digest;

	if (rctx->nbytes)
		scatterwalk_start(&walk, req->src);
	while (offset < rctx->nbytes || state->buffered == SHA256_BLOCK_SIZE) {
		if (state->buffered == SHA256_BLOCK_SIZE) {
			ret = novatek_hash_blocks(hash,
						  state, ctx->mode,
						  state->buffer, SHA256_BLOCK_SIZE);
			if (ret)
				goto out_wipe_digest;
			state->buffered = 0;
		}

		if (hash->dma_buffer && !state->buffered &&
		    rctx->nbytes - offset >= SHA256_BLOCK_SIZE) {
			count = min(round_down(rctx->nbytes - offset, SHA256_BLOCK_SIZE),
				    HASH_DMA_BUFFER_SIZE);
			memcpy_from_scatterwalk(hash->dma_buffer, &walk, count);

			ret = novatek_hash_blocks(hash, state, ctx->mode, hash->dma_buffer, count);
			if (ret)
				goto out_wipe_digest;
			state->count += count;
			offset += count;
			continue;
		}

		count = min(rctx->nbytes - offset, SHA256_BLOCK_SIZE - state->buffered);
		if (count) {
			memcpy_from_scatterwalk(state->buffer + state->buffered, &walk, count);
			state->buffered += count;
			state->count += count;
			offset += count;
		}
	}

	if (!rctx->final)
		goto out_wipe_digest;

	ret = novatek_hash_finish(hash, state, ctx->mode);
	if (ret)
		goto out_wipe_digest;

	if (ctx->hmac) {
		for (i = 0; i < digestsize / sizeof(u32); i++)
			put_unaligned_le32(state->digest[i], digest + i * sizeof(u32));

		state->flags &= ~HASH_STATE_STARTED;
		ret = novatek_hash_blocks(hash, state, ctx->mode, state->opad, SHA256_BLOCK_SIZE);
		if (ret)
			goto out_wipe_digest;

		memcpy(state->buffer, digest, digestsize);
		state->buffered = digestsize;
		state->count = SHA256_BLOCK_SIZE + digestsize;
		ret = novatek_hash_finish(hash, state, ctx->mode);
		if (ret)
			goto out_wipe_digest;
	}

	for (i = 0; i < digestsize / sizeof(u32); i++)
		put_unaligned_le32(state->digest[i], req->result + i * sizeof(u32));

	memzero_explicit(state, sizeof(*state));
	state->flags = HASH_STATE_FINAL;

out_wipe_digest:
	memzero_explicit(digest, sizeof(digest));
	if (ret) {
		state->flags |= HASH_STATE_FAILED;
		if (!READ_ONCE(hash->faulted) && novatek_hash_reset(hash)) {
			WRITE_ONCE(hash->faulted, true);
			dev_err_ratelimited(hash->dev, "Failed to reset hash engine\n");
		}
	}
out_complete_request:
	local_bh_disable();
	crypto_finalize_hash_request(engine, req, ret);
	local_bh_enable();
	if (atomic_dec_and_test(&hash->pending))
		wake_up(&hash->idle);
	return 0;
}

static int novatek_hash_init(struct ahash_request *req)
{
	struct novatek_hash_ctx *ctx;
	struct novatek_hash_reqctx *rctx = ahash_request_ctx(req);
	struct novatek_hash_state *state = &rctx->state;
	uint i;

	ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	memset(rctx, 0, sizeof(*rctx));
	if (ctx->hmac) {
		for (i = 0; i < SHA256_BLOCK_SIZE; i++) {
			state->buffer[i] = ctx->key[i] ^ HMAC_IPAD_VALUE;
			state->opad[i] = ctx->key[i] ^ HMAC_OPAD_VALUE;
		}
		state->buffered = SHA256_BLOCK_SIZE;
		state->count = SHA256_BLOCK_SIZE;
	}
	return 0;
}

static int novatek_hash_enqueue(struct ahash_request *req, bool update, bool final)
{
	struct novatek_hash_ctx *ctx;
	struct novatek_hash_reqctx *rctx = ahash_request_ctx(req);
	struct novatek_hash_dev *hash;
	unsigned long flags;
	int ret;

	ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	hash = ctx->dev;

	if (rctx->state.flags & (HASH_STATE_FAILED | HASH_STATE_FINAL))
		return -EINVAL;

	if (update && req->nbytes && (!req->src || sg_nents_for_len(req->src, req->nbytes) < 0))
		return -EINVAL;

	if (final && !req->result)
		return -EINVAL;

	rctx->final = final;
	rctx->nbytes = update ? req->nbytes : 0;
	spin_lock_irqsave(&hash->lock, flags);
	if (hash->removing) {
		ret = -ENODEV;
	} else if (hash->suspended) {
		ret = -ESHUTDOWN;
	} else if (READ_ONCE(hash->faulted)) {
		ret = -EIO;
	} else {
		atomic_inc(&hash->pending);
		ret = crypto_transfer_hash_request_to_engine(hash->engine, req);
		if (ret != -EINPROGRESS && ret != -EBUSY) {
			if (atomic_dec_and_test(&hash->pending))
				wake_up(&hash->idle);
		}
	}
	spin_unlock_irqrestore(&hash->lock, flags);
	return ret;
}

static int novatek_hash_update(struct ahash_request *req)
{
	return novatek_hash_enqueue(req, true, false);
}

static int novatek_hash_final(struct ahash_request *req)
{
	return novatek_hash_enqueue(req, false, true);
}

static int novatek_hash_finup(struct ahash_request *req)
{
	return novatek_hash_enqueue(req, true, true);
}

static int novatek_hash_digest(struct ahash_request *req)
{
	novatek_hash_init(req);
	return novatek_hash_enqueue(req, true, true);
}

static int novatek_hash_export(struct ahash_request *req, void *out)
{
	struct novatek_hash_reqctx *rctx = ahash_request_ctx(req);

	memcpy(out, &rctx->state, sizeof(rctx->state));
	return 0;
}

static int novatek_hash_import(struct ahash_request *req, const void *in)
{
	struct novatek_hash_reqctx *rctx = ahash_request_ctx(req);
	struct novatek_hash_state state;
	int ret = 0;

	memcpy(&state, in, sizeof(state));
	if (state.buffered > SHA256_BLOCK_SIZE ||
	    (state.count % SHA256_BLOCK_SIZE) != (state.buffered % SHA256_BLOCK_SIZE) ||
	    state.flags & ~(HASH_STATE_STARTED | HASH_STATE_FAILED | HASH_STATE_FINAL)) {
		ret = -EINVAL;
		goto out_wipe_state;
	}
	memcpy(&rctx->state, &state, sizeof(state));

out_wipe_state:
	memzero_explicit(&state, sizeof(state));
	if (!ret)
		rctx->final = false;
	return ret;
}

static int novatek_hash_setkey(struct crypto_ahash *tfm, const u8 *key, unsigned int keylen)
{
	struct novatek_hash_ctx *ctx = crypto_ahash_ctx(tfm);
	u8 block[SHA256_BLOCK_SIZE] = {};
	int ret = 0;

	if (keylen > sizeof(block))
		ret = crypto_shash_tfm_digest(ctx->fallback, key, keylen, block);
	else
		memcpy(block, key, keylen);
	if (!ret)
		memcpy(ctx->key, block, sizeof(block));
	memzero_explicit(block, sizeof(block));
	return ret;
}

static int novatek_hash_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_ahash *ahash = __crypto_ahash_cast(tfm);
	struct novatek_hash_ctx *ctx = crypto_tfm_ctx(tfm);
	struct novatek_hash_alg *alg;
	int ret;

	alg = container_of(crypto_ahash_alg(ahash), struct novatek_hash_alg, alg.base);
	ctx->dev = alg->dev;
	kref_get(&ctx->dev->refcount);

	ctx->hmac = alg->hmac;
	ctx->mode = alg->sha256 ? HASH_CONFIG_MODE_SHA256 : HASH_CONFIG_MODE_SHA1;

	crypto_ahash_set_reqsize(ahash, sizeof(struct novatek_hash_reqctx));

	if (ctx->hmac) {
		ctx->fallback = crypto_alloc_shash(alg->sha256 ? "sha256" : "sha1",
						   0, CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(ctx->fallback)) {
			ret = PTR_ERR(ctx->fallback);
			kref_put(&ctx->dev->refcount, novatek_hash_release);
			return ret;
		}
	}
	return 0;
}

static void novatek_hash_cra_exit(struct crypto_tfm *tfm)
{
	struct novatek_hash_ctx *ctx = crypto_tfm_ctx(tfm);

	if (ctx->hmac)
		crypto_free_shash(ctx->fallback);
	memzero_explicit(ctx->key, sizeof(ctx->key));
	kref_put(&ctx->dev->refcount, novatek_hash_release);
}

static const struct novatek_hash_alg novatek_hash_algs[] = {
	{
		.alg.base.halg.base.cra_name = "sha1",
		.alg.base.halg.base.cra_driver_name = "novatek-sha1",
		.alg.base.halg.digestsize = SHA1_DIGEST_SIZE,
	}, {
		.alg.base.halg.base.cra_name = "sha256",
		.alg.base.halg.base.cra_driver_name = "novatek-sha256",
		.alg.base.halg.digestsize = SHA256_DIGEST_SIZE,
		.sha256 = true,
	}, {
		.alg.base.halg.base.cra_name = "hmac(sha1)",
		.alg.base.halg.base.cra_driver_name = "novatek-hmac-sha1",
		.alg.base.halg.digestsize = SHA1_DIGEST_SIZE,
		.hmac = true,
	}, {
		.alg.base.halg.base.cra_name = "hmac(sha256)",
		.alg.base.halg.base.cra_driver_name = "novatek-hmac-sha256",
		.alg.base.halg.digestsize = SHA256_DIGEST_SIZE,
		.sha256 = true,
		.hmac = true,
	},
};

#ifdef CONFIG_PM_SLEEP
static int novatek_hash_suspend(struct device *dev)
{
	struct novatek_hash_dev *hash = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&hash->lock, flags);
	hash->suspended = true;
	spin_unlock_irqrestore(&hash->lock, flags);

	ret = -EBUSY;
	if (!wait_event_timeout(hash->idle,
				!atomic_read(&hash->pending),
				msecs_to_jiffies(HASH_PM_TIMEOUT_MS)))
		goto out_restore_requests;
	if (hash->dma_mapped)
		goto out_restore_requests;

	ret = crypto_engine_stop(hash->engine);
	if (ret)
		goto out_restore_requests;

	if (hash->powered) {
		writel(HASH_INT_ENABLE_DISABLE_ALL, hash->base + HASH_INT_ENABLE_REG);
		if (hash->irq_requested) {
			disable_irq(hash->irq);
			hash->irq_disabled = true;
		}

		writel(HASH_CONFIG_DISABLED, hash->base + HASH_CONFIG_REG);

		clk_disable_unprepare(hash->clk);
		hash->powered = false;
	}
	return 0;

out_restore_requests:
	spin_lock_irqsave(&hash->lock, flags);
	hash->suspended = false;
	spin_unlock_irqrestore(&hash->lock, flags);
	return ret;
}

static int novatek_hash_resume(struct device *dev)
{
	struct novatek_hash_dev *hash = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	ret = clk_prepare_enable(hash->clk);
	if (ret)
		return ret;
	hash->powered = true;

	ret = reset_control_deassert(hash->reset);
	if (ret)
		goto out_disable_clk;

	ret = novatek_hash_reset(hash);
	if (ret)
		goto out_disable_clk;

	ret = crypto_engine_start(hash->engine);
	if (ret)
		goto out_disable_clk;

	if (hash->irq_disabled) {
		enable_irq(hash->irq);
		hash->irq_disabled = false;
	}

	spin_lock_irqsave(&hash->lock, flags);
	hash->faulted = false;
	hash->suspended = false;
	spin_unlock_irqrestore(&hash->lock, flags);
	return 0;

out_disable_clk:
	clk_disable_unprepare(hash->clk);
	hash->powered = false;
	return ret;
}
#endif

static const struct dev_pm_ops novatek_hash_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_hash_suspend, novatek_hash_resume)
};

static int novatek_hash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_hash_dev *hash;
	struct ahash_alg *alg;
	unsigned long flags;
	uint i;
	int ret;

	hash = kzalloc_obj(*hash);
	if (!hash)
		return -ENOMEM;

	hash->dev = get_device(dev);
	kref_init(&hash->refcount);
	atomic_set(&hash->pending, 0);
	init_waitqueue_head(&hash->idle);
	spin_lock_init(&hash->lock);
	init_completion(&hash->dma_complete);
	platform_set_drvdata(pdev, hash);

	hash->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hash->base)) {
		ret = PTR_ERR(hash->base);
		goto out_put_hash;
	}

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		ret = dma_set_mask(dev, DMA_BIT_MASK(32));
		if (ret)
			goto out_put_hash;

		hash->dma_buffer = kmalloc(HASH_DMA_BUFFER_SIZE, GFP_KERNEL);
		if (!hash->dma_buffer) {
			ret = -ENOMEM;
			goto out_put_hash;
		}
	}

	hash->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(hash->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(hash->clk), "Failed to get clock\n");
		goto out_put_hash;
	}

	hash->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(hash->reset)) {
		ret = dev_err_probe(dev, PTR_ERR(hash->reset), "Failed to get reset\n");
		goto out_put_hash;
	}

	ret = clk_prepare_enable(hash->clk);
	if (ret)
		goto out_put_hash;
	hash->powered = true;

	ret = reset_control_deassert(hash->reset);
	if (ret)
		goto out_disable_clk;

	ret = novatek_hash_reset(hash);
	if (ret)
		goto out_disable_device;

	if (hash->dma_buffer) {
		hash->irq = platform_get_irq(pdev, 0);
		if (hash->irq < 0) {
			ret = hash->irq;
			goto out_disable_device;
		}
		ret = devm_request_irq(dev, hash->irq, novatek_hash_irq, 0, dev_name(dev), hash);
		if (ret)
			goto out_disable_device;
		hash->irq_requested = true;
	}

	hash->engine = crypto_engine_alloc_init(dev, false);
	if (!hash->engine) {
		ret = -ENOMEM;
		goto out_disable_device;
	}

	ret = crypto_engine_start(hash->engine);
	if (ret)
		goto out_exit_engine;

	memcpy(hash->algs, novatek_hash_algs, sizeof(hash->algs));
	for (i = 0; i < ARRAY_SIZE(hash->algs); i++) {
		hash->algs[i].dev = hash;
		alg = &hash->algs[i].alg.base;
		alg->init = novatek_hash_init;
		alg->update = novatek_hash_update;
		alg->final = novatek_hash_final;
		alg->finup = novatek_hash_finup;
		alg->digest = novatek_hash_digest;
		alg->export = novatek_hash_export;
		alg->import = novatek_hash_import;
		if (hash->algs[i].hmac)
			alg->setkey = novatek_hash_setkey;

		alg->halg.statesize = sizeof(struct novatek_hash_state);
		alg->halg.base.cra_priority = HASH_PRIORITY;
		alg->halg.base.cra_flags = CRYPTO_ALG_ASYNC;
		alg->halg.base.cra_blocksize = SHA256_BLOCK_SIZE;
		alg->halg.base.cra_ctxsize = sizeof(struct novatek_hash_ctx);
		alg->halg.base.cra_module = THIS_MODULE;

		alg->halg.base.cra_init = novatek_hash_cra_init;
		alg->halg.base.cra_exit = novatek_hash_cra_exit;
		alg->halg.base.cra_destroy = novatek_hash_destroy_alg;
		hash->algs[i].alg.op.do_one_request = novatek_hash_do_request;

		kref_get(&hash->refcount);
		ret = crypto_engine_register_ahash(&hash->algs[i].alg);
		if (ret) {
			if (!READ_ONCE(hash->algs[i].released))
				kref_put(&hash->refcount, novatek_hash_release);
			goto out_stop_requests;
		}
	}
	return 0;

out_stop_requests:
	spin_lock_irqsave(&hash->lock, flags);
	hash->removing = true;
	spin_unlock_irqrestore(&hash->lock, flags);
	while (i--)
		crypto_engine_unregister_ahash(&hash->algs[i].alg);
	wait_event(hash->idle, !atomic_read(&hash->pending));
out_exit_engine:
	crypto_engine_exit(hash->engine);
out_disable_device:
	novatek_hash_disable(hash);
	goto out_put_hash;
out_disable_clk:
	clk_disable_unprepare(hash->clk);
	hash->powered = false;
out_put_hash:
	kref_put(&hash->refcount, novatek_hash_release);
	return ret;
}

static void novatek_hash_remove(struct platform_device *pdev)
{
	struct novatek_hash_dev *hash = platform_get_drvdata(pdev);
	unsigned long flags;
	uint i;

	spin_lock_irqsave(&hash->lock, flags);
	hash->removing = true;
	spin_unlock_irqrestore(&hash->lock, flags);

	for (i = 0; i < ARRAY_SIZE(hash->algs); i++)
		crypto_engine_unregister_ahash(&hash->algs[i].alg);

	wait_event(hash->idle, !atomic_read(&hash->pending));
	crypto_engine_exit(hash->engine);

	novatek_hash_disable(hash);
	kref_put(&hash->refcount, novatek_hash_release);
}

static const struct of_device_id novatek_hash_of_match[] = {
	{ .compatible = "novatek,na51089-hash" },
	{}
};
MODULE_DEVICE_TABLE(of, novatek_hash_of_match);

static struct platform_driver novatek_hash_driver = {
	.probe = novatek_hash_probe,
	.remove = novatek_hash_remove,
	.driver = {
		.name = "novatek-hash",
		.of_match_table = novatek_hash_of_match,
		.pm = pm_sleep_ptr(&novatek_hash_pm_ops),
	},
};
module_platform_driver(novatek_hash_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek hash accelerator driver");
MODULE_LICENSE("GPL");
