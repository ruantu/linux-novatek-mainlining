// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/engine.h>
#include <crypto/gcm.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/des.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
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

#define CRYPTO_CONFIG_REG			0x00
#define CRYPTO_CONTROL_REG			0x04
#define CRYPTO_INT_ENABLE_REG			0x08
#define CRYPTO_INT_STATUS_REG			0x0c
#define CRYPTO_KEY_REG				0x10
#define CRYPTO_INPUT_REG			0x30
#define CRYPTO_OUTPUT_REG			0x40
#define CRYPTO_DMA_ADDRESS_REG			0x50
#define CRYPTO_AXI_CONTROL_REG			0x68

/* CRYPTO_CONFIG bit fields */
#define CRYPTO_CONFIG_RESET			BIT(0)
#define CRYPTO_CONFIG_ENABLE			BIT(1)
#define CRYPTO_CONFIG_MODE			GENMASK(5, 4)
#define CRYPTO_CONFIG_DES			0
#define CRYPTO_CONFIG_DES3			1
#define CRYPTO_CONFIG_AES128			2
#define CRYPTO_CONFIG_AES256			3
#define CRYPTO_CONFIG_DIRECTION			BIT(8)
#define CRYPTO_CONFIG_DISABLED \
	(FIELD_PREP_CONST(CRYPTO_CONFIG_ENABLE, 0))
#define CRYPTO_CONFIG_DEFAULT \
	(FIELD_PREP_CONST(CRYPTO_CONFIG_ENABLE, 1))

/* CRYPTO_CONTROL bit fields */
#define CRYPTO_CONTROL_START			BIT(0)
#define CRYPTO_CONTROL_DMA			BIT(4)

/* CRYPTO_INT_ENABLE bit fields */
#define CRYPTO_INT_ENABLE_DMA			BIT(4)
#define CRYPTO_INT_ENABLE_DISABLE_ALL \
	(FIELD_PREP_CONST(CRYPTO_INT_ENABLE_DMA, 0))

/* CRYPTO_INT_STATUS bit fields */
#define CRYPTO_INT_STATUS_PIO			BIT(0)
#define CRYPTO_INT_STATUS_DMA			BIT(4)

/* CRYPTO_AXI_CONTROL bit fields */
#define CRYPTO_AXI_CONTROL_DMA_DISABLE		BIT(16)

#define CRYPTO_DESC_DIRECTION			BIT(0)
#define CRYPTO_DESC_MODE			GENMASK(5, 4)
#define CRYPTO_DESC_OP				GENMASK(10, 8)
#define CRYPTO_DESC_ECB				0
#define CRYPTO_DESC_CBC				1
#define CRYPTO_DESC_CTR				4
#define CRYPTO_DESC_GCM				5

#define CRYPTO_DESC_LAST			BIT(0)

#define CRYPTO_DMA_CHUNK			SZ_64K
#define CRYPTO_DMA_BLOCKS			32

#define CRYPTO_PRIORITY				300
#define CRYPTO_AEAD_PRIORITY			400
#define CRYPTO_ALG_COUNT			9

#define CRYPTO_TIMEOUT_US			3000
#define CRYPTO_RESET_US				10
#define CRYPTO_DMA_TIMEOUT_MS			3000
#define CRYPTO_PM_TIMEOUT_MS			10000

enum novatek_crypto_type {
	NOVATEK_CRYPTO_AES,
	NOVATEK_CRYPTO_DES,
	NOVATEK_CRYPTO_DES3,
};

enum novatek_crypto_mode {
	NOVATEK_CRYPTO_ECB,
	NOVATEK_CRYPTO_CBC,
	NOVATEK_CRYPTO_CTR,
};

struct novatek_crypto;

struct novatek_crypto_dma_block {
	__le32 src;
	__le32 dst;
	__le32 length;
	__le32 config;
};

struct novatek_crypto_dma_desc {
	u8 key[AES_MAX_KEY_SIZE];
	u8 iv[AES_BLOCK_SIZE];
	__le32 counter[AES_BLOCK_SIZE / sizeof(u32)];
	__le32 header;
	__le32 reserved[3];
	u8 cv[AES_BLOCK_SIZE];
	u8 s0[AES_BLOCK_SIZE];
	u8 ghash[AES_BLOCK_SIZE];
	struct novatek_crypto_dma_block blocks[CRYPTO_DMA_BLOCKS];
};

struct novatek_crypto_dma_buf {
	void *data;
	dma_addr_t addr;
	size_t mapped_size;
	enum dma_data_direction direction;
	bool mapped;
};

struct novatek_crypto_alg {
	struct skcipher_engine_alg alg;
	struct novatek_crypto *crypto;
	enum novatek_crypto_type type;
	enum novatek_crypto_mode mode;
	bool released;
};

struct novatek_crypto {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct crypto_engine *engine;
	struct novatek_crypto_alg algs[CRYPTO_ALG_COUNT];
	struct aead_engine_alg aead;
	struct novatek_crypto_dma_desc *desc;
	dma_addr_t desc_dma;
	struct novatek_crypto_dma_buf input;
	struct novatek_crypto_dma_buf output;
	struct novatek_crypto_dma_buf aad;
	struct completion complete;
	struct kref ref;
	spinlock_t lock;
	wait_queue_head_t wait;
	atomic_t pending;
	int irq;
	bool irq_enabled;
	bool dma_active;
	bool aead_registered;
	bool aead_released;
	bool suspended;
	bool removing;
	bool faulted;
	bool powered;
};

struct novatek_crypto_ctx {
	struct novatek_crypto *crypto;
	struct crypto_skcipher *fallback;
	u8 key[AES_MAX_KEY_SIZE];
	uint keylen;
	uint blocksize;
	u32 hwmode;
	enum novatek_crypto_type type;
	enum novatek_crypto_mode mode;
};

struct novatek_crypto_reqctx {
	bool decrypt;
	struct skcipher_request fallback;
};

struct novatek_crypto_aead_ctx {
	struct novatek_crypto *crypto;
	struct crypto_aead *fallback;
	u8 key[AES_MAX_KEY_SIZE];
	uint keylen;
	u32 hwmode;
};

struct novatek_crypto_aead_reqctx {
	bool decrypt;
	struct aead_request fallback;
};

static void novatek_crypto_release(struct kref *ref)
{
	struct novatek_crypto *crypto;

	crypto = container_of(ref, struct novatek_crypto, ref);
	kfree(crypto);
}

static void novatek_crypto_alg_destroy(struct crypto_alg *base)
{
	struct novatek_crypto_alg *alg;

	alg = container_of(base, struct novatek_crypto_alg, alg.base.base);
	WRITE_ONCE(alg->released, true);
	kref_put(&alg->crypto->ref, novatek_crypto_release);
}

static int novatek_crypto_init_tfm(struct crypto_skcipher *tfm)
{
	struct novatek_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct novatek_crypto_alg *alg;
	const char *name = crypto_tfm_alg_name(&tfm->base);
	uint reqsize = sizeof(struct novatek_crypto_reqctx);

	alg = container_of(crypto_skcipher_alg(tfm), struct novatek_crypto_alg, alg.base);
	if (alg->type == NOVATEK_CRYPTO_AES) {
		ctx->fallback = crypto_alloc_skcipher(name,
						      0, CRYPTO_ALG_ASYNC |
						      CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(ctx->fallback))
			return PTR_ERR(ctx->fallback);
	}

	ctx->crypto = alg->crypto;
	ctx->type = alg->type;
	ctx->mode = alg->mode;
	ctx->blocksize = alg->alg.base.chunksize;

	kref_get(&ctx->crypto->ref);

	if (ctx->fallback)
		reqsize += crypto_skcipher_reqsize(ctx->fallback);
	crypto_skcipher_set_reqsize(tfm, reqsize);
	return 0;
}

static void novatek_crypto_exit_tfm(struct crypto_skcipher *tfm)
{
	struct novatek_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);

	memzero_explicit(ctx->key, sizeof(ctx->key));
	if (ctx->fallback)
		crypto_free_skcipher(ctx->fallback);
	kref_put(&ctx->crypto->ref, novatek_crypto_release);
}

static int novatek_crypto_setkey(struct crypto_skcipher *tfm, const u8 *key, unsigned int keylen)
{
	struct novatek_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);
	int ret = 0;

	ctx->keylen = 0;
	memzero_explicit(ctx->key, sizeof(ctx->key));

	switch (ctx->type) {
	case NOVATEK_CRYPTO_DES:
		ret = verify_skcipher_des_key(tfm, key);
		ctx->hwmode = CRYPTO_CONFIG_DES;
		break;

	case NOVATEK_CRYPTO_DES3:
		ret = verify_skcipher_des3_key(tfm, key);
		ctx->hwmode = CRYPTO_CONFIG_DES3;
		break;

	case NOVATEK_CRYPTO_AES:
		if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
		    keylen != AES_KEYSIZE_256)
			return -EINVAL;

		crypto_skcipher_clear_flags(ctx->fallback, CRYPTO_TFM_REQ_MASK);
		crypto_skcipher_set_flags(ctx->fallback,
					  crypto_skcipher_get_flags(tfm) & CRYPTO_TFM_REQ_MASK);
		ret = crypto_skcipher_setkey(ctx->fallback, key, keylen);
		ctx->hwmode = keylen == AES_KEYSIZE_128 ?
			     CRYPTO_CONFIG_AES128 : CRYPTO_CONFIG_AES256;
		break;
	}
	if (ret)
		return ret;

	if (ctx->type != NOVATEK_CRYPTO_AES || keylen != AES_KEYSIZE_192)
		memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	return 0;
}

static void novatek_crypto_dma_unmap(struct novatek_crypto *crypto)
{
	struct novatek_crypto_dma_buf *buffers[] = {
		&crypto->input, &crypto->output, &crypto->aad,
	};
	struct novatek_crypto_dma_buf *buf;
	uint i;

	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		buf = buffers[i];
		if (!buf->mapped)
			continue;
		dma_unmap_single(crypto->dev, buf->addr, buf->mapped_size, buf->direction);
		buf->mapped = false;
	}
}

static void novatek_crypto_dma_free(struct novatek_crypto *crypto)
{
	if (crypto->dma_active)
		return;

	novatek_crypto_dma_unmap(crypto);
	kfree_sensitive(crypto->input.data);
	kfree_sensitive(crypto->output.data);
	kfree_sensitive(crypto->aad.data);
	memset(&crypto->input, 0, sizeof(crypto->input));
	memset(&crypto->output, 0, sizeof(crypto->output));
	memset(&crypto->aad, 0, sizeof(crypto->aad));
	memzero_explicit(crypto->desc, sizeof(*crypto->desc));
}

static int novatek_crypto_dma_alloc(struct novatek_crypto_dma_buf *buf, uint len)
{
	size_t size;

	if (!len)
		return 0;
	if (len > SIZE_MAX - 3)
		return -ENOMEM;
	size = ALIGN((size_t)len, sizeof(u32));
	buf->data = kzalloc(size, GFP_KERNEL | __GFP_NOWARN);
	return buf->data ? 0 : -ENOMEM;
}

static int novatek_crypto_dma_map(struct novatek_crypto *crypto, struct novatek_crypto_dma_buf *buf,
				  uint len, enum dma_data_direction direction)
{
	if (!len)
		return 0;
	buf->mapped_size = ALIGN((size_t)len, sizeof(u32));
	buf->direction = direction;
	buf->addr = dma_map_single(crypto->dev, buf->data, buf->mapped_size, direction);
	if (dma_mapping_error(crypto->dev, buf->addr))
		return -ENOMEM;
	buf->mapped = true;
	return 0;
}

static int novatek_crypto_dma_reset(struct novatek_crypto *crypto)
{
	u32 status;
	int ret;

	writel(CRYPTO_CONFIG_DISABLED, crypto->base + CRYPTO_CONFIG_REG);
	writel(CRYPTO_CONFIG_RESET, crypto->base + CRYPTO_CONFIG_REG);
	ret = readl_poll_timeout(crypto->base + CRYPTO_CONFIG_REG,
				 status, !(status & CRYPTO_CONFIG_RESET),
				 1, CRYPTO_TIMEOUT_US);
	if (!ret) {
		crypto->dma_active = false;
		return 0;
	}

	ret = reset_control_assert(crypto->reset);
	if (ret) {
		crypto->faulted = true;
		return ret;
	}
	reset_control_status(crypto->reset);
	udelay(CRYPTO_RESET_US);
	crypto->dma_active = false;

	ret = reset_control_deassert(crypto->reset);
	if (ret)
		crypto->faulted = true;
	else
		reset_control_status(crypto->reset);
	return ret;
}

static irqreturn_t novatek_crypto_irq(int irq, void *data)
{
	struct novatek_crypto *crypto = data;
	irqreturn_t ret = IRQ_NONE;
	u32 status;

	spin_lock(&crypto->lock);
	status = readl(crypto->base + CRYPTO_INT_STATUS_REG) &
		 readl(crypto->base + CRYPTO_INT_ENABLE_REG) & CRYPTO_INT_STATUS_DMA;
	if (!status)
		goto out_unlock;
	writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);
	writel(status, crypto->base + CRYPTO_INT_STATUS_REG);

	complete(&crypto->complete);
	ret = IRQ_HANDLED;

out_unlock:
	spin_unlock(&crypto->lock);
	return ret;
}

static int novatek_crypto_dma_start(struct novatek_crypto *crypto)
{
	ulong flags, time_left;
	u32 control;

	spin_lock_irqsave(&crypto->lock, flags);
	reinit_completion(&crypto->complete);
	writel(CRYPTO_INT_STATUS_DMA, crypto->base + CRYPTO_INT_STATUS_REG);

	writel(CRYPTO_CONFIG_DEFAULT, crypto->base + CRYPTO_CONFIG_REG);

	control = readl(crypto->base + CRYPTO_AXI_CONTROL_REG);
	control &= ~CRYPTO_AXI_CONTROL_DMA_DISABLE;
	writel(control, crypto->base + CRYPTO_AXI_CONTROL_REG);

	writel(CRYPTO_INT_ENABLE_DMA, crypto->base + CRYPTO_INT_ENABLE_REG);

	dma_wmb();
	writel(crypto->desc_dma, crypto->base + CRYPTO_DMA_ADDRESS_REG);
	crypto->dma_active = true;
	writel(CRYPTO_CONTROL_DMA, crypto->base + CRYPTO_CONTROL_REG);
	spin_unlock_irqrestore(&crypto->lock, flags);

	time_left = wait_for_completion_timeout(&crypto->complete,
						msecs_to_jiffies(CRYPTO_DMA_TIMEOUT_MS));
	writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);
	readl(crypto->base + CRYPTO_INT_ENABLE_REG);
	synchronize_irq(crypto->irq);
	if (!time_left) {
		novatek_crypto_dma_reset(crypto);
		return -ETIMEDOUT;
	}
	crypto->dma_active = false;
	dma_rmb();
	return 0;
}

static void novatek_crypto_dma_header(struct novatek_crypto *crypto, const u8 *key, uint keylen,
				      const u8 *iv, uint ivsize, u32 mode, u32 op, bool decrypt)
{
	struct novatek_crypto_dma_desc *desc = crypto->desc;
	u32 header;

	memzero_explicit(desc, sizeof(*desc));
	memcpy(desc->key, key, keylen);
	if (ivsize)
		memcpy(desc->iv, iv, ivsize);

	header = (decrypt ? CRYPTO_DESC_DIRECTION : 0) |
		 FIELD_PREP(CRYPTO_DESC_MODE, mode) |
		 FIELD_PREP(CRYPTO_DESC_OP, op);
	desc->header = cpu_to_le32(header);

	if (op == CRYPTO_DESC_CTR)
		desc->counter[ivsize / sizeof(u32) - 1] = cpu_to_le32(1);
}

static void novatek_crypto_wipe(struct novatek_crypto *crypto)
{
	uint i;

	for (i = 0; i < AES_MAX_KEY_SIZE; i += sizeof(u32))
		writel(0, crypto->base + CRYPTO_KEY_REG + i);

	for (i = 0; i < AES_BLOCK_SIZE; i += sizeof(u32))
		writel(0, crypto->base + CRYPTO_INPUT_REG + i);
}

static int novatek_crypto_run_dma(struct skcipher_request *req)
{
	struct novatek_crypto_ctx *ctx;
	struct novatek_crypto_reqctx *rctx = skcipher_request_ctx(req);
	struct novatek_crypto *crypto;
	struct novatek_crypto_dma_block *block;
	struct scatter_walk src, dst;
	u8 iv[AES_BLOCK_SIZE];
	u8 next_iv[AES_BLOCK_SIZE];
	uint remaining = req->cryptlen;
	uint count, blocks, i;
	u32 op;
	u64 available;
	int ret;

	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	crypto = ctx->crypto;
	ret = novatek_crypto_dma_alloc(&crypto->input, CRYPTO_DMA_CHUNK);
	if (ret)
		goto out_wipe_key;

	ret = novatek_crypto_dma_alloc(&crypto->output, CRYPTO_DMA_CHUNK);
	if (ret)
		goto out_wipe_key;

	if (ctx->mode != NOVATEK_CRYPTO_ECB)
		memcpy(iv, req->iv, ctx->blocksize);
	op = ctx->mode == NOVATEK_CRYPTO_ECB ? CRYPTO_DESC_ECB :
	     ctx->mode == NOVATEK_CRYPTO_CBC ? CRYPTO_DESC_CBC : CRYPTO_DESC_CTR;

	scatterwalk_start(&src, req->src);
	scatterwalk_start(&dst, req->dst);

	while (remaining) {
		if (READ_ONCE(crypto->removing)) {
			ret = -ENODEV;
			break;
		}

		count = min_t(uint, remaining, CRYPTO_DMA_CHUNK);
		if (ctx->mode == NOVATEK_CRYPTO_CTR) {
			/*
			 * End before the low word wraps. The next batch starts
			 * with the full counter incremented in software.
			 */
			available = (u64)U32_MAX - get_unaligned_be32(iv + ctx->blocksize - 4) + 1;
			available *= ctx->blocksize;
			count = min_t(u64, count, available);
		}
		memcpy_from_scatterwalk(crypto->input.data, &src, count);
		if (ctx->mode == NOVATEK_CRYPTO_CBC && rctx->decrypt)
			memcpy(next_iv, crypto->input.data + count - ctx->blocksize,
			       ctx->blocksize);

		ret = novatek_crypto_dma_map(crypto, &crypto->input, count, DMA_TO_DEVICE);
		if (ret)
			break;

		ret = novatek_crypto_dma_map(crypto, &crypto->output, count, DMA_FROM_DEVICE);
		if (ret)
			break;

		novatek_crypto_dma_header(crypto,
					  ctx->key, ctx->keylen,
					  iv, ctx->mode == NOVATEK_CRYPTO_ECB ? 0 : ctx->blocksize,
					  ctx->hwmode, op,
					  ctx->mode != NOVATEK_CRYPTO_CTR && rctx->decrypt);

		block = &crypto->desc->blocks[0];
		block->src = cpu_to_le32(crypto->input.addr);
		block->dst = cpu_to_le32(crypto->output.addr);
		block->length = cpu_to_le32(count);
		block->config = cpu_to_le32(CRYPTO_DESC_LAST);

		ret = novatek_crypto_dma_start(crypto);
		if (ret)
			break;

		novatek_crypto_dma_unmap(crypto);

		if (ctx->mode == NOVATEK_CRYPTO_CBC) {
			memcpy(iv, rctx->decrypt ? next_iv :
			       crypto->output.data + count - ctx->blocksize,
			       ctx->blocksize);
		} else if (ctx->mode == NOVATEK_CRYPTO_CTR) {
			blocks = DIV_ROUND_UP(count, ctx->blocksize);
			for (i = ctx->blocksize; i && blocks; i--) {
				blocks += iv[i - 1];
				iv[i - 1] = blocks;
				blocks >>= 8;
			}
		}

		memcpy_to_scatterwalk(&dst, crypto->output.data, count);
		remaining -= count;
		cond_resched();
	}

	if (!ret && ctx->mode != NOVATEK_CRYPTO_ECB)
		memcpy(req->iv, iv, ctx->blocksize);

out_wipe_key:
	if (!crypto->dma_active && !crypto->faulted)
		novatek_crypto_wipe(crypto);
	novatek_crypto_dma_free(crypto);
	memzero_explicit(iv, sizeof(iv));
	memzero_explicit(next_iv, sizeof(next_iv));
	return ret;
}

static int novatek_crypto_block(struct novatek_crypto *crypto,
				const u8 *input, u8 *output, uint blocksize)
{
	u32 status, value;
	uint i;
	int ret;

	for (i = 0; i < AES_BLOCK_SIZE; i += sizeof(u32)) {
		value = i < blocksize ? get_unaligned_le32(input + i) : 0;
		writel(value, crypto->base + CRYPTO_INPUT_REG + i);
	}

	writel(CRYPTO_INT_STATUS_PIO, crypto->base + CRYPTO_INT_STATUS_REG);
	writel(CRYPTO_CONTROL_START, crypto->base + CRYPTO_CONTROL_REG);
	ret = readl_poll_timeout(crypto->base + CRYPTO_INT_STATUS_REG,
				 status, status & CRYPTO_INT_STATUS_PIO,
				 1, CRYPTO_TIMEOUT_US);
	if (ret)
		return ret;

	writel(CRYPTO_INT_STATUS_PIO, crypto->base + CRYPTO_INT_STATUS_REG);

	for (i = 0; i < blocksize; i += sizeof(u32))
		put_unaligned_le32(readl(crypto->base + CRYPTO_OUTPUT_REG + i), output + i);
	return 0;
}

static int novatek_crypto_run(struct skcipher_request *req)
{
	struct novatek_crypto_reqctx *rctx = skcipher_request_ctx(req);
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct novatek_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct novatek_crypto *crypto = ctx->crypto;
	struct scatter_walk src, dst;
	u8 input[AES_BLOCK_SIZE], output[AES_BLOCK_SIZE];
	u8 iv[AES_BLOCK_SIZE];
	uint remaining = req->cryptlen;
	uint blocksize = ctx->blocksize;
	u32 config, status;
	uint count, i;
	int ret;

	if (crypto->faulted)
		return -EIO;

	if (crypto->desc)
		return novatek_crypto_run_dma(req);

	ret = readl_poll_timeout(crypto->base + CRYPTO_CONTROL_REG,
				 status, !(status & CRYPTO_CONTROL_START),
				 1, CRYPTO_TIMEOUT_US);
	if (ret)
		goto out_reset_engine;

	config = CRYPTO_CONFIG_DEFAULT |
		 FIELD_PREP(CRYPTO_CONFIG_MODE, ctx->hwmode) |
		 (ctx->mode != NOVATEK_CRYPTO_CTR && rctx->decrypt ? CRYPTO_CONFIG_DIRECTION : 0);
	writel(config, crypto->base + CRYPTO_CONFIG_REG);

	for (i = 0; i < ctx->keylen; i += sizeof(u32))
		writel(get_unaligned_le32(ctx->key + i), crypto->base + CRYPTO_KEY_REG + i);

	if (ctx->mode != NOVATEK_CRYPTO_ECB)
		memcpy(iv, req->iv, blocksize);

	scatterwalk_start(&src, req->src);
	scatterwalk_start(&dst, req->dst);

	while (remaining) {
		if (READ_ONCE(crypto->removing)) {
			ret = -ENODEV;
			break;
		}

		count = min(remaining, blocksize);
		memcpy_from_scatterwalk(input, &src, count);

		if (ctx->mode == NOVATEK_CRYPTO_CTR) {
			ret = novatek_crypto_block(crypto, iv, output, blocksize);
			if (ret)
				break;
			crypto_xor(output, input, count);

			crypto_inc(iv, blocksize);
		} else {
			if (ctx->mode == NOVATEK_CRYPTO_CBC && !rctx->decrypt)
				crypto_xor(input, iv, blocksize);
			ret = novatek_crypto_block(crypto, input, output, blocksize);
			if (ret)
				break;

			if (ctx->mode == NOVATEK_CRYPTO_CBC) {
				if (rctx->decrypt) {
					crypto_xor(output, iv, blocksize);
					memcpy(iv, input, blocksize);
				} else {
					memcpy(iv, output, blocksize);
				}
			}
		}

		memcpy_to_scatterwalk(&dst, output, count);
		remaining -= count;
		cond_resched();
	}

	if (!ret && ctx->mode != NOVATEK_CRYPTO_ECB)
		memcpy(req->iv, iv, blocksize);

out_reset_engine:
	if (ret) {
		if (reset_control_reset(crypto->reset)) {
			crypto->faulted = true;
			goto out_wipe_buffers;
		}
		writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);
	}
	for (i = 0; i < AES_MAX_KEY_SIZE; i += sizeof(u32))
		writel(0, crypto->base + CRYPTO_KEY_REG + i);
	for (i = 0; i < AES_BLOCK_SIZE; i += sizeof(u32))
		writel(0, crypto->base + CRYPTO_INPUT_REG + i);
out_wipe_buffers:
	memzero_explicit(input, sizeof(input));
	memzero_explicit(output, sizeof(output));
	memzero_explicit(iv, sizeof(iv));
	return ret;
}

static int novatek_crypto_do_request(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req;
	struct novatek_crypto_reqctx *rctx;
	struct novatek_crypto *crypto;
	struct novatek_crypto_ctx *ctx;
	int ret;

	req = container_of(areq, struct skcipher_request, base);
	rctx = skcipher_request_ctx(req);
	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	crypto = ctx->crypto;
	if (READ_ONCE(crypto->removing)) {
		ret = -ENODEV;
	} else if (ctx->type == NOVATEK_CRYPTO_AES && ctx->keylen == AES_KEYSIZE_192) {
		skcipher_request_set_tfm(&rctx->fallback, ctx->fallback);
		skcipher_request_set_callback(&rctx->fallback, 0, NULL, NULL);
		skcipher_request_set_crypt(&rctx->fallback,
					   req->src, req->dst, req->cryptlen,
					   req->iv);
		ret = rctx->decrypt ? crypto_skcipher_decrypt(&rctx->fallback) :
				      crypto_skcipher_encrypt(&rctx->fallback);
	} else {
		ret = novatek_crypto_run(req);
	}

	local_bh_disable();
	crypto_finalize_skcipher_request(engine, req, ret);
	local_bh_enable();

	if (atomic_dec_and_test(&crypto->pending))
		wake_up(&crypto->wait);
	return 0;
}

static int novatek_crypto_crypt(struct skcipher_request *req, bool decrypt)
{
	struct novatek_crypto_reqctx *rctx = skcipher_request_ctx(req);
	struct novatek_crypto_ctx *ctx;
	struct novatek_crypto *crypto;
	ulong flags;
	int ret;

	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	crypto = ctx->crypto;
	if (!req->cryptlen)
		return 0;
	if (ctx->mode != NOVATEK_CRYPTO_CTR && !IS_ALIGNED(req->cryptlen, ctx->blocksize))
		return -EINVAL;

	if (sg_nents_for_len(req->src, req->cryptlen) < 0 ||
	    sg_nents_for_len(req->dst, req->cryptlen) < 0)
		return -EINVAL;

	rctx->decrypt = decrypt;
	spin_lock_irqsave(&crypto->lock, flags);
	if (crypto->removing) {
		ret = -ENODEV;
	} else if (crypto->suspended) {
		ret = -ESHUTDOWN;
	} else {
		atomic_inc(&crypto->pending);
		ret = crypto_transfer_skcipher_request_to_engine(crypto->engine, req);
		if (ret != -EINPROGRESS && ret != -EBUSY) {
			if (atomic_dec_and_test(&crypto->pending))
				wake_up(&crypto->wait);
		}
	}
	spin_unlock_irqrestore(&crypto->lock, flags);
	return ret;
}

static int novatek_crypto_encrypt(struct skcipher_request *req)
{
	return novatek_crypto_crypt(req, false);
}

static int novatek_crypto_decrypt(struct skcipher_request *req)
{
	return novatek_crypto_crypt(req, true);
}

static void novatek_crypto_aead_destroy(struct crypto_alg *base)
{
	struct novatek_crypto *crypto;

	crypto = container_of(base, struct novatek_crypto, aead.base.base);
	WRITE_ONCE(crypto->aead_released, true);
	kref_put(&crypto->ref, novatek_crypto_release);
}

static int novatek_crypto_aead_init(struct crypto_aead *tfm)
{
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);
	struct novatek_crypto *crypto;

	crypto = container_of(crypto_aead_alg(tfm), struct novatek_crypto, aead.base);

	ctx->fallback = crypto_alloc_aead("gcm(aes)",
					  0, CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback))
		return PTR_ERR(ctx->fallback);

	ctx->crypto = crypto;
	kref_get(&crypto->ref);

	crypto_aead_set_reqsize(tfm,
				sizeof(struct novatek_crypto_aead_reqctx) +
				crypto_aead_reqsize(ctx->fallback));
	return 0;
}

static void novatek_crypto_aead_exit(struct crypto_aead *tfm)
{
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);

	memzero_explicit(ctx->key, sizeof(ctx->key));
	crypto_free_aead(ctx->fallback);
	kref_put(&ctx->crypto->ref, novatek_crypto_release);
}

static int novatek_crypto_aead_setkey(struct crypto_aead *tfm, const u8 *key, unsigned int keylen)
{
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);
	int ret;

	ctx->keylen = 0;
	memzero_explicit(ctx->key, sizeof(ctx->key));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256)
		return -EINVAL;

	crypto_aead_clear_flags(ctx->fallback, CRYPTO_TFM_REQ_MASK);
	crypto_aead_set_flags(ctx->fallback, crypto_aead_get_flags(tfm) & CRYPTO_TFM_REQ_MASK);
	ret = crypto_aead_setkey(ctx->fallback, key, keylen);
	if (ret)
		return ret;

	if (keylen != AES_KEYSIZE_192)
		memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	ctx->hwmode = keylen == AES_KEYSIZE_128 ? CRYPTO_CONFIG_AES128 : CRYPTO_CONFIG_AES256;
	return 0;
}

static int novatek_crypto_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);
	int ret;

	ret = crypto_gcm_check_authsize(authsize);
	if (ret)
		return ret;
	return crypto_aead_setauthsize(ctx->fallback, authsize);
}

static int novatek_crypto_aead_fallback(struct aead_request *req)
{
	struct novatek_crypto_aead_reqctx *rctx = aead_request_ctx(req);
	struct novatek_crypto_aead_ctx *ctx;

	ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	aead_request_set_tfm(&rctx->fallback, ctx->fallback);
	aead_request_set_callback(&rctx->fallback, CRYPTO_TFM_REQ_MAY_SLEEP, NULL, NULL);
	aead_request_set_crypt(&rctx->fallback, req->src, req->dst, req->cryptlen, req->iv);
	aead_request_set_ad(&rctx->fallback, req->assoclen);
	if (rctx->decrypt)
		return crypto_aead_decrypt(&rctx->fallback);
	return crypto_aead_encrypt(&rctx->fallback);
}

static int novatek_crypto_aead_run(struct aead_request *req)
{
	struct novatek_crypto_aead_reqctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);
	struct novatek_crypto *crypto = ctx->crypto;
	struct novatek_crypto_dma_desc *desc = crypto->desc;
	struct novatek_crypto_dma_block *block;
	struct scatter_walk src, dst;
	u8 tag[AES_BLOCK_SIZE], expected[AES_BLOCK_SIZE];
	uint authsize = crypto_aead_authsize(tfm);
	uint len = req->cryptlen - (rctx->decrypt ? authsize : 0);
	u32 last;
	int ret;

	if (crypto->faulted)
		return -EIO;

	ret = novatek_crypto_dma_alloc(&crypto->input, len);
	if (ret)
		goto out_free_dma;

	ret = novatek_crypto_dma_alloc(&crypto->output, len);
	if (ret)
		goto out_free_dma;

	ret = novatek_crypto_dma_alloc(&crypto->aad, req->assoclen);
	if (ret)
		goto out_free_dma;

	if (req->assoclen || req->cryptlen)
		scatterwalk_start(&src, req->src);
	if (req->assoclen)
		memcpy_from_scatterwalk(crypto->aad.data, &src, req->assoclen);
	if (len)
		memcpy_from_scatterwalk(crypto->input.data, &src, len);
	if (rctx->decrypt)
		memcpy_from_scatterwalk(expected, &src, authsize);

	ret = novatek_crypto_dma_map(crypto, &crypto->input, len, DMA_TO_DEVICE);
	if (ret)
		goto out_free_dma;

	ret = novatek_crypto_dma_map(crypto, &crypto->output, len, DMA_FROM_DEVICE);
	if (ret)
		goto out_free_dma;

	ret = novatek_crypto_dma_map(crypto, &crypto->aad, req->assoclen, DMA_TO_DEVICE);
	if (ret)
		goto out_free_dma;

	novatek_crypto_dma_header(crypto,
				  ctx->key, ctx->keylen,
				  req->iv, GCM_AES_IV_SIZE,
				  ctx->hwmode, CRYPTO_DESC_GCM, rctx->decrypt);

	last = CRYPTO_DESC_LAST;
	block = &desc->blocks[0];
	block->src = cpu_to_le32(crypto->aad.addr);
	block->length = cpu_to_le32(req->assoclen);
	block->config = cpu_to_le32(last);

	block = &desc->blocks[1];
	block->src = cpu_to_le32(crypto->input.addr);
	block->dst = cpu_to_le32(crypto->output.addr);
	block->length = cpu_to_le32(len);
	block->config = cpu_to_le32(last);

	ret = novatek_crypto_dma_start(crypto);
	if (ret)
		goto out_wipe_key;

	novatek_crypto_dma_unmap(crypto);

	memcpy(tag, desc->s0, sizeof(tag));
	crypto_xor(tag, desc->ghash, sizeof(tag));
	if (rctx->decrypt && crypto_memneq(tag, expected, authsize)) {
		ret = -EBADMSG;
		goto out_wipe_key;
	}

	if (req->assoclen || len || !rctx->decrypt)
		scatterwalk_start(&dst, req->dst);
	if (req->assoclen)
		memcpy_to_scatterwalk(&dst, crypto->aad.data, req->assoclen);
	if (len)
		memcpy_to_scatterwalk(&dst, crypto->output.data, len);
	if (!rctx->decrypt)
		memcpy_to_scatterwalk(&dst, tag, authsize);

out_wipe_key:
	if (!crypto->dma_active && !crypto->faulted)
		novatek_crypto_wipe(crypto);
out_free_dma:
	novatek_crypto_dma_free(crypto);
	memzero_explicit(tag, sizeof(tag));
	memzero_explicit(expected, sizeof(expected));
	return ret;
}

static int novatek_crypto_aead_do_request(struct crypto_engine *engine, void *areq)
{
	struct aead_request *req;
	struct novatek_crypto_aead_ctx *ctx;
	struct novatek_crypto *crypto;
	int ret;

	req = container_of(areq, struct aead_request, base);
	ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	crypto = ctx->crypto;
	if (READ_ONCE(crypto->removing))
		ret = -ENODEV;
	else if (ctx->keylen == AES_KEYSIZE_192)
		ret = novatek_crypto_aead_fallback(req);
	else
		ret = novatek_crypto_aead_run(req);

	local_bh_disable();
	crypto_finalize_aead_request(engine, req, ret);
	local_bh_enable();

	if (atomic_dec_and_test(&crypto->pending))
		wake_up(&crypto->wait);
	return 0;
}

static int novatek_crypto_aead_crypt(struct aead_request *req, bool decrypt)
{
	struct novatek_crypto_aead_reqctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct novatek_crypto_aead_ctx *ctx = crypto_aead_ctx(tfm);
	struct novatek_crypto *crypto = ctx->crypto;
	uint authsize = crypto_aead_authsize(tfm);
	u64 src_len, dst_len;
	ulong flags;
	int ret;

	if (decrypt && req->cryptlen < authsize)
		return -EINVAL;

	src_len = (u64)req->assoclen + req->cryptlen;
	dst_len = src_len + (decrypt ? -(s64)authsize : authsize);
	if ((src_len && sg_nents_for_len(req->src, src_len) < 0) ||
	    (dst_len && sg_nents_for_len(req->dst, dst_len) < 0))
		return -EINVAL;

	rctx->decrypt = decrypt;
	spin_lock_irqsave(&crypto->lock, flags);
	if (crypto->removing) {
		ret = -ENODEV;
	} else if (crypto->suspended) {
		ret = -ESHUTDOWN;
	} else {
		atomic_inc(&crypto->pending);
		ret = crypto_transfer_aead_request_to_engine(crypto->engine, req);
		if (ret != -EINPROGRESS && ret != -EBUSY) {
			if (atomic_dec_and_test(&crypto->pending))
				wake_up(&crypto->wait);
		}
	}
	spin_unlock_irqrestore(&crypto->lock, flags);
	return ret;
}

static int novatek_crypto_aead_encrypt(struct aead_request *req)
{
	return novatek_crypto_aead_crypt(req, false);
}

static int novatek_crypto_aead_decrypt(struct aead_request *req)
{
	return novatek_crypto_aead_crypt(req, true);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_crypto_suspend(struct device *dev)
{
	struct novatek_crypto *crypto = dev_get_drvdata(dev);
	ulong flags;
	int ret;

	spin_lock_irqsave(&crypto->lock, flags);
	crypto->suspended = true;
	spin_unlock_irqrestore(&crypto->lock, flags);

	ret = -EBUSY;
	if (!wait_event_timeout(crypto->wait,
				!atomic_read(&crypto->pending),
				msecs_to_jiffies(CRYPTO_PM_TIMEOUT_MS)))
		goto out_restore_requests;
	if (crypto->dma_active)
		goto out_restore_requests;

	ret = crypto_engine_stop(crypto->engine);
	if (ret)
		goto out_restore_requests;

	if (crypto->irq_enabled) {
		writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);
		disable_irq(crypto->irq);
		crypto->irq_enabled = false;
	}

	if (crypto->powered) {
		clk_disable_unprepare(crypto->clk);
		crypto->powered = false;
	}
	return 0;

out_restore_requests:
	spin_lock_irqsave(&crypto->lock, flags);
	crypto->suspended = false;
	spin_unlock_irqrestore(&crypto->lock, flags);
	return ret;
}

static int novatek_crypto_resume(struct device *dev)
{
	struct novatek_crypto *crypto = dev_get_drvdata(dev);
	ulong flags;
	int ret;

	ret = clk_prepare_enable(crypto->clk);
	if (ret)
		return ret;

	ret = reset_control_reset(crypto->reset);
	if (ret)
		goto out_disable_clk;
	crypto->dma_active = false;

	if (crypto->desc)
		novatek_crypto_dma_free(crypto);
	crypto->faulted = false;

	writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);

	ret = crypto_engine_start(crypto->engine);
	if (ret)
		goto out_disable_clk;
	crypto->powered = true;

	if (crypto->desc && !crypto->irq_enabled) {
		enable_irq(crypto->irq);
		crypto->irq_enabled = true;
	}

	spin_lock_irqsave(&crypto->lock, flags);
	crypto->suspended = false;
	spin_unlock_irqrestore(&crypto->lock, flags);
	return 0;

out_disable_clk:
	clk_disable_unprepare(crypto->clk);
	return ret;
}
#endif

static const struct dev_pm_ops novatek_crypto_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_crypto_suspend, novatek_crypto_resume)
};

static const struct {
	const char *name;
	const char *driver_name;
	enum novatek_crypto_type type;
	enum novatek_crypto_mode mode;
} novatek_crypto_algs[] = {
	{ "ecb(aes)", "ecb-aes-novatek", NOVATEK_CRYPTO_AES, NOVATEK_CRYPTO_ECB },
	{ "cbc(aes)", "cbc-aes-novatek", NOVATEK_CRYPTO_AES, NOVATEK_CRYPTO_CBC },
	{ "ctr(aes)", "ctr-aes-novatek", NOVATEK_CRYPTO_AES, NOVATEK_CRYPTO_CTR },
	{ "ecb(des)", "ecb-des-novatek", NOVATEK_CRYPTO_DES, NOVATEK_CRYPTO_ECB },
	{ "cbc(des)", "cbc-des-novatek", NOVATEK_CRYPTO_DES, NOVATEK_CRYPTO_CBC },
	{ "ctr(des)", "ctr-des-novatek", NOVATEK_CRYPTO_DES, NOVATEK_CRYPTO_CTR },
	{ "ecb(des3_ede)", "ecb-des3-novatek", NOVATEK_CRYPTO_DES3, NOVATEK_CRYPTO_ECB },
	{ "cbc(des3_ede)", "cbc-des3-novatek", NOVATEK_CRYPTO_DES3, NOVATEK_CRYPTO_CBC },
	{ "ctr(des3_ede)", "ctr-des3-novatek", NOVATEK_CRYPTO_DES3, NOVATEK_CRYPTO_CTR },
};

static int novatek_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_crypto *crypto;
	struct novatek_crypto_alg *entry;
	struct skcipher_alg *alg;
	struct aead_alg *aead;
	uint blocksize, i;
	int ret;

	crypto = kzalloc_obj(*crypto);
	if (!crypto)
		return -ENOMEM;

	crypto->dev = dev;
	kref_init(&crypto->ref);
	spin_lock_init(&crypto->lock);
	init_waitqueue_head(&crypto->wait);
	atomic_set(&crypto->pending, 0);
	init_completion(&crypto->complete);
	platform_set_drvdata(pdev, crypto);

	crypto->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(crypto->base)) {
		ret = PTR_ERR(crypto->base);
		goto out_put_crypto;
	}

	crypto->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(crypto->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(crypto->clk), "Failed to get clock\n");
		goto out_put_crypto;
	}

	crypto->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(crypto->reset)) {
		ret = dev_err_probe(dev, PTR_ERR(crypto->reset), "Failed to get reset\n");
		goto out_put_crypto;
	}

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			goto out_put_crypto;

		crypto->irq = platform_get_irq(pdev, 0);
		if (crypto->irq < 0) {
			ret = crypto->irq;
			goto out_put_crypto;
		}

		crypto->desc = dma_alloc_coherent(dev,
						  sizeof(*crypto->desc), &crypto->desc_dma,
						  GFP_KERNEL);
		if (!crypto->desc) {
			ret = -ENOMEM;
			goto out_put_crypto;
		}
	}

	ret = clk_prepare_enable(crypto->clk);
	if (ret)
		goto out_free_dma;

	ret = reset_control_reset(crypto->reset);
	if (ret)
		goto out_stop_dma;

	writel(CRYPTO_INT_ENABLE_DISABLE_ALL, crypto->base + CRYPTO_INT_ENABLE_REG);

	if (crypto->desc) {
		ret = devm_request_irq(dev,
				       crypto->irq, novatek_crypto_irq, 0,
				       dev_name(dev), crypto);
		if (ret)
			goto out_stop_dma;
		crypto->irq_enabled = true;
	}

	crypto->engine = crypto_engine_alloc_init(dev, false);
	if (!crypto->engine) {
		ret = -ENOMEM;
		goto out_free_irq;
	}

	ret = crypto_engine_start(crypto->engine);
	if (ret)
		goto out_exit_engine;
	crypto->powered = true;

	for (i = 0; i < ARRAY_SIZE(crypto->algs); i++) {
		entry = &crypto->algs[i];
		entry->crypto = crypto;
		entry->type = novatek_crypto_algs[i].type;
		entry->mode = novatek_crypto_algs[i].mode;

		entry->alg.op.do_one_request = novatek_crypto_do_request;

		alg = &entry->alg.base;
		strscpy(alg->base.cra_name, novatek_crypto_algs[i].name);
		strscpy(alg->base.cra_driver_name, novatek_crypto_algs[i].driver_name);
		alg->base.cra_priority = CRYPTO_PRIORITY;
		alg->base.cra_flags = CRYPTO_ALG_ASYNC;
		blocksize = entry->type == NOVATEK_CRYPTO_AES ? AES_BLOCK_SIZE : DES_BLOCK_SIZE;
		alg->base.cra_blocksize = entry->mode == NOVATEK_CRYPTO_CTR ? 1 : blocksize;
		alg->base.cra_ctxsize = sizeof(struct novatek_crypto_ctx);
		alg->base.cra_module = THIS_MODULE;
		alg->base.cra_destroy = novatek_crypto_alg_destroy;

		if (entry->type == NOVATEK_CRYPTO_AES) {
			alg->base.cra_flags |= CRYPTO_ALG_NEED_FALLBACK;
			alg->min_keysize = AES_MIN_KEY_SIZE;
			alg->max_keysize = AES_MAX_KEY_SIZE;
		} else {
			alg->min_keysize = entry->type == NOVATEK_CRYPTO_DES ?
					   DES_KEY_SIZE : DES3_EDE_KEY_SIZE;
			alg->max_keysize = alg->min_keysize;
		}
		alg->ivsize = entry->mode == NOVATEK_CRYPTO_ECB ? 0 : blocksize;
		alg->chunksize = blocksize;

		alg->setkey = novatek_crypto_setkey;
		alg->encrypt = novatek_crypto_encrypt;
		alg->decrypt = novatek_crypto_decrypt;
		alg->init = novatek_crypto_init_tfm;
		alg->exit = novatek_crypto_exit_tfm;

		kref_get(&crypto->ref);
		ret = crypto_engine_register_skcipher(&entry->alg);
		if (ret) {
			if (!READ_ONCE(entry->released))
				kref_put(&crypto->ref, novatek_crypto_release);
			goto out_stop_requests;
		}
	}

	if (crypto->desc) {
		aead = &crypto->aead.base;
		strscpy(aead->base.cra_name, "gcm(aes)");
		strscpy(aead->base.cra_driver_name, "gcm-aes-novatek");
		aead->base.cra_priority = CRYPTO_AEAD_PRIORITY;
		aead->base.cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK;
		aead->base.cra_blocksize = 1;
		aead->base.cra_ctxsize = sizeof(struct novatek_crypto_aead_ctx);
		aead->base.cra_module = THIS_MODULE;
		aead->base.cra_destroy = novatek_crypto_aead_destroy;

		aead->ivsize = GCM_AES_IV_SIZE;
		aead->maxauthsize = AES_BLOCK_SIZE;
		aead->chunksize = AES_BLOCK_SIZE;

		aead->init = novatek_crypto_aead_init;
		aead->exit = novatek_crypto_aead_exit;
		aead->setkey = novatek_crypto_aead_setkey;
		aead->setauthsize = novatek_crypto_aead_setauthsize;
		aead->encrypt = novatek_crypto_aead_encrypt;
		aead->decrypt = novatek_crypto_aead_decrypt;
		crypto->aead.op.do_one_request = novatek_crypto_aead_do_request;

		kref_get(&crypto->ref);
		ret = crypto_engine_register_aead(&crypto->aead);
		if (ret) {
			if (!READ_ONCE(crypto->aead_released))
				kref_put(&crypto->ref, novatek_crypto_release);
			goto out_stop_requests;
		}
		crypto->aead_registered = true;
	}
	return 0;

out_stop_requests:
	spin_lock_irq(&crypto->lock);
	crypto->removing = true;
	spin_unlock_irq(&crypto->lock);
	while (i--)
		crypto_engine_unregister_skcipher(&crypto->algs[i].alg);
	wait_event(crypto->wait, !atomic_read(&crypto->pending));
out_exit_engine:
	crypto_engine_exit(crypto->engine);
out_free_irq:
	if (crypto->desc)
		devm_free_irq(dev, crypto->irq, crypto);
out_stop_dma:
	if (crypto->dma_active) {
		if (reset_control_assert(crypto->reset)) {
			dev_err(dev, "DMA not stopped, retaining buffers\n");
			goto out_put_crypto;
		}
		reset_control_status(crypto->reset);
		udelay(CRYPTO_RESET_US);
		crypto->dma_active = false;
	}
	clk_disable_unprepare(crypto->clk);
out_free_dma:
	if (crypto->desc) {
		novatek_crypto_dma_free(crypto);
		dma_free_coherent(dev, sizeof(*crypto->desc), crypto->desc, crypto->desc_dma);
	}
out_put_crypto:
	kref_put(&crypto->ref, novatek_crypto_release);
	return ret;
}

static void novatek_crypto_remove(struct platform_device *pdev)
{
	struct novatek_crypto *crypto = platform_get_drvdata(pdev);
	ulong flags;
	uint i;
	int ret;

	spin_lock_irqsave(&crypto->lock, flags);
	crypto->removing = true;
	spin_unlock_irqrestore(&crypto->lock, flags);

	for (i = 0; i < ARRAY_SIZE(crypto->algs); i++)
		crypto_engine_unregister_skcipher(&crypto->algs[i].alg);
	if (crypto->aead_registered)
		crypto_engine_unregister_aead(&crypto->aead);

	wait_event(crypto->wait, !atomic_read(&crypto->pending));
	crypto_engine_exit(crypto->engine);

	if (crypto->desc) {
		devm_free_irq(&pdev->dev, crypto->irq, crypto);
		if (crypto->dma_active) {
			ret = reset_control_assert(crypto->reset);
			if (ret) {
				dev_err(&pdev->dev, "DMA not stopped, retaining buffers\n");
				goto out_put_crypto;
			}
			reset_control_status(crypto->reset);
			udelay(CRYPTO_RESET_US);
			crypto->dma_active = false;
		}

		novatek_crypto_dma_free(crypto);
		dma_free_coherent(&pdev->dev,
				  sizeof(*crypto->desc), crypto->desc, crypto->desc_dma);
	}

	if (crypto->powered)
		clk_disable_unprepare(crypto->clk);

out_put_crypto:
	kref_put(&crypto->ref, novatek_crypto_release);
}

static const struct of_device_id novatek_crypto_of_match[] = {
	{ .compatible = "novatek,na51089-crypto" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_crypto_of_match);

static struct platform_driver novatek_crypto_driver = {
	.probe = novatek_crypto_probe,
	.remove = novatek_crypto_remove,
	.driver = {
		.name = "novatek-crypto",
		.of_match_table = novatek_crypto_of_match,
		.pm = pm_sleep_ptr(&novatek_crypto_pm_ops),
	},
};
module_platform_driver(novatek_crypto_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek cryptographic accelerator driver");
MODULE_LICENSE("GPL");
