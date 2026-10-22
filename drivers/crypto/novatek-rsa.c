// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <crypto/akcipher.h>
#include <crypto/engine.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/rsa.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#define RSA_CONFIG_REG			0x00
#define RSA_CONTROL_REG			0x04
#define RSA_INT_ENABLE_REG		0x08
#define RSA_INT_STATUS_REG		0x0c
#define RSA_KEY_N_REG			0x10
#define RSA_KEY_N_ADDRESS_REG		0x14
#define RSA_KEY_ED_REG			0x18
#define RSA_KEY_ED_ADDRESS_REG		0x1c
#define RSA_DATA_REG			0x20
#define RSA_DATA_ADDRESS_REG		0x24

/* RSA_CONFIG bit fields */
#define RSA_CONFIG_RESET		BIT(0)
#define RSA_CONFIG_KEY			GENMASK(3, 1)
#define RSA_CONFIG_KEY_256		0x0
#define RSA_CONFIG_KEY_512		0x1
#define RSA_CONFIG_KEY_1024		0x2
#define RSA_CONFIG_KEY_2048		0x3
#define RSA_CONFIG_MODE			GENMASK(5, 4)
#define RSA_CONFIG_MODE_NORMAL		0x0
#define RSA_CONFIG_CLEAR \
	(FIELD_PREP_CONST(RSA_CONFIG_RESET, 0) | \
	 FIELD_PREP_CONST(RSA_CONFIG_KEY, RSA_CONFIG_KEY_256) | \
	 FIELD_PREP_CONST(RSA_CONFIG_MODE, RSA_CONFIG_MODE_NORMAL))

/* RSA_CONTROL bit fields */
#define RSA_CONTROL_ENABLE		BIT(0)
#define RSA_CONTROL_STOP \
	(FIELD_PREP_CONST(RSA_CONTROL_ENABLE, 0))
#define RSA_CONTROL_START \
	(FIELD_PREP_CONST(RSA_CONTROL_ENABLE, 1))

/* RSA_INT_ENABLE bit fields */
#define RSA_INT_ENABLE_DONE		BIT(0)
#define RSA_INT_ENABLE_DISABLE_ALL \
	(FIELD_PREP_CONST(RSA_INT_ENABLE_DONE, 0))

/* RSA_INT_STATUS bit fields */
#define RSA_INT_STATUS_DONE		BIT(0)

#define RSA_RESET_TIMEOUT_US		100
#define RSA_TIMEOUT_US			5000000
#define RSA_PM_TIMEOUT_MS		10000

struct novatek_rsa_soc_data {
	uint max_size;
	uint sram_size;
};

struct novatek_rsa_dev {
	const struct novatek_rsa_soc_data *data;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct crypto_engine *engine;
	struct akcipher_engine_alg alg;
	struct kref ref;
	spinlock_t lock;
	wait_queue_head_t wait;
	atomic_t pending;
	atomic_t alg_ref;
	bool accepting;
	bool removing;
	bool faulted;
	bool powered;
};

struct novatek_rsa_key {
	u8 *n;
	u8 *e;
	u8 *d;
	u8 *limit;
	uint size;
	u32 width;
	bool hardware;
	bool private;
};

struct novatek_rsa_ctx {
	struct novatek_rsa_dev *rsa;
	struct crypto_akcipher *fallback;
	struct novatek_rsa_key key;
	u8 key_data[];
};

struct novatek_rsa_request_ctx {
	bool encrypt;
	u8 buffer[];
};

static int novatek_rsa_reset(struct novatek_rsa_dev *rsa)
{
	u32 value;
	int ret;

	writel(RSA_CONTROL_STOP, rsa->base + RSA_CONTROL_REG);

	writel(RSA_INT_ENABLE_DISABLE_ALL, rsa->base + RSA_INT_ENABLE_REG);

	writel(RSA_CONFIG_CLEAR, rsa->base + RSA_CONFIG_REG);
	writel(RSA_CONFIG_RESET, rsa->base + RSA_CONFIG_REG);
	ret = readl_poll_timeout(rsa->base + RSA_CONFIG_REG,
				 value, !(value & RSA_CONFIG_RESET),
				 1, RSA_RESET_TIMEOUT_US);

	writel(RSA_INT_STATUS_DONE, rsa->base + RSA_INT_STATUS_REG);
	return ret;
}

static void novatek_rsa_write_operand(struct novatek_rsa_dev *rsa, uint address, uint data,
				      const u8 *buffer)
{
	uint i;
	u32 value;

	writel(0, rsa->base + address);
	for (i = 0; i < rsa->data->sram_size; i += sizeof(u32)) {
		value = get_unaligned_be32(buffer + rsa->data->max_size - i - sizeof(u32));
		writel(value, rsa->base + data);
	}
}

static void novatek_rsa_clear(struct novatek_rsa_dev *rsa)
{
	uint i;

	writel(RSA_CONTROL_STOP, rsa->base + RSA_CONTROL_REG);

	writel(0, rsa->base + RSA_KEY_N_ADDRESS_REG);
	writel(0, rsa->base + RSA_KEY_ED_ADDRESS_REG);
	writel(0, rsa->base + RSA_DATA_ADDRESS_REG);
	for (i = 0; i < rsa->data->sram_size; i += sizeof(u32)) {
		writel(0, rsa->base + RSA_KEY_N_REG);
		writel(0, rsa->base + RSA_KEY_ED_REG);
		writel(0, rsa->base + RSA_DATA_REG);
	}
}

static int novatek_rsa_power_on(struct novatek_rsa_dev *rsa)
{
	int ret;

	ret = clk_prepare_enable(rsa->clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(rsa->reset);
	if (ret)
		goto out_disable_clk;

	ret = novatek_rsa_reset(rsa);
	if (ret)
		goto out_assert_reset;

	novatek_rsa_clear(rsa);
	rsa->faulted = false;
	rsa->powered = true;
	return 0;

out_assert_reset:
	reset_control_assert(rsa->reset);
out_disable_clk:
	clk_disable_unprepare(rsa->clk);
	return ret;
}

static void novatek_rsa_power_off(struct novatek_rsa_dev *rsa)
{
	int ret;

	if (!rsa->powered)
		return;

	ret = reset_control_assert(rsa->reset);
	if (ret)
		dev_warn(rsa->dev, "Failed to assert reset: %d\n", ret);

	clk_disable_unprepare(rsa->clk);
	rsa->powered = false;
}

static int novatek_rsa_fallback(struct akcipher_request *req, bool encrypt)
{
	struct novatek_rsa_ctx *ctx;
	struct akcipher_request *subreq;
	int ret;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	subreq = akcipher_request_alloc(ctx->fallback, GFP_KERNEL);
	if (!subreq)
		return -ENOMEM;

	akcipher_request_set_callback(subreq, CRYPTO_TFM_REQ_MAY_SLEEP, NULL, NULL);
	akcipher_request_set_crypt(subreq, req->src, req->dst, req->src_len, ctx->key.size);
	ret = encrypt ? crypto_akcipher_encrypt(subreq) : crypto_akcipher_decrypt(subreq);
	req->dst_len = subreq->dst_len;

	akcipher_request_free(subreq);
	return ret;
}

static int novatek_rsa_run(struct novatek_rsa_dev *rsa, struct akcipher_request *req, bool encrypt)
{
	struct novatek_rsa_request_ctx *rctx = akcipher_request_ctx(req);
	struct novatek_rsa_ctx *ctx;
	struct novatek_rsa_key *key;
	u8 *buffer = rctx->buffer;
	uint max_size = rsa->data->max_size;
	uint offset, i;
	u32 value;
	int nents, ret;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	key = &ctx->key;

	if (rsa->faulted)
		return -EIO;

	memset(buffer, 0, max_size);

	nents = sg_nents_for_len(req->src, req->src_len);
	if (nents < 0)
		return nents;
	if (sg_copy_to_buffer(req->src, nents,
			      buffer + max_size - req->src_len, req->src_len) != req->src_len) {
		ret = -EINVAL;
		goto out_wipe_buffer;
	}
	if ((!memchr_inv(buffer, 0, max_size - 1) && buffer[max_size - 1] <= 1) ||
	    memcmp(buffer, key->limit, max_size) >= 0) {
		ret = -EINVAL;
		goto out_wipe_buffer;
	}

	novatek_rsa_write_operand(rsa, RSA_DATA_ADDRESS_REG, RSA_DATA_REG, buffer);
	novatek_rsa_write_operand(rsa, RSA_KEY_N_ADDRESS_REG, RSA_KEY_N_REG, key->n);
	novatek_rsa_write_operand(rsa, RSA_KEY_ED_ADDRESS_REG, RSA_KEY_ED_REG,
				  encrypt ? key->e : key->d);

	value = FIELD_PREP(RSA_CONFIG_KEY, key->width) |
		FIELD_PREP(RSA_CONFIG_MODE, RSA_CONFIG_MODE_NORMAL);
	writel(value, rsa->base + RSA_CONFIG_REG);

	writel(RSA_INT_STATUS_DONE, rsa->base + RSA_INT_STATUS_REG);
	writel(RSA_CONTROL_START, rsa->base + RSA_CONTROL_REG);
	ret = readl_poll_timeout(rsa->base + RSA_INT_STATUS_REG,
				 value, value & RSA_INT_STATUS_DONE,
				 100, RSA_TIMEOUT_US);
	if (ret) {
		if (novatek_rsa_reset(rsa)) {
			reset_control_assert(rsa->reset);
			rsa->faulted = true;
		}
		goto out_clear_sram;
	}

	offset = max_size - key->size;
	writel(0, rsa->base + RSA_DATA_ADDRESS_REG);
	for (i = key->size; i; i -= sizeof(u32)) {
		value = readl(rsa->base + RSA_DATA_REG);
		put_unaligned_be32(value, buffer + offset + i - sizeof(u32));
	}

	nents = sg_nents_for_len(req->dst, key->size);
	if (nents < 0) {
		ret = nents;
		goto out_clear_sram;
	}
	if (sg_copy_from_buffer(req->dst, nents, buffer + offset, key->size) != key->size) {
		ret = -EINVAL;
		goto out_clear_sram;
	}
	req->dst_len = key->size;

out_clear_sram:
	if (!rsa->faulted)
		novatek_rsa_clear(rsa);
out_wipe_buffer:
	memzero_explicit(buffer, max_size);
	return ret;
}

static int novatek_rsa_do_request(struct crypto_engine *engine, void *areq)
{
	struct akcipher_request *req = areq;
	struct novatek_rsa_request_ctx *rctx = akcipher_request_ctx(req);
	struct novatek_rsa_ctx *ctx;
	struct novatek_rsa_dev *rsa;
	int ret;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	rsa = ctx->rsa;
	if (ctx->key.hardware)
		ret = novatek_rsa_run(rsa, req, rctx->encrypt);
	else
		ret = novatek_rsa_fallback(req, rctx->encrypt);

	local_bh_disable();
	crypto_finalize_akcipher_request(engine, req, ret);
	local_bh_enable();

	if (atomic_dec_and_test(&rsa->pending))
		wake_up_all(&rsa->wait);
	return 0;
}

static int novatek_rsa_crypt(struct akcipher_request *req, bool encrypt)
{
	struct novatek_rsa_request_ctx *rctx = akcipher_request_ctx(req);
	struct novatek_rsa_ctx *ctx;
	struct novatek_rsa_dev *rsa;
	ulong flags;
	int ret;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	rsa = ctx->rsa;
	if (!ctx->key.size || (!encrypt && !ctx->key.private))
		return -EINVAL;

	if (req->dst_len < ctx->key.size) {
		req->dst_len = ctx->key.size;
		return -EOVERFLOW;
	}

	if (!req->src || !req->dst || !req->src_len || req->src_len > ctx->key.size)
		return -EINVAL;

	rctx->encrypt = encrypt;
	spin_lock_irqsave(&rsa->lock, flags);
	if (!rsa->accepting) {
		ret = -ESHUTDOWN;
		goto out_unlock;
	}

	atomic_inc(&rsa->pending);
	ret = crypto_transfer_akcipher_request_to_engine(rsa->engine, req);
	if (ret != -EINPROGRESS && ret != -EBUSY && atomic_dec_and_test(&rsa->pending))
		wake_up_all(&rsa->wait);

out_unlock:
	spin_unlock_irqrestore(&rsa->lock, flags);
	return ret;
}

static int novatek_rsa_encrypt(struct akcipher_request *req)
{
	return novatek_rsa_crypt(req, true);
}

static int novatek_rsa_decrypt(struct akcipher_request *req)
{
	return novatek_rsa_crypt(req, false);
}

static void novatek_rsa_clear_key(struct novatek_rsa_ctx *ctx)
{
	struct novatek_rsa_key *key = &ctx->key;
	uint max_size = ctx->rsa->data->max_size;

	memzero_explicit(ctx->key_data, 4 * max_size);
	memzero_explicit(key, sizeof(*key));
	key->n = ctx->key_data;
	key->e = key->n + max_size;
	key->d = key->e + max_size;
	key->limit = key->d + max_size;
}

static int novatek_rsa_copy_key(u8 *dst, const u8 *src, size_t len, uint size, uint max_size)
{
	while (len && !*src) {
		src++;
		len--;
	}
	if (!len || len > size)
		return -EINVAL;
	memcpy(dst + max_size - len, src, len);
	return 0;
}

static int novatek_rsa_set_key(struct crypto_akcipher *tfm, const void *data,
			       unsigned int len, bool private)
{
	struct novatek_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct novatek_rsa_key *key = &ctx->key;
	struct rsa_key raw = {};
	uint max_size = ctx->rsa->data->max_size;
	uint i;
	int ret;

	novatek_rsa_clear_key(ctx);

	if (private)
		ret = crypto_akcipher_set_priv_key(ctx->fallback, data, len);
	else
		ret = crypto_akcipher_set_pub_key(ctx->fallback, data, len);
	if (ret)
		return ret;

	key->size = crypto_akcipher_maxsize(ctx->fallback);
	key->private = private;
	switch (key->size) {
	case 64:
		key->width = RSA_CONFIG_KEY_512;
		break;
	case 128:
		key->width = RSA_CONFIG_KEY_1024;
		break;
	case 256:
		key->width = RSA_CONFIG_KEY_2048;
		break;
	case 192:
	case 384:
	case 512:
		return 0;
	default:
		ret = -EINVAL;
		goto out_wipe_key;
	}

	if (private)
		ret = rsa_parse_priv_key(&raw, data, len);
	else
		ret = rsa_parse_pub_key(&raw, data, len);
	if (ret)
		goto out_wipe_key;

	ret = novatek_rsa_copy_key(key->n, raw.n, raw.n_sz, key->size, max_size);
	if (ret)
		goto out_wipe_key;

	ret = novatek_rsa_copy_key(key->e, raw.e, raw.e_sz, key->size, max_size);
	if (ret)
		goto out_wipe_key;

	if (private) {
		ret = novatek_rsa_copy_key(key->d, raw.d, raw.d_sz, key->size, max_size);
		if (ret)
			goto out_wipe_key;
	}

	memcpy(key->limit, key->n, max_size);
	for (i = max_size; i; i--)
		if (key->limit[i - 1]--)
			break;
	key->hardware = true;
	return 0;

out_wipe_key:
	novatek_rsa_clear_key(ctx);
	return ret;
}

static int novatek_rsa_set_public_key(struct crypto_akcipher *tfm,
				      const void *key, unsigned int len)
{
	return novatek_rsa_set_key(tfm, key, len, false);
}

static int novatek_rsa_set_private_key(struct crypto_akcipher *tfm,
				       const void *key, unsigned int len)
{
	return novatek_rsa_set_key(tfm, key, len, true);
}

static unsigned int novatek_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct novatek_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	return ctx->key.size;
}

static int novatek_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct novatek_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct akcipher_alg *alg = crypto_akcipher_alg(tfm);

	ctx->rsa = container_of(alg, struct novatek_rsa_dev, alg.base);
	novatek_rsa_clear_key(ctx);

	ctx->fallback = crypto_alloc_akcipher("rsa-generic", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(ctx->fallback))
		return PTR_ERR(ctx->fallback);

	akcipher_set_reqsize(tfm,
			     sizeof(struct novatek_rsa_request_ctx) + ctx->rsa->data->max_size);
	return 0;
}

static void novatek_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	struct novatek_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	memzero_explicit(ctx->key_data, 4 * ctx->rsa->data->max_size);
	memzero_explicit(&ctx->key, sizeof(ctx->key));
	crypto_free_akcipher(ctx->fallback);
}

static void novatek_rsa_release(struct kref *ref)
{
	struct novatek_rsa_dev *rsa;

	rsa = container_of(ref, struct novatek_rsa_dev, ref);
	kfree(rsa);
}

static void novatek_rsa_destroy_alg(struct crypto_alg *alg)
{
	struct novatek_rsa_dev *rsa;

	rsa = container_of(alg, struct novatek_rsa_dev, alg.base.base);
	if (atomic_xchg(&rsa->alg_ref, 0))
		kref_put(&rsa->ref, novatek_rsa_release);
}

static const struct akcipher_engine_alg novatek_rsa_alg = {
	.base = {
		.encrypt = novatek_rsa_encrypt,
		.decrypt = novatek_rsa_decrypt,
		.set_pub_key = novatek_rsa_set_public_key,
		.set_priv_key = novatek_rsa_set_private_key,
		.max_size = novatek_rsa_max_size,
		.init = novatek_rsa_init_tfm,
		.exit = novatek_rsa_exit_tfm,
		.base = {
			.cra_name = "rsa",
			.cra_driver_name = "rsa-novatek",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_ASYNC |
				     CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_NEED_FALLBACK,
			.cra_module = THIS_MODULE,
			.cra_destroy = novatek_rsa_destroy_alg,
		},
	},
	.op = {
		.do_one_request = novatek_rsa_do_request,
	},
};

static int novatek_rsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct novatek_rsa_soc_data *data;
	struct novatek_rsa_dev *rsa;
	int ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	rsa = kzalloc_obj(*rsa);
	if (!rsa)
		return -ENOMEM;

	rsa->data = data;
	rsa->dev = dev;
	kref_init(&rsa->ref);
	spin_lock_init(&rsa->lock);
	init_waitqueue_head(&rsa->wait);
	atomic_set(&rsa->pending, 0);
	atomic_set(&rsa->alg_ref, 0);
	platform_set_drvdata(pdev, rsa);

	rsa->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rsa->base)) {
		ret = PTR_ERR(rsa->base);
		goto out_put_rsa;
	}

	rsa->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(rsa->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(rsa->clk), "Failed to get clock\n");
		goto out_put_rsa;
	}

	rsa->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rsa->reset)) {
		ret = dev_err_probe(dev, PTR_ERR(rsa->reset), "Failed to get reset\n");
		goto out_put_rsa;
	}

	ret = novatek_rsa_power_on(rsa);
	if (ret)
		goto out_put_rsa;

	rsa->engine = crypto_engine_alloc_init(dev, false);
	if (!rsa->engine) {
		ret = -ENOMEM;
		goto out_disable_power;
	}

	ret = crypto_engine_start(rsa->engine);
	if (ret)
		goto out_exit_engine;

	rsa->alg = novatek_rsa_alg;
	rsa->alg.base.base.cra_ctxsize = sizeof(struct novatek_rsa_ctx) + 4 * data->max_size;
	rsa->accepting = true;
	atomic_set(&rsa->alg_ref, 1);

	kref_get(&rsa->ref);
	ret = crypto_engine_register_akcipher(&rsa->alg);
	if (ret) {
		novatek_rsa_destroy_alg(&rsa->alg.base.base);
		goto out_exit_engine;
	}
	return 0;

out_exit_engine:
	crypto_engine_exit(rsa->engine);
out_disable_power:
	novatek_rsa_power_off(rsa);
out_put_rsa:
	kref_put(&rsa->ref, novatek_rsa_release);
	return ret;
}

static void novatek_rsa_remove(struct platform_device *pdev)
{
	struct novatek_rsa_dev *rsa = platform_get_drvdata(pdev);
	ulong flags;

	spin_lock_irqsave(&rsa->lock, flags);
	rsa->accepting = false;
	rsa->removing = true;
	spin_unlock_irqrestore(&rsa->lock, flags);

	wait_event(rsa->wait, !atomic_read(&rsa->pending));
	crypto_engine_exit(rsa->engine);

	novatek_rsa_power_off(rsa);

	crypto_engine_unregister_akcipher(&rsa->alg);
	kref_put(&rsa->ref, novatek_rsa_release);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_rsa_suspend(struct device *dev)
{
	struct novatek_rsa_dev *rsa = dev_get_drvdata(dev);
	ulong flags;
	int ret;

	spin_lock_irqsave(&rsa->lock, flags);
	rsa->accepting = false;
	spin_unlock_irqrestore(&rsa->lock, flags);

	if (!wait_event_timeout(rsa->wait,
				!atomic_read(&rsa->pending),
				msecs_to_jiffies(RSA_PM_TIMEOUT_MS))) {
		ret = -EBUSY;
		goto out_restore_requests;
	}

	ret = crypto_engine_stop(rsa->engine);
	if (ret)
		goto out_restore_requests;

	novatek_rsa_power_off(rsa);
	return 0;

out_restore_requests:
	spin_lock_irqsave(&rsa->lock, flags);
	rsa->accepting = true;
	spin_unlock_irqrestore(&rsa->lock, flags);
	return ret;
}

static int novatek_rsa_resume(struct device *dev)
{
	struct novatek_rsa_dev *rsa = dev_get_drvdata(dev);
	ulong flags;
	int ret;

	ret = novatek_rsa_power_on(rsa);
	if (ret)
		return ret;

	ret = crypto_engine_start(rsa->engine);
	if (ret) {
		novatek_rsa_power_off(rsa);
		return ret;
	}

	spin_lock_irqsave(&rsa->lock, flags);
	if (!rsa->removing)
		rsa->accepting = true;
	spin_unlock_irqrestore(&rsa->lock, flags);
	return 0;
}
#endif

static const struct dev_pm_ops novatek_rsa_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_rsa_suspend, novatek_rsa_resume)
};

static const struct novatek_rsa_soc_data na51089_rsa_data = {
	.max_size = 512,
	.sram_size = 256,
};

static const struct of_device_id novatek_rsa_of_match[] = {
	{
		.compatible = "novatek,na51089-rsa",
		.data = &na51089_rsa_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_rsa_of_match);

static struct platform_driver novatek_rsa_driver = {
	.probe = novatek_rsa_probe,
	.remove = novatek_rsa_remove,
	.driver = {
		.name = "novatek-rsa",
		.of_match_table = novatek_rsa_of_match,
		.pm = pm_sleep_ptr(&novatek_rsa_pm_ops),
	},
};
module_platform_driver(novatek_rsa_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek RSA accelerator driver");
MODULE_LICENSE("GPL");
