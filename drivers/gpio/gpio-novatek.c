// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <dt-bindings/gpio/novatek,na51089-gpio.h>

#define GPIO_BANK_WIDTH			32

#define GPIO_BANK_STRIDE		0x4
#define GPIO_BANK_OFS(pin) \
	(((pin) / GPIO_BANK_WIDTH) * GPIO_BANK_STRIDE)

#define GPIO_DATA_BASE			0x00
#define GPIO_DATA_OFS(pin) \
	(GPIO_DATA_BASE + GPIO_BANK_OFS(pin))
#define GPIO_DATA_MASK(bit)		BIT(bit)
#define GPIO_DATA_LOW			0x0
#define GPIO_DATA_HIGH			0x1

#define GPIO_DIR_BASE			0x20
#define GPIO_DIR_OFS(pin) \
	(GPIO_DIR_BASE + GPIO_BANK_OFS(pin))
#define GPIO_DIR_MASK(bit)		BIT(bit)
#define GPIO_DIR_INPUT			0x0
#define GPIO_DIR_OUTPUT			0x1

#define GPIO_SET_BASE			0x40
#define GPIO_SET_OFS(pin) \
	(GPIO_SET_BASE + GPIO_BANK_OFS(pin))
#define GPIO_SET_MASK(bit)		BIT(bit)
#define GPIO_SET_KEEP			0x0
#define GPIO_SET_HIGH			0x1

#define GPIO_CLR_BASE			0x60
#define GPIO_CLR_OFS(pin) \
	(GPIO_CLR_BASE + GPIO_BANK_OFS(pin))
#define GPIO_CLR_MASK(bit)		BIT(bit)
#define GPIO_CLR_KEEP			0x0
#define GPIO_CLR_LOW			0x1

#define GPIO_IRQ_STATUS_OFS		0x80
#define DGPIO_IRQ_STATUS_OFS		0xc0
#define GPIO_IRQ_STATUS_MASK(bit)	BIT(bit)
#define GPIO_IRQ_STATUS_KEEP		0x0
#define GPIO_IRQ_STATUS_CLEAR		0x1
#define GPIO_IRQ_STATUS_NONE		0x0
#define GPIO_IRQ_STATUS_PENDING		0x1

#define GPIO_IRQ_ENABLE_OFS		0x90
#define DGPIO_IRQ_ENABLE_OFS		0xd0
#define GPIO_IRQ_ENABLE_MASK(bit)	BIT(bit)
#define GPIO_IRQ_DISABLE		0x0
#define GPIO_IRQ_ENABLE			0x1
#define GPIO_IRQ_DISABLE_ALL		0x0

#define GPIO_IRQ_TYPE_OFS		0xa0
#define DGPIO_IRQ_TYPE_OFS		0xe0
#define GPIO_IRQ_TYPE_MASK(bit)		BIT(bit)
#define GPIO_IRQ_TYPE_EDGE		0x0
#define GPIO_IRQ_TYPE_LEVEL		0x1

#define GPIO_IRQ_POLARITY_OFS		0xa4
#define DGPIO_IRQ_POLARITY_OFS		0xe4
#define GPIO_IRQ_POLARITY_MASK(bit)	BIT(bit)
#define GPIO_IRQ_POLARITY_NORMAL	0x0
#define GPIO_IRQ_POLARITY_INVERTED	0x1

#define GPIO_IRQ_EDGE_OFS		0xa8
#define DGPIO_IRQ_EDGE_OFS		0xe8
#define GPIO_IRQ_EDGE_MASK(bit)		BIT(bit)
#define GPIO_IRQ_EDGE_SINGLE		0x0
#define GPIO_IRQ_EDGE_BOTH		0x1

#define NA51089_CGPIO_NUM_PINS		23
#define NA51089_PGPIO_NUM_PINS		26
#define NA51089_SGPIO_NUM_PINS		9
#define NA51089_LGPIO_NUM_PINS		10
#define NA51089_DGPIO_NUM_PINS		8
#define NA51089_HGPIO_NUM_PINS		12
#define NA51089_AGPIO_NUM_PINS		3
#define NA51089_DSIGPIO_NUM_PINS	11

struct novatek_gpio_bank {
	uint pin_base;
	u32 valid_mask;
	uint resource;
	u32 data;
	u32 dir;
	u32 set;
	u32 clr;
};

struct novatek_gpio_irq_bank {
	u32 status;
	u32 enable;
	u32 type;
	u32 polarity;
	u32 edge;
	u32 valid_mask;
	uint resource;
	uint pins[GPIO_BANK_WIDTH];
};

struct novatek_gpio_soc_data {
	uint ngpios;
	uint num_resources;
	const struct novatek_gpio_bank *banks;
	uint num_banks;
	const struct novatek_gpio_irq_bank *irq_banks;
	uint num_irq_banks;
};

struct novatek_gpio {
	void __iomem **bases;
	raw_spinlock_t lock;
	const struct novatek_gpio_soc_data *soc;
	struct gpio_chip gc;
};

static const struct novatek_gpio_bank *novatek_gpio_find_bank(
	struct novatek_gpio *gpio, uint offset, uint *bit)
{
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	uint i;

	for (i = 0; i < soc->num_banks; i++) {
		const struct novatek_gpio_bank *bank = &soc->banks[i];

		if (offset < bank->pin_base)
			continue;
		*bit = offset - bank->pin_base;
		if (*bit < GPIO_BANK_WIDTH &&
		    bank->valid_mask & BIT(*bit))
			return bank;
	}

	return NULL;
}

static const struct novatek_gpio_irq_bank *novatek_gpio_find_irq(
	struct novatek_gpio *gpio, uint offset, uint *bit)
{
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	uint i;

	for (i = 0; i < soc->num_irq_banks; i++) {
		const struct novatek_gpio_irq_bank *bank = &soc->irq_banks[i];

		for (*bit = 0; *bit < GPIO_BANK_WIDTH; (*bit)++) {
			if (bank->valid_mask & BIT(*bit) &&
			    bank->pins[*bit] == offset)
				return bank;
		}
	}

	return NULL;
}

static int novatek_gpio_get_direction(struct gpio_chip *gc,
				      unsigned int offset)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_bank *bank;
	uint bit;

	bank = novatek_gpio_find_bank(gpio, offset, &bit);
	if (!bank)
		return -EINVAL;
	return readl(gpio->bases[bank->resource] + bank->dir) &
		GPIO_DIR_MASK(bit) ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int novatek_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_bank *bank;
	ulong flags;
	uint bit;
	u32 reg;

	bank = novatek_gpio_find_bank(gpio, offset, &bit);
	if (!bank)
		return -EINVAL;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	reg = readl(gpio->bases[bank->resource] + bank->dir);
	reg &= ~GPIO_DIR_MASK(bit);
	writel(reg, gpio->bases[bank->resource] + bank->dir);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int novatek_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int value)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_bank *bank;
	ulong flags;
	uint bit;
	u32 reg;

	bank = novatek_gpio_find_bank(gpio, offset, &bit);
	if (!bank)
		return -EINVAL;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel((u32)(value ? GPIO_SET_HIGH : GPIO_CLR_LOW) << bit,
	       gpio->bases[bank->resource] + (value ? bank->set : bank->clr));

	reg = readl(gpio->bases[bank->resource] + bank->dir);
	reg |= (u32)GPIO_DIR_OUTPUT << bit;
	writel(reg, gpio->bases[bank->resource] + bank->dir);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int novatek_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_bank *bank;
	uint bit;

	bank = novatek_gpio_find_bank(gpio, offset, &bit);
	if (!bank)
		return -EINVAL;
	return !!(readl(gpio->bases[bank->resource] + bank->data) &
		  GPIO_DATA_MASK(bit));
}

static int novatek_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int value)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_bank *bank;
	ulong flags;
	uint bit;

	bank = novatek_gpio_find_bank(gpio, offset, &bit);
	if (!bank)
		return -EINVAL;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel((u32)(value ? GPIO_SET_HIGH : GPIO_CLR_LOW) << bit,
	       gpio->bases[bank->resource] + (value ? bank->set : bank->clr));
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int novatek_gpio_init_valid_mask(struct gpio_chip *gc,
					unsigned long *valid_mask,
					unsigned int ngpios)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	uint bit, i;

	bitmap_zero(valid_mask, ngpios);
	for (i = 0; i < soc->num_banks; i++) {
		const struct novatek_gpio_bank *bank = &soc->banks[i];

		for (bit = 0; bit < GPIO_BANK_WIDTH; bit++) {
			if (bank->valid_mask & BIT(bit))
				set_bit(bank->pin_base + bit, valid_mask);
		}
	}

	return 0;
}

static void novatek_gpio_irq_init_valid_mask(struct gpio_chip *gc,
					     unsigned long *valid_mask,
					     unsigned int ngpios)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	uint bit, i;

	bitmap_zero(valid_mask, ngpios);
	for (i = 0; i < soc->num_irq_banks; i++) {
		const struct novatek_gpio_irq_bank *bank = &soc->irq_banks[i];

		for (bit = 0; bit < GPIO_BANK_WIDTH; bit++) {
			if (bank->valid_mask & BIT(bit))
				set_bit(bank->pins[bit], valid_mask);
		}
	}
}

static void novatek_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_irq_bank *bank;
	ulong flags;
	uint bit;

	bank = novatek_gpio_find_irq(gpio, irqd_to_hwirq(d), &bit);
	if (!bank)
		return;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel((u32)GPIO_IRQ_STATUS_CLEAR << bit,
	       gpio->bases[bank->resource] + bank->status);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static void novatek_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_irq_bank *bank;
	ulong flags;
	uint bit;
	u32 reg;

	bank = novatek_gpio_find_irq(gpio, irqd_to_hwirq(d), &bit);
	if (!bank)
		return;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	reg = readl(gpio->bases[bank->resource] + bank->enable);
	reg &= ~GPIO_IRQ_ENABLE_MASK(bit);
	writel(reg, gpio->bases[bank->resource] + bank->enable);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	gpiochip_disable_irq(gc, irqd_to_hwirq(d));
}

static void novatek_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_irq_bank *bank;
	ulong flags;
	uint bit;
	u32 reg;

	bank = novatek_gpio_find_irq(gpio, irqd_to_hwirq(d), &bit);
	if (!bank)
		return;

	gpiochip_enable_irq(gc, irqd_to_hwirq(d));

	raw_spin_lock_irqsave(&gpio->lock, flags);
	reg = readl(gpio->bases[bank->resource] + bank->enable);
	reg |= (u32)GPIO_IRQ_ENABLE << bit;
	writel(reg, gpio->bases[bank->resource] + bank->enable);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static int novatek_gpio_irq_request(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	uint offset = irqd_to_hwirq(d);
	int ret;

	ret = novatek_gpio_direction_input(gc, offset);
	if (ret)
		return ret;

	return gpiochip_reqres_irq(gc, offset);
}

static void novatek_gpio_irq_release(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	gpiochip_relres_irq(gc, irqd_to_hwirq(d));
}

static int novatek_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_irq_bank *bank;
	irq_flow_handler_t handler;
	ulong flags;
	uint bit;
	bool level, inverted, both_edges;
	u32 reg;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		handler = handle_level_irq;
		level = true;
		inverted = false;
		both_edges = false;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		handler = handle_level_irq;
		level = true;
		inverted = true;
		both_edges = false;
		break;
	case IRQ_TYPE_EDGE_RISING:
		handler = handle_edge_irq;
		level = false;
		inverted = false;
		both_edges = false;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		handler = handle_edge_irq;
		level = false;
		inverted = true;
		both_edges = false;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		handler = handle_edge_irq;
		level = false;
		inverted = false;
		both_edges = true;
		break;
	default:
		return -EINVAL;
	}

	bank = novatek_gpio_find_irq(gpio, irqd_to_hwirq(d), &bit);
	if (!bank)
		return -EINVAL;
	raw_spin_lock_irqsave(&gpio->lock, flags);
	reg = readl(gpio->bases[bank->resource] + bank->type);
	reg = level ? reg | ((u32)GPIO_IRQ_TYPE_LEVEL << bit) :
			 reg & ~GPIO_IRQ_TYPE_MASK(bit);
	writel(reg, gpio->bases[bank->resource] + bank->type);

	reg = readl(gpio->bases[bank->resource] + bank->polarity);
	reg = inverted ? reg | ((u32)GPIO_IRQ_POLARITY_INVERTED << bit) :
			 reg & ~GPIO_IRQ_POLARITY_MASK(bit);
	writel(reg, gpio->bases[bank->resource] + bank->polarity);

	reg = readl(gpio->bases[bank->resource] + bank->edge);
	reg = both_edges ? reg | ((u32)GPIO_IRQ_EDGE_BOTH << bit) :
			 reg & ~GPIO_IRQ_EDGE_MASK(bit);
	writel(reg, gpio->bases[bank->resource] + bank->edge);

	writel((u32)GPIO_IRQ_STATUS_CLEAR << bit,
	       gpio->bases[bank->resource] + bank->status);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	irq_set_handler_locked(d, handler);

	return 0;
}

static int novatek_gpio_irq_init_hw(struct gpio_chip *gc)
{
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	ulong flags;
	uint i;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	for (i = 0; i < soc->num_irq_banks; i++) {
		const struct novatek_gpio_irq_bank *bank = &soc->irq_banks[i];

		writel(GPIO_IRQ_DISABLE_ALL,
		       gpio->bases[bank->resource] + bank->enable);
		writel(bank->valid_mask,
		       gpio->bases[bank->resource] + bank->status);
	}
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static void novatek_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct novatek_gpio *gpio = gpiochip_get_data(gc);
	const struct novatek_gpio_soc_data *soc = gpio->soc;
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct irq_domain *domain = gc->irq.domain;
	uint bit, i;

	chained_irq_enter(irqchip, desc);

	for (i = 0; i < soc->num_irq_banks; i++) {
		const struct novatek_gpio_irq_bank *bank = &soc->irq_banks[i];
		u32 pending;

		pending = readl(gpio->bases[bank->resource] + bank->status);
		pending &= readl(gpio->bases[bank->resource] + bank->enable);
		pending &= bank->valid_mask;
		while (pending) {
			bit = __ffs((ulong)pending);
			pending &= ~BIT(bit);
			generic_handle_domain_irq(domain, bank->pins[bit]);
		}
	}

	chained_irq_exit(irqchip, desc);
}

static const struct irq_chip novatek_gpio_irqchip = {
	.irq_ack = novatek_gpio_irq_ack,
	.irq_mask = novatek_gpio_irq_mask,
	.irq_unmask = novatek_gpio_irq_unmask,
	.irq_set_type = novatek_gpio_irq_set_type,
	.irq_request_resources = novatek_gpio_irq_request,
	.irq_release_resources = novatek_gpio_irq_release,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SKIP_SET_WAKE,
};

static const struct novatek_gpio_bank na51089_gpio_banks[] = {
	{
		.pin_base = NA51089_CGPIO(0),
		.valid_mask = GENMASK(NA51089_CGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_CGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_CGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_CGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_CGPIO(0)),
	},
	{
		.pin_base = NA51089_PGPIO(0),
		.valid_mask = GENMASK(NA51089_PGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_PGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_PGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_PGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_PGPIO(0)),
	},
	{
		.pin_base = NA51089_SGPIO(0),
		.valid_mask = GENMASK(NA51089_SGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_SGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_SGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_SGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_SGPIO(0)),
	},
	{
		.pin_base = NA51089_LGPIO(0),
		.valid_mask = GENMASK(NA51089_LGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_LGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_LGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_LGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_LGPIO(0)),
	},
	{
		.pin_base = NA51089_DGPIO(0),
		.valid_mask = GENMASK(NA51089_DGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_DGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_DGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_DGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_DGPIO(0)),
	},
	{
		.pin_base = NA51089_HGPIO(0),
		.valid_mask = GENMASK(NA51089_HGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_HGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_HGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_HGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_HGPIO(0)),
	},
	{
		.pin_base = NA51089_AGPIO(0),
		.valid_mask = GENMASK(NA51089_AGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_AGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_AGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_AGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_AGPIO(0)),
	},
	{
		.pin_base = NA51089_DSIGPIO(0),
		.valid_mask = GENMASK(NA51089_DSIGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.data = GPIO_DATA_OFS(NA51089_DSIGPIO(0)),
		.dir = GPIO_DIR_OFS(NA51089_DSIGPIO(0)),
		.set = GPIO_SET_OFS(NA51089_DSIGPIO(0)),
		.clr = GPIO_CLR_OFS(NA51089_DSIGPIO(0)),
	},
};

static const struct novatek_gpio_irq_bank na51089_gpio_irq_banks[] = {
	{
		.status = GPIO_IRQ_STATUS_OFS,
		.enable = GPIO_IRQ_ENABLE_OFS,
		.type = GPIO_IRQ_TYPE_OFS,
		.polarity = GPIO_IRQ_POLARITY_OFS,
		.edge = GPIO_IRQ_EDGE_OFS,
		.valid_mask = GENMASK(GPIO_BANK_WIDTH - 1, 0),
		.resource = 0,
		.pins = {
			NA51089_CGPIO(3), NA51089_CGPIO(5),
			NA51089_CGPIO(7), NA51089_CGPIO(9),
			NA51089_CGPIO(12), NA51089_CGPIO(14),
			NA51089_CGPIO(16), NA51089_CGPIO(18),
			NA51089_CGPIO(20), NA51089_CGPIO(22),
			NA51089_HGPIO(0), NA51089_HGPIO(11),
			NA51089_SGPIO(1), NA51089_SGPIO(4),
			NA51089_SGPIO(6), NA51089_SGPIO(8),
			NA51089_SGPIO(2), NA51089_PGPIO(3),
			NA51089_PGPIO(7), NA51089_PGPIO(8),
			NA51089_PGPIO(9), NA51089_PGPIO(11),
			NA51089_PGPIO(20), NA51089_PGPIO(17),
			NA51089_PGPIO(18), NA51089_PGPIO(24),
			NA51089_DSIGPIO(1), NA51089_DSIGPIO(6),
			NA51089_DSIGPIO(10), NA51089_LGPIO(4),
			NA51089_LGPIO(8), NA51089_LGPIO(0),
		},
	},
	{
		.status = DGPIO_IRQ_STATUS_OFS,
		.enable = DGPIO_IRQ_ENABLE_OFS,
		.type = DGPIO_IRQ_TYPE_OFS,
		.polarity = DGPIO_IRQ_POLARITY_OFS,
		.edge = DGPIO_IRQ_EDGE_OFS,
		.valid_mask = GENMASK(NA51089_DGPIO_NUM_PINS - 1, 0),
		.resource = 0,
		.pins = {
			[0] = NA51089_DGPIO(0),
			[1] = NA51089_DGPIO(1),
			[2] = NA51089_DGPIO(2),
			[3] = NA51089_DGPIO(3),
			[4] = NA51089_DGPIO(4),
			[5] = NA51089_DGPIO(5),
			[6] = NA51089_DGPIO(6),
			[7] = NA51089_DGPIO(7),
		},
	},
};

static const struct novatek_gpio_soc_data na51089_gpio_soc = {
	.ngpios = NA51089_DSIGPIO(NA51089_DSIGPIO_NUM_PINS - 1) + 1,
	.num_resources = 1,
	.banks = na51089_gpio_banks,
	.num_banks = ARRAY_SIZE(na51089_gpio_banks),
	.irq_banks = na51089_gpio_irq_banks,
	.num_irq_banks = ARRAY_SIZE(na51089_gpio_irq_banks),
};

static int novatek_gpio_probe(struct platform_device *pdev)
{
	const struct novatek_gpio_soc_data *soc;
	struct device *dev = &pdev->dev;
	struct novatek_gpio *gpio;
	struct gpio_irq_chip *girq;
	uint i;
	int irq;

	soc = device_get_match_data(dev);
	if (!soc)
		return -EINVAL;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->bases = devm_kcalloc(dev, soc->num_resources,
				   sizeof(*gpio->bases), GFP_KERNEL);
	if (!gpio->bases)
		return -ENOMEM;

	for (i = 0; i < soc->num_resources; i++) {
		gpio->bases[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(gpio->bases[i]))
			return PTR_ERR(gpio->bases[i]);
	}

	raw_spin_lock_init(&gpio->lock);
	gpio->soc = soc;

	gpio->gc.parent = dev;
	gpio->gc.label = dev_name(dev);
	gpio->gc.base = -1;
	gpio->gc.ngpio = soc->ngpios;
	gpio->gc.can_sleep = false;

	gpio->gc.request = gpiochip_generic_request;
	gpio->gc.free = gpiochip_generic_free;
	gpio->gc.get_direction = novatek_gpio_get_direction;
	gpio->gc.direction_input = novatek_gpio_direction_input;
	gpio->gc.direction_output = novatek_gpio_direction_output;
	gpio->gc.get = novatek_gpio_get;
	gpio->gc.set = novatek_gpio_set;
	gpio->gc.set_config = gpiochip_generic_config;
	gpio->gc.init_valid_mask = novatek_gpio_init_valid_mask;

	if (!soc->num_irq_banks)
		return devm_gpiochip_add_data(dev, &gpio->gc, gpio);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	girq = &gpio->gc.irq;
	gpio_irq_chip_set_chip(girq, &novatek_gpio_irqchip);
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->parent_handler = novatek_gpio_irq_handler;
	girq->parent_handler_data = &gpio->gc;
	girq->num_parents = 1;

	girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;
	girq->parents[0] = irq;
	girq->init_hw = novatek_gpio_irq_init_hw;
	girq->init_valid_mask = novatek_gpio_irq_init_valid_mask;

	return devm_gpiochip_add_data(dev, &gpio->gc, gpio);
}

static const struct of_device_id novatek_gpio_of_match[] = {
	{
		.compatible = "novatek,na51089-gpio",
		.data = &na51089_gpio_soc,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_gpio_of_match);

static struct platform_driver novatek_gpio_driver = {
	.probe = novatek_gpio_probe,
	.driver = {
		.name = "novatek-gpio",
		.of_match_table = novatek_gpio_of_match,
	},
};
module_platform_driver(novatek_gpio_driver);

MODULE_DESCRIPTION("Novatek GPIO driver");
MODULE_LICENSE("GPL");
