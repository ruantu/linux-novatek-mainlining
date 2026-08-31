// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/time64.h>

#include "8250.h"

#define UART_REGSHIFT		2

#define UART_PSR		(0x08 / sizeof(u32))
#define UART_PSR_DIV_MASK	GENMASK(4, 0)
#define UART_PSR_DIV1		0x1

#define UART_MCR_AFC_MASK	BIT(6)
#define UART_MCR_AFC_DISABLE	0x0
#define UART_MCR_AFC_ENABLE	0x1

#define UART_R485R_OFFSET	0x20
#define UART_R485R		(UART_R485R_OFFSET / sizeof(u32))
#define UART_R485R_EN_MASK	BIT(0)
#define UART_R485R_DISABLE	0x0
#define UART_R485R_ENABLE	0x1
#define UART_R485R_SETUP_MASK	GENMASK(15, 4)
#define UART_R485R_HOLD_MASK	GENMASK(31, 20)

struct novatek8250_data {
	struct clk *clk;
	struct reset_control *rst;
	int line;
	uint baud;
};

static void novatek8250_set_divisor(struct uart_port *port, unsigned int baud,
				    unsigned int quot, unsigned int quot_frac)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);

	serial8250_do_set_divisor(port, baud, quot);

	/* DLAB is still set here, exposing the prescaler at offset 0x08. */
	serial_out(up, UART_PSR,
		   FIELD_PREP(UART_PSR_DIV_MASK, UART_PSR_DIV1));

	data->baud = DIV_ROUND_CLOSEST(port->uartclk, 16 * quot);
}

static void novatek8250_set_hw_flow(struct uart_port *port, bool enable)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	if (enable) {
		up->mcr |= FIELD_PREP(UART_MCR_AFC_MASK,
				     UART_MCR_AFC_ENABLE);
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
	} else {
		up->mcr &= ~UART_MCR_AFC_MASK;
		port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	}
}

static u32 novatek8250_delay_to_bits(u32 delay_ms, uint baud)
{
	u64 bits;

	bits = DIV_ROUND_UP_ULL((u64)delay_ms * baud, MSEC_PER_SEC);
	return min_t(u64, bits, FIELD_MAX(UART_R485R_SETUP_MASK));
}

static void novatek8250_write_rs485(struct uart_port *port,
				    struct serial_rs485 *rs485,
				    uint baud)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	u32 setup = 0;
	u32 hold = 0;
	u32 value = FIELD_PREP(UART_R485R_EN_MASK, UART_R485R_DISABLE);

	if (rs485->flags & SER_RS485_ENABLED) {
		if (rs485->delay_rts_before_send)
			setup = novatek8250_delay_to_bits(
				rs485->delay_rts_before_send, baud);
		if (rs485->delay_rts_after_send)
			hold = novatek8250_delay_to_bits(
				rs485->delay_rts_after_send, baud);
		value = FIELD_PREP(UART_R485R_EN_MASK, UART_R485R_ENABLE) |
			FIELD_PREP(UART_R485R_SETUP_MASK, setup) |
			FIELD_PREP(UART_R485R_HOLD_MASK, hold);

		if (setup)
			rs485->delay_rts_before_send =
				DIV_ROUND_CLOSEST_ULL(
					(u64)setup * MSEC_PER_SEC, baud);
		if (hold)
			rs485->delay_rts_after_send =
				DIV_ROUND_CLOSEST_ULL(
					(u64)hold * MSEC_PER_SEC, baud);
	}

	serial_out(up, UART_R485R, value);
}

static int novatek8250_rs485_config(struct uart_port *port,
				    struct ktermios *termios,
				    struct serial_rs485 *rs485)
{
	struct novatek8250_data *data = port->private_data;
	uint baud = data->baud;
	bool enable = rs485->flags & SER_RS485_ENABLED;

	lockdep_assert_held_once(&port->lock);

	if (enable && (rs485->delay_rts_before_send ||
		       rs485->delay_rts_after_send) && !baud)
		return -EINVAL;
	if (enable && termios && termios->c_cflag & CRTSCTS)
		return -EBUSY;

	novatek8250_set_hw_flow(port, false);
	serial8250_do_set_mctrl(port, port->mctrl);

	novatek8250_write_rs485(port, rs485, baud);

	return 0;
}

static const struct serial_rs485 novatek8250_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND,
	.delay_rts_before_send = 1,
	.delay_rts_after_send = 1,
};

static void novatek8250_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    const struct ktermios *old)
{
	struct novatek8250_data *data = port->private_data;
	ulong flags;
	bool flow;

	flow = termios->c_cflag & CRTSCTS && port->flags & UPF_HARD_FLOW &&
	       !(port->rs485.flags & SER_RS485_ENABLED);
	if (!flow)
		termios->c_cflag &= ~CRTSCTS;
	uart_port_lock_irqsave(port, &flags);
	novatek8250_set_hw_flow(port, flow);
	uart_port_unlock_irqrestore(port, flags);

	serial8250_do_set_termios(port, termios, old);

	if (!tty_termios_baud_rate(termios))
		return;

	uart_port_lock_irqsave(port, &flags);
	if (port->rs485.flags & SER_RS485_ENABLED)
		novatek8250_write_rs485(port, &port->rs485, data->baud);
	uart_port_unlock_irqrestore(port, flags);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init novatek8250_early_console_setup(
	struct earlycon_device *device, const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32;
	device->port.regshift = UART_REGSHIFT;

	device->baud = 0;
	return early_serial8250_setup(device, options);
}

OF_EARLYCON_DECLARE(nvt_na51089, "novatek,na51089-uart",
		    novatek8250_early_console_setup);
#endif

static int novatek8250_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek8250_data *data;
	struct uart_8250_port up = {};
	struct resource *regs;
	ulong rate;
	bool has_rtscts;
	bool rs485_at_boot;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return dev_err_probe(dev, -EINVAL, "missing registers\n");
	up.port.membase = devm_ioremap(dev, regs->start, resource_size(regs));
	if (!up.port.membase)
		return -ENOMEM;

	up.port.dev = dev;
	up.port.private_data = data;
	up.port.mapbase = regs->start;
	up.port.mapsize = resource_size(regs);

	ret = uart_read_port_properties(&up.port);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read UART properties\n");

	data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk),
				     "failed to get UART clock\n");
	ret = clk_prepare_enable(data->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable UART clock\n");

	rate = clk_get_rate(data->clk);
	if (!rate) {
		ret = dev_err_probe(dev, -EINVAL,
				    "invalid UART clock rate %lu\n", rate);
		goto err_disable_clock;
	}
	up.port.uartclk = rate;

	data->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(data->rst)) {
		ret = dev_err_probe(dev, PTR_ERR(data->rst),
				    "failed to get UART reset\n");
		goto err_disable_clock;
	}
	ret = reset_control_deassert(data->rst);
	if (ret) {
		ret = dev_err_probe(dev, ret,
				    "failed to deassert UART reset\n");
		goto err_disable_clock;
	}

	has_rtscts = device_property_read_bool(dev, "uart-has-rtscts");
	rs485_at_boot = device_property_read_bool(
		dev, "linux,rs485-enabled-at-boot-time");
	if (has_rtscts || !rs485_at_boot)
		writel(FIELD_PREP(UART_R485R_EN_MASK, UART_R485R_DISABLE),
		       up.port.membase + UART_R485R_OFFSET);

	device_property_read_u32(dev, "current-speed", &data->baud);

	up.port.type = PORT_NOVATEK;
	up.port.iotype = UPIO_MEM32;
	up.port.regshift = UART_REGSHIFT;
	up.port.flags |= UPF_FIXED_PORT | UPF_FIXED_TYPE;
	up.bugs |= UART_BUG_NOMSR;

	up.port.set_divisor = novatek8250_set_divisor;
	up.port.set_termios = novatek8250_set_termios;

	if (has_rtscts) {
		up.port.flags |= UPF_HARD_FLOW;
	} else {
		up.port.rs485_config = novatek8250_rs485_config;
		up.port.rs485_supported = novatek8250_rs485_supported;
	}

	ret = serial8250_register_8250_port(&up);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register 8250 port\n");
		goto err_assert_reset;
	}
	data->line = ret;
	platform_set_drvdata(pdev, data);

	return 0;

err_assert_reset:
	reset_control_assert(data->rst);
err_disable_clock:
	clk_disable_unprepare(data->clk);

	return ret;
}

static void novatek8250_remove(struct platform_device *pdev)
{
	struct novatek8250_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);

	reset_control_assert(data->rst);
	clk_disable_unprepare(data->clk);
}

static int __maybe_unused novatek8250_suspend(struct device *dev)
{
	struct novatek8250_data *data = dev_get_drvdata(dev);
	struct uart_8250_port *up = serial8250_get_port(data->line);

	serial8250_suspend_port(data->line);

	if (!uart_console(&up->port) || console_suspend_enabled)
		clk_disable_unprepare(data->clk);

	return 0;
}

static int __maybe_unused novatek8250_resume(struct device *dev)
{
	struct novatek8250_data *data = dev_get_drvdata(dev);
	struct uart_8250_port *up = serial8250_get_port(data->line);
	int ret;

	if (!uart_console(&up->port) || console_suspend_enabled) {
		ret = clk_prepare_enable(data->clk);
		if (ret)
			return ret;
	}

	serial8250_resume_port(data->line);

	return 0;
}

static const struct dev_pm_ops novatek8250_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek8250_suspend, novatek8250_resume)
};

static const struct of_device_id novatek8250_of_match[] = {
	{ .compatible = "novatek,na51089-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek8250_of_match);

static struct platform_driver novatek8250_platform_driver = {
	.probe = novatek8250_probe,
	.remove = novatek8250_remove,
	.driver = {
		.name = "8250_novatek",
		.of_match_table = novatek8250_of_match,
		.pm = &novatek8250_pm_ops,
	},
};
module_platform_driver(novatek8250_platform_driver);

MODULE_DESCRIPTION("Novatek 8250-compatible UART driver");
MODULE_LICENSE("GPL");
