// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/time64.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>

#include "8250.h"

#define UART_PSR_REG				0x08
#define UART_SPR_REG				0x1c
#define UART_R485R_REG				0x20
#define UART_TX_ADDR_REG			0x80
#define UART_TX_SIZE_REG			0x84
#define UART_TX_CURRENT_REG			0x88
#define UART_TX_CTRL_REG			0x8c
#define UART_RX_ADDR_REG			0x90
#define UART_RX_SIZE_REG			0x94
#define UART_RX_CURRENT_REG			0x98
#define UART_RX_CTRL_REG			0x9c
#define UART_DMA_IER_REG			0xa0
#define UART_DMA_ISR_REG			0xa4

/* UART_IER bit fields */
#define UART_IER_DISABLE \
	(FIELD_PREP_CONST(UART_IER_RDI, 0) | \
	 FIELD_PREP_CONST(UART_IER_THRI, 0) | \
	 FIELD_PREP_CONST(UART_IER_RLSI, 0) | \
	 FIELD_PREP_CONST(UART_IER_MSI, 0))

/* UART_PSR bit fields */
#define UART_PSR_DIV				GENMASK(4, 0)
#define UART_PSR_DIV_1				0x01
#define UART_PSR_DEFAULT \
	(FIELD_PREP_CONST(UART_PSR_DIV, UART_PSR_DIV_1))

/* UART_FCR bit fields */
#define UART_FCR_DISABLE \
	(FIELD_PREP_CONST(UART_FCR_ENABLE_FIFO, 0) | \
	 FIELD_PREP_CONST(UART_FCR_CLEAR_RCVR, 0) | \
	 FIELD_PREP_CONST(UART_FCR_CLEAR_XMIT, 0) | \
	 FIELD_PREP_CONST(UART_FCR_DMA_SELECT, 0))

/* UART_MCR bit fields */
#define UART_MCR_AFC				BIT(6)

/* UART_SPR bit fields */
#define UART_SPR_RX_IDLE			BIT(11)

/* UART_R485R bit fields */
#define UART_R485R_EN				BIT(0)
#define UART_R485R_SETUP			GENMASK(15, 4)
#define UART_R485R_HOLD				GENMASK(31, 20)
#define UART_R485R_DISABLED \
	(FIELD_PREP_CONST(UART_R485R_EN, 0) | \
	 FIELD_PREP_CONST(UART_R485R_SETUP, 0) | \
	 FIELD_PREP_CONST(UART_R485R_HOLD, 0))

/* UART_TX_SIZE bit fields */
#define UART_TX_SIZE_COUNT			GENMASK(19, 0)

/* UART_TX_CTRL bit fields */
#define UART_TX_CTRL_EN				BIT(0)
#define UART_TX_CTRL_DISABLED \
	(FIELD_PREP_CONST(UART_TX_CTRL_EN, 0))
#define UART_TX_CTRL_ENABLED \
	(FIELD_PREP_CONST(UART_TX_CTRL_EN, 1))

/* UART_RX_SIZE bit fields */
#define UART_RX_SIZE_COUNT			GENMASK(19, 0)

/* UART_DMA_IER bit fields */
#define UART_DMA_IER_TX_DONE			BIT(0)
#define UART_DMA_IER_RX_DONE			BIT(8)
#define UART_DMA_IER_RX_ERR			BIT(9)
#define UART_DMA_IER_DISABLE_ALL \
	(FIELD_PREP_CONST(UART_DMA_IER_TX_DONE, 0) | \
	 FIELD_PREP_CONST(UART_DMA_IER_RX_DONE, 0) | \
	 FIELD_PREP_CONST(UART_DMA_IER_RX_ERR, 0))
#define UART_DMA_IER_RX				(UART_DMA_IER_RX_DONE | UART_DMA_IER_RX_ERR)

/* UART_DMA_ISR bit fields */
#define UART_DMA_ISR_TX_DONE			BIT(0)
#define UART_DMA_ISR_RX_DONE			BIT(8)
#define UART_DMA_ISR_RX_ERR			BIT(9)
#define UART_DMA_ISR_OVERRUN			BIT(16)
#define UART_DMA_ISR_PARITY			BIT(17)
#define UART_DMA_ISR_FRAME			BIT(18)
#define UART_DMA_ISR_BREAK			BIT(19)
#define UART_DMA_ISR_RX_IRQ			(UART_DMA_ISR_RX_DONE | UART_DMA_ISR_RX_ERR)
#define UART_DMA_ISR_RX_STATUS \
	(UART_DMA_ISR_RX_IRQ | \
	 UART_DMA_ISR_OVERRUN | \
	 UART_DMA_ISR_PARITY | UART_DMA_ISR_FRAME | \
	 UART_DMA_ISR_BREAK)
#define UART_DMA_ISR_CLEAR_RX \
	(FIELD_PREP_CONST(UART_DMA_ISR_RX_DONE, 1) | \
	 FIELD_PREP_CONST(UART_DMA_ISR_RX_ERR, 1) | \
	 FIELD_PREP_CONST(UART_DMA_ISR_OVERRUN, 1) | \
	 FIELD_PREP_CONST(UART_DMA_ISR_PARITY, 1) | \
	 FIELD_PREP_CONST(UART_DMA_ISR_FRAME, 1) | \
	 FIELD_PREP_CONST(UART_DMA_ISR_BREAK, 1))
#define UART_DMA_ISR_CLEAR_ALL \
	(FIELD_PREP_CONST(UART_DMA_ISR_TX_DONE, 1) | \
	 UART_DMA_ISR_CLEAR_RX)

#define UART_PSR_OFFSET				(UART_PSR_REG / sizeof(u32))
#define UART_R485R_OFFSET			(UART_R485R_REG / sizeof(u32))

#define UART_REGSHIFT				2

#define UART_DMA_SIZE				4096

#define UART_DMA_STOP_US			1000
#define UART_DMA_POLL_MS			10
#define UART_RX_IDLE_US				100

struct novatek8250_buffer {
	void *cpu;
	dma_addr_t addr;
};

struct novatek8250_data {
	struct clk *clk;
	struct reset_control *rst;
	int line;
	uint baud;
	struct uart_8250_port *up;
	struct uart_8250_dma dma;
	struct uart_ops ops;
	const struct uart_ops *base_ops;
	struct novatek8250_buffer rx[2];
	struct novatek8250_buffer tx;
	struct timer_list timer;
	/* FCR sysfs lacks port.lock; take io_lock after port.lock if held. */
	spinlock_t io_lock;
	u32 fcr;
	u32 pending_fcr;
	u32 pending_clear;
	bool pending_disable;
	bool fcr_pending;
	bool dma_session;
	uint rx_index;
	uint tx_size;
	u32 dma_ier;
	bool dma_enabled;
	bool opened;
	bool configuring;
	bool suspending;
	bool throttled;
	bool rx_stopped;
	bool rx_allowed;
	bool rx_running;
	bool tx_running;
	bool tx_stopped;
	bool faulted;
	bool reset_asserted;
};

static void novatek8250_dma_fault(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;

	WRITE_ONCE(data->faulted, true);
	data->dma_ier = UART_DMA_IER_DISABLE_ALL;
	writel(data->dma_ier, up->port.membase + UART_DMA_IER_REG);
	writel(UART_IER_DISABLE, up->port.membase + (UART_IER << up->port.regshift));
	dev_err_ratelimited(up->port.dev, "UART DMA fault; port disabled\n");
}

static bool novatek8250_dma_stop(struct uart_8250_port *up, uint reg)
{
	u32 value;
	int ret;

	writel(UART_TX_CTRL_DISABLED, up->port.membase + reg);
	ret = readl_poll_timeout_atomic(up->port.membase + reg,
					value, !(value & UART_TX_CTRL_EN),
					1, UART_DMA_STOP_US);
	if (ret) {
		novatek8250_dma_fault(up);
		return false;
	}
	return true;
}

static int novatek8250_dma_count(struct uart_port *port, struct novatek8250_buffer *buf, uint reg,
				 uint size)
{
	u32 count = readl(port->membase + reg) - lower_32_bits(buf->addr);

	if (count <= size)
		return count;
	return -EIO;
}

static void novatek8250_stop_tx_dma(struct uart_8250_port *up, bool account)
{
	struct novatek8250_data *data = up->port.private_data;
	struct uart_port *port = &up->port;
	u32 status;
	int count;

	if (!data->tx_running)
		return;
	if (!account)
		data->tx_size = 0;

	data->dma_ier &= ~UART_DMA_IER_TX_DONE;
	writel(data->dma_ier, port->membase + UART_DMA_IER_REG);
	if (!novatek8250_dma_stop(up, UART_TX_CTRL_REG))
		return;

	status = readl(port->membase + UART_DMA_ISR_REG);
	if (!account)
		count = 0;
	else if (status & UART_DMA_ISR_TX_DONE)
		count = data->tx_size;
	else
		count = novatek8250_dma_count(port, &data->tx, UART_TX_CURRENT_REG, data->tx_size);
	writel(status & UART_DMA_ISR_TX_DONE, port->membase + UART_DMA_ISR_REG);
	data->tx_running = false;
	data->tx_size = 0;
	if (count < 0) {
		novatek8250_dma_fault(up);
		return;
	}
	if (account && count) {
		uart_xmit_advance(port, count);
		if (kfifo_len(&port->state->port.xmit_fifo) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}
}

static void novatek8250_stop_rx_dma(struct uart_8250_port *up, bool drain)
{
	struct novatek8250_data *data = up->port.private_data;
	struct uart_port *port = &up->port;
	struct tty_port *tport;
	struct novatek8250_buffer *buf = &data->rx[data->rx_index];
	struct irq_data *irqd;
	u32 status;
	uint copied;
	u16 lsr;
	int count;

	if (!data->rx_running)
		return;
	tport = &port->state->port;
	data->dma_ier &= ~UART_DMA_IER_RX;
	writel(data->dma_ier, port->membase + UART_DMA_IER_REG);
	if (!novatek8250_dma_stop(up, UART_RX_CTRL_REG))
		return;

	status = readl(port->membase + UART_DMA_ISR_REG);
	if (status & UART_DMA_ISR_RX_DONE)
		count = UART_DMA_SIZE;
	else
		count = novatek8250_dma_count(port, buf, UART_RX_CURRENT_REG, UART_DMA_SIZE);
	writel(status & UART_DMA_ISR_RX_STATUS, port->membase + UART_DMA_ISR_REG);
	WRITE_ONCE(data->rx_running, false);
	if (count < 0) {
		novatek8250_dma_fault(up);
		return;
	}

	if (status & UART_DMA_ISR_PARITY)
		port->icount.parity++;
	if (status & UART_DMA_ISR_FRAME)
		port->icount.frame++;
	if (status & UART_DMA_ISR_BREAK) {
		port->icount.brk++;
		uart_handle_break(port);
	}
	if (status & UART_DMA_ISR_OVERRUN) {
		port->icount.overrun++;
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
	}

	dma_rmb();
	irqd = irq_get_irq_data(port->irq);
	if (count && irqd && irqd_is_wakeup_set(irqd))
		pm_wakeup_event(tport->tty->dev, 0);
	copied = tty_insert_flip_string(tport, buf->cpu, count);
	port->icount.rx += count;
	port->icount.buf_overrun += count - copied;
	tty_flip_buffer_push(tport);
	if (count)
		data->rx_index ^= 1;

	if (drain) {
		lsr = serial_lsr_in(up);
		if (lsr & (UART_LSR_DR | UART_LSR_BI))
			serial8250_rx_chars(up, lsr);
	}

	serial_out(up, UART_IER, up->ier);
}

static bool novatek8250_rx_dma_allowed(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;

	return data->opened && !data->configuring && !data->suspending &&
		!data->throttled && !data->rx_stopped && data->rx_allowed &&
		!(READ_ONCE(up->port.flags) & UPF_SAK) &&
		!uart_console(&up->port) &&
		!(up->port.rs485.flags & SER_RS485_ENABLED);
}

static int novatek8250_rx_dma(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;
	struct uart_port *port = &up->port;
	struct novatek8250_buffer *buf = &data->rx[data->rx_index];
	uint count = 0;
	u16 lsr;
	int ret;

	if (data->faulted || data->rx_running)
		return 0;
	if (!novatek8250_rx_dma_allowed(up))
		return -EAGAIN;

	writel(lower_32_bits(buf->addr), port->membase + UART_RX_ADDR_REG);
	writel(FIELD_PREP(UART_RX_SIZE_COUNT, UART_DMA_SIZE), port->membase + UART_RX_SIZE_REG);

	writel(UART_DMA_ISR_CLEAR_RX, port->membase + UART_DMA_ISR_REG);

	serial_out(up, UART_IER, up->ier & ~(UART_IER_RDI | UART_IER_RLSI));
	data->dma_ier |= UART_DMA_IER_RX_DONE | UART_DMA_IER_RX_ERR;
	writel(data->dma_ier, port->membase + UART_DMA_IER_REG);

	lsr = serial_lsr_in(up);
	while (count < 256 && lsr & (UART_LSR_DR | UART_LSR_BI)) {
		serial8250_read_char(up, lsr);
		count++;
		lsr = serial_lsr_in(up);
	}
	if (lsr & (UART_LSR_DR | UART_LSR_BI))
		goto out_disable_rx_dma_irq;

	ret = read_poll_timeout_atomic(serial_lsr_in,
				       lsr, (lsr & (UART_LSR_DR | UART_LSR_BI)) ||
					    (readl(port->membase + UART_SPR_REG) &
					     UART_SPR_RX_IDLE),
				       1, UART_RX_IDLE_US, false,
				       up);
	if (ret)
		goto out_disable_rx_dma_irq;

	lsr = serial_lsr_in(up);
	while (count < 256 && lsr & (UART_LSR_DR | UART_LSR_BI)) {
		serial8250_read_char(up, lsr);
		count++;
		lsr = serial_lsr_in(up);
	}
	if (lsr & (UART_LSR_DR | UART_LSR_BI))
		goto out_disable_rx_dma_irq;

	WRITE_ONCE(data->rx_running, true);
	writel(UART_TX_CTRL_ENABLED, port->membase + UART_RX_CTRL_REG);

	ret = 0;
	goto out_push_rx;

out_disable_rx_dma_irq:
	data->dma_ier &= ~UART_DMA_IER_RX;
	writel(data->dma_ier, port->membase + UART_DMA_IER_REG);
	serial_out(up, UART_IER, up->ier);
	ret = -EAGAIN;
out_push_rx:
	if (count)
		tty_flip_buffer_push(&port->state->port);
	return ret;
}

static int novatek8250_tx_dma(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;
	struct uart_port *port = &up->port;
	struct tty_port *tport = &port->state->port;
	u16 lsr;

	if (data->faulted || data->configuring || data->suspending)
		goto out_clear_thri;

	if (port->x_char) {
		novatek8250_stop_tx_dma(up, true);
		if (data->faulted)
			return 0;
		goto out_set_tx_error;
	}

	if (data->tx_running)
		return 0;
	if (data->tx_stopped || uart_tx_stopped(port) || kfifo_is_empty(&tport->xmit_fifo))
		goto out_clear_thri;
	if (uart_console(port) || port->rs485.flags & SER_RS485_ENABLED)
		goto out_set_tx_error;

	if (data->rx_running)
		lsr = serial_in(up, UART_LSR);
	else
		lsr = serial_lsr_in(up);
	if (!(lsr & UART_LSR_THRE))
		goto out_set_tx_error;

	data->tx_size = kfifo_out_peek(&tport->xmit_fifo, data->tx.cpu, UART_DMA_SIZE);
	dma_wmb();

	writel(lower_32_bits(data->tx.addr), port->membase + UART_TX_ADDR_REG);
	writel(FIELD_PREP(UART_TX_SIZE_COUNT, data->tx_size), port->membase + UART_TX_SIZE_REG);

	writel(UART_DMA_ISR_TX_DONE, port->membase + UART_DMA_ISR_REG);

	data->tx_running = true;
	data->dma.tx_err = 0;
	serial8250_clear_THRI(up);
	data->dma_ier |= UART_DMA_IER_TX_DONE;
	writel(data->dma_ier, port->membase + UART_DMA_IER_REG);

	writel(UART_TX_CTRL_ENABLED, port->membase + UART_TX_CTRL_REG);

	return 0;

out_clear_thri:
	serial8250_clear_THRI(up);
	return 0;

out_set_tx_error:
	data->dma.tx_err = 1;
	return -EAGAIN;
}

static int novatek8250_start_tx_dma(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;

	data->tx_stopped = false;
	return novatek8250_tx_dma(up);
}

static void novatek8250_apply_fcr(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	void __iomem *fcr = port->membase + (UART_FCR << port->regshift);

	lockdep_assert_held(&data->io_lock);
	if (data->pending_clear)
		writel(data->pending_fcr | UART_FCR_ENABLE_FIFO | data->pending_clear, fcr);
	if (data->pending_disable && data->pending_fcr & UART_FCR_ENABLE_FIFO)
		writel(UART_FCR_DISABLE, fcr);
	writel(data->pending_fcr, fcr);
	data->fcr = data->pending_fcr;
	data->pending_clear = 0;
	data->pending_disable = false;
	data->fcr_pending = false;
}

static void novatek8250_serial_out(struct uart_port *port, unsigned int reg, u32 value)
{
	struct novatek8250_data *data = port->private_data;
	ulong flags;
	u32 lcr;

	/* Emergency console output may reenter with io_lock already held. */
	if (oops_in_progress && uart_console(port)) {
		writel(value, port->membase + (reg << port->regshift));
		return;
	}

	spin_lock_irqsave(&data->io_lock, flags);
	lcr = readl(port->membase + (UART_LCR << port->regshift));

	if (reg == UART_FCR) {
		if (READ_ONCE(data->faulted))
			goto out_unlock;
		if (data->dma_session) {
			if (!(value & UART_FCR_ENABLE_FIFO))
				goto out_unlock;
			value &= ~(UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		} else {
			data->pending_clear |= value & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
			data->pending_disable |= !(value & UART_FCR_ENABLE_FIFO);
		}
		data->pending_fcr = value & ~(UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		data->fcr_pending = true;
		if (!(lcr & UART_LCR_DLAB))
			novatek8250_apply_fcr(port);
		goto out_unlock;
	}

	if (reg == UART_IER && !(lcr & UART_LCR_DLAB)) {
		if (READ_ONCE(data->rx_running))
			value &= ~(UART_IER_RDI | UART_IER_RLSI);
		if (READ_ONCE(data->faulted))
			value = UART_IER_DISABLE;
	}
	writel(value, port->membase + (reg << port->regshift));
	if (reg == UART_LCR && !(value & UART_LCR_DLAB) && data->fcr_pending &&
	    !READ_ONCE(data->faulted))
		novatek8250_apply_fcr(port);

out_unlock:
	spin_unlock_irqrestore(&data->io_lock, flags);
}

static void novatek8250_reset_rx_fifo(struct uart_8250_port *up)
{
	struct novatek8250_data *data = up->port.private_data;
	struct uart_port *port = &up->port;
	ulong flags;

	lockdep_assert_held_once(&port->lock);
	novatek8250_stop_rx_dma(up, false);
	novatek8250_stop_tx_dma(up, true);
	if (data->faulted)
		return;

	spin_lock_irqsave(&data->io_lock, flags);
	writel(data->fcr | UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR,
	       port->membase + (UART_FCR << port->regshift));
	writel(data->fcr, port->membase + (UART_FCR << port->regshift));
	spin_unlock_irqrestore(&data->io_lock, flags);

	if (!data->tx_stopped)
		serial8250_set_THRI(up);
}

static void novatek8250_stop_tx(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;

	data->tx_stopped = true;
	data->base_ops->stop_tx(port);
}

static void novatek8250_stop_rx(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;

	data->rx_stopped = true;
	data->base_ops->stop_rx(port);
	novatek8250_stop_rx_dma(up_to_u8250p(port), false);
}

static void novatek8250_flush_buffer(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;

	data->tx_stopped = true;
	novatek8250_stop_tx_dma(up_to_u8250p(port), false);
	data->base_ops->stop_tx(port);
}

static unsigned int novatek8250_tx_empty(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;
	uint empty;
	u16 lsr;

	uart_port_lock_irqsave(port, &flags);
	if (data->rx_running)
		lsr = serial_in(up, UART_LSR);
	else
		lsr = serial_lsr_in(up);
	empty = !data->tx_running && !data->faulted && uart_lsr_tx_empty(lsr);
	uart_port_unlock_irqrestore(port, flags);
	return empty ? TIOCSER_TEMT : 0;
}

static void novatek8250_throttle(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;

	uart_port_lock_irqsave(port, &flags);
	data->throttled = true;
	up->ier &= ~(UART_IER_RDI | UART_IER_RLSI);
	serial_out(up, UART_IER, up->ier);
	novatek8250_stop_rx_dma(up, false);
	uart_port_unlock_irqrestore(port, flags);
}

static void novatek8250_unthrottle(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;

	uart_port_lock_irqsave(port, &flags);
	data->throttled = false;
	if (!data->rx_stopped) {
		up->ier |= UART_IER_RDI | UART_IER_RLSI;
		if (up->dma)
			novatek8250_rx_dma(up);
		serial_out(up, UART_IER, up->ier);
	}
	uart_port_unlock_irqrestore(port, flags);
}

static int novatek8250_handle_irq(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;
	struct irq_data *irqd;
	u32 pending;
	uint iir;
	u16 lsr;
	int handled = 0;

	uart_port_lock_irqsave(port, &flags);
	if (data->faulted)
		goto out_unlock;

	if (!up->dma) {
		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			serial8250_handle_irq_locked(port, iir);
			handled = 1;
		}
		goto out_unlock;
	}

	pending = readl(port->membase + UART_DMA_ISR_REG) & data->dma_ier;
	iir = serial_in(up, UART_IIR);
	if (!pending && iir & UART_IIR_NO_INT)
		goto out_unlock;
	handled = 1;
	if (pending & UART_DMA_ISR_RX_IRQ)
		novatek8250_stop_rx_dma(up, true);
	if (pending & UART_DMA_ISR_TX_DONE)
		novatek8250_stop_tx_dma(up, true);
	if (data->faulted)
		goto out_unlock;

	if (data->rx_running)
		lsr = serial_in(up, UART_LSR);
	else
		lsr = serial_lsr_in(up);
	if (!data->rx_running && !(lsr & UART_LSR_DR) && lsr & UART_LSR_FIFOE) {
		novatek8250_reset_rx_fifo(up);
		if (data->faulted)
			goto out_unlock;
	}

	if (!data->rx_running && !data->throttled && !data->rx_stopped) {
		if (lsr & (UART_LSR_DR | UART_LSR_BI)) {
			irqd = irq_get_irq_data(port->irq);
			if (irqd && irqd_is_wakeup_set(irqd))
				pm_wakeup_event(port->state->port.tty->dev, 0);
		}
		if (novatek8250_rx_dma_allowed(up))
			novatek8250_rx_dma(up);
		else if (lsr & (UART_LSR_DR | UART_LSR_BI))
			serial8250_rx_chars(up, lsr);
	}

	serial8250_modem_status(up);

	if (!data->tx_running &&
	    (pending & UART_DMA_ISR_TX_DONE ||
	     (lsr & UART_LSR_THRE && up->ier & UART_IER_THRI))) {
		if (novatek8250_tx_dma(up)) {
			serial8250_set_THRI(up);
			if (lsr & UART_LSR_THRE)
				serial8250_tx_chars(up);
		}
	}

out_unlock:
	uart_unlock_and_check_sysrq_irqrestore(port, flags);
	return handled;
}

static void novatek8250_dma_timer(struct timer_list *timer)
{
	struct novatek8250_data *data = timer_container_of(data, timer, timer);
	struct uart_8250_port *up = data->up;
	struct uart_port *port = &up->port;
	ulong flags;
	u32 control;
	bool tx_completed = false;

	uart_port_lock_irqsave(port, &flags);
	if (!data->opened || data->faulted || data->suspending)
		goto out_unlock;
	if (data->configuring)
		goto out_rearm_timer;

	if (data->rx_running) {
		control = readl(port->membase + UART_RX_CURRENT_REG);
		if (READ_ONCE(port->flags) & UPF_SAK ||
		    control != lower_32_bits(data->rx[data->rx_index].addr) ||
		    serial_in(up, UART_LSR) & UART_LSR_DR)
			novatek8250_stop_rx_dma(up, true);
	}

	if (data->tx_running) {
		control = readl(port->membase + UART_TX_CTRL_REG);
		if (!(control & UART_TX_CTRL_EN)) {
			novatek8250_stop_tx_dma(up, true);
			tx_completed = true;
		}
	}
	if (data->faulted)
		goto out_unlock;
	if (tx_completed && novatek8250_tx_dma(up))
		serial8250_set_THRI(up);

	novatek8250_rx_dma(up);

out_rearm_timer:
	mod_timer(&data->timer, jiffies + msecs_to_jiffies(UART_DMA_POLL_MS));
out_unlock:
	uart_unlock_and_check_sysrq_irqrestore(port, flags);
}

static int novatek8250_startup(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct tty_struct *tty = port->state->port.tty;
	ulong flags;
	int ret;

	if (data->reset_asserted) {
		ret = reset_control_deassert(data->rst);
		if (ret)
			return ret;
		data->reset_asserted = false;
		WRITE_ONCE(data->faulted, false);

		spin_lock_irqsave(&data->io_lock, flags);
		data->fcr = UART_FCR_DISABLE;
		data->pending_clear = 0;
		data->pending_disable = false;
		data->fcr_pending = false;
		data->dma_session = false;
		spin_unlock_irqrestore(&data->io_lock, flags);
	}
	if (data->faulted)
		return -EIO;

	if (!data->base_ops) {
		data->base_ops = port->ops;
		data->ops = *port->ops;
		data->ops.stop_tx = novatek8250_stop_tx;
		data->ops.stop_rx = novatek8250_stop_rx;
		data->ops.flush_buffer = novatek8250_flush_buffer;
		data->ops.tx_empty = novatek8250_tx_empty;
	}

	up->dma = NULL;
	ret = serial8250_do_startup(port);
	if (ret)
		return ret;

	uart_port_lock_irqsave(port, &flags);
	data->up = up;
	data->opened = true;
	data->configuring = false;
	data->suspending = false;
	data->throttled = false;
	data->rx_stopped = false;
	data->rx_allowed = false;
	data->rx_index = 0;
	data->tx_stopped = false;

	if (data->dma_enabled && !uart_console(port)) {
		data->throttled = tty && tty_throttled(tty);
		if (data->throttled)
			up->ier &= ~(UART_IER_RDI | UART_IER_RLSI);

		spin_lock(&data->io_lock);
		data->dma_session = true;
		spin_unlock(&data->io_lock);

		port->ops = &data->ops;
		port->status |= UPSTAT_SYNC_FIFO;
		up->dma = &data->dma;

		mod_timer(&data->timer, jiffies + msecs_to_jiffies(UART_DMA_POLL_MS));
	}
	uart_port_unlock_irqrestore(port, flags);
	return 0;
}

static void novatek8250_shutdown(struct uart_port *port)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;
	int ret;

	uart_port_lock_irqsave(port, &flags);
	data->opened = false;
	data->configuring = true;

	novatek8250_stop_rx_dma(up, false);
	novatek8250_stop_tx_dma(up, false);
	if (data->dma_enabled) {
		data->dma_ier = UART_DMA_IER_DISABLE_ALL;
		writel(data->dma_ier, port->membase + UART_DMA_IER_REG);
	}

	up->dma = NULL;
	port->status &= ~UPSTAT_SYNC_FIFO;
	if (!data->rx_running && !data->tx_running) {
		spin_lock(&data->io_lock);
		data->dma_session = false;
		spin_unlock(&data->io_lock);
	}
	uart_port_unlock_irqrestore(port, flags);

	timer_delete_sync(&data->timer);

	serial8250_do_shutdown(port);

	if (data->faulted) {
		ret = reset_control_assert(data->rst);
		if (!ret) {
			data->reset_asserted = true;
			WRITE_ONCE(data->rx_running, false);
			data->tx_running = false;
		} else {
			dev_err(port->dev, "failed to reset stalled UART DMA\n");
		}
	}

	uart_port_lock_irqsave(port, &flags);
	port->ops = data->base_ops;
	uart_port_unlock_irqrestore(port, flags);
}

static void novatek8250_free_dma(struct device *dev, struct novatek8250_data *data)
{
	uint i;

	if (data->rx_running || data->tx_running)
		return;

	for (i = 0; i < ARRAY_SIZE(data->rx); i++) {
		if (!data->rx[i].cpu)
			continue;
		dma_free_coherent(dev, UART_DMA_SIZE, data->rx[i].cpu, data->rx[i].addr);
		data->rx[i].cpu = NULL;
	}

	if (data->tx.cpu) {
		dma_free_coherent(dev, UART_DMA_SIZE, data->tx.cpu, data->tx.addr);
		data->tx.cpu = NULL;
	}
}

static int novatek8250_init_dma(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	struct novatek8250_data *data = port->private_data;
	struct device *dev = port->dev;
	uint i;
	int ret;

	if (port->mapsize < UART_DMA_ISR_REG + sizeof(u32))
		return 0;

	writel(UART_DMA_IER_DISABLE_ALL, port->membase + UART_DMA_IER_REG);
	if (!novatek8250_dma_stop(up, UART_TX_CTRL_REG) ||
	    !novatek8250_dma_stop(up, UART_RX_CTRL_REG))
		return -ETIMEDOUT;
	writel(UART_DMA_ISR_CLEAR_ALL, port->membase + UART_DMA_ISR_REG);

	if (device_property_read_bool(dev, "novatek,no-dma"))
		return 0;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto out_free_dma;

	for (i = 0; i < ARRAY_SIZE(data->rx); i++) {
		data->rx[i].cpu = dma_alloc_coherent(dev, UART_DMA_SIZE,
						     &data->rx[i].addr, GFP_KERNEL);
		if (!data->rx[i].cpu)
			goto out_free_dma;
	}

	data->tx.cpu = dma_alloc_coherent(dev, UART_DMA_SIZE, &data->tx.addr, GFP_KERNEL);
	if (!data->tx.cpu)
		goto out_free_dma;

	data->dma.tx_dma = novatek8250_start_tx_dma;
	data->dma.rx_dma = novatek8250_rx_dma;
	data->dma_enabled = true;
	return 0;

out_free_dma:
	novatek8250_free_dma(dev, data);
	dev_warn(dev, "UART DMA unavailable, using PIO\n");
	return 0;
}

static void novatek8250_set_divisor(struct uart_port *port, unsigned int baud,
				    unsigned int quot, unsigned int quot_frac)
{
	struct novatek8250_data *data = port->private_data;
	ulong flags;
	bool locked = !oops_in_progress || !uart_console(port);

	serial8250_do_set_divisor(port, baud, quot);

	/* DLAB aliases FCR with the prescaler; bypass the FCR accessor. */
	if (locked)
		spin_lock_irqsave(&data->io_lock, flags);
	writel(UART_PSR_DEFAULT, port->membase + (UART_PSR_OFFSET << port->regshift));
	if (locked)
		spin_unlock_irqrestore(&data->io_lock, flags);

	data->baud = DIV_ROUND_CLOSEST(port->uartclk, 16 * quot);
}

static void novatek8250_set_hw_flow(struct uart_port *port, bool enable)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	if (enable) {
		up->mcr |= UART_MCR_AFC;
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
	} else {
		up->mcr &= ~UART_MCR_AFC;
		port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	}
}

static u32 novatek8250_delay_to_bits(u32 delay_ms, uint baud)
{
	u64 bits;

	bits = DIV_ROUND_UP_ULL((u64)delay_ms * baud, MSEC_PER_SEC);
	return min_t(u64, bits, FIELD_MAX(UART_R485R_SETUP));
}

static void novatek8250_write_rs485(struct uart_port *port, struct serial_rs485 *rs485, uint baud)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	u32 setup = 0;
	u32 hold = 0;
	u32 value;

	value = 0;
	if (rs485->flags & SER_RS485_ENABLED) {
		if (rs485->delay_rts_before_send)
			setup = novatek8250_delay_to_bits(rs485->delay_rts_before_send, baud);
		if (rs485->delay_rts_after_send)
			hold = novatek8250_delay_to_bits(rs485->delay_rts_after_send, baud);
		value = UART_R485R_EN |
			FIELD_PREP(UART_R485R_SETUP, setup) |
			FIELD_PREP(UART_R485R_HOLD, hold);

		if (setup)
			rs485->delay_rts_before_send =
				DIV_ROUND_CLOSEST_ULL((u64)setup * MSEC_PER_SEC, baud);
		if (hold)
			rs485->delay_rts_after_send =
				DIV_ROUND_CLOSEST_ULL((u64)hold * MSEC_PER_SEC, baud);
	}

	serial_out(up, UART_R485R_OFFSET, value);
}

static int novatek8250_rs485_config(struct uart_port *port, struct ktermios *termios,
				    struct serial_rs485 *rs485)
{
	struct novatek8250_data *data = port->private_data;
	uint baud = data->baud;
	bool enable = rs485->flags & SER_RS485_ENABLED;

	lockdep_assert_held_once(&port->lock);

	if (enable && (rs485->delay_rts_before_send || rs485->delay_rts_after_send) && !baud)
		return -EINVAL;
	if (enable && termios && termios->c_cflag & CRTSCTS)
		return -EBUSY;

	if (enable) {
		novatek8250_stop_tx_dma(up_to_u8250p(port), true);
		novatek8250_stop_rx_dma(up_to_u8250p(port), true);
		if (data->faulted)
			return -EIO;
	}

	novatek8250_set_hw_flow(port, false);
	serial8250_do_set_mctrl(port, port->mctrl);

	novatek8250_write_rs485(port, rs485, baud);

	if (up_to_u8250p(port)->dma && !data->tx_stopped)
		serial8250_set_THRI(up_to_u8250p(port));

	return 0;
}

static const struct serial_rs485 novatek8250_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND,
	.delay_rts_before_send = 1,
	.delay_rts_after_send = 1,
};

static void novatek8250_set_termios(struct uart_port *port, struct ktermios *termios,
				    const struct ktermios *old)
{
	struct novatek8250_data *data = port->private_data;
	struct uart_8250_port *up = up_to_u8250p(port);
	ulong flags;
	bool flow;

	flow = termios->c_cflag & CRTSCTS && port->flags & UPF_HARD_FLOW &&
	       !(port->rs485.flags & SER_RS485_ENABLED);
	if (!flow)
		termios->c_cflag &= ~CRTSCTS;

	uart_port_lock_irqsave(port, &flags);
	data->configuring = true;
	novatek8250_stop_tx_dma(up, true);
	novatek8250_stop_rx_dma(up, true);
	if (data->faulted)
		goto out_unlock;

	novatek8250_set_hw_flow(port, flow);
	uart_port_unlock_irqrestore(port, flags);

	serial8250_do_set_termios(port, termios, old);

	uart_port_lock_irqsave(port, &flags);
	if (tty_termios_baud_rate(termios) && port->rs485.flags & SER_RS485_ENABLED)
		novatek8250_write_rs485(port, &port->rs485, data->baud);

	data->rx_allowed = termios->c_cflag & CREAD &&
		!(termios->c_iflag & (INPCK | IGNPAR | PARMRK | IGNBRK | BRKINT));
	data->configuring = false;

	if (up->dma) {
		novatek8250_rx_dma(up);
		if (novatek8250_tx_dma(up))
			serial8250_set_THRI(up);
	}

out_unlock:
	uart_port_unlock_irqrestore(port, flags);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init novatek8250_early_console_setup(struct earlycon_device *device,
						  const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32;
	device->port.regshift = UART_REGSHIFT;

	device->baud = 0;
	return early_serial8250_setup(device, options);
}

OF_EARLYCON_DECLARE(nvt_na51089, "novatek,na51089-uart", novatek8250_early_console_setup);
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
	spin_lock_init(&data->io_lock);
	timer_setup(&data->timer, novatek8250_dma_timer, 0);

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
		return dev_err_probe(dev, ret, "failed to read UART properties\n");

	data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk), "failed to get UART clock\n");

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable UART clock\n");

	rate = clk_get_rate(data->clk);
	if (!rate) {
		ret = dev_err_probe(dev, -EINVAL, "invalid UART clock rate %lu\n", rate);
		goto out_disable_clock;
	}
	up.port.uartclk = rate;

	data->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(data->rst)) {
		ret = dev_err_probe(dev, PTR_ERR(data->rst), "failed to get UART reset\n");
		goto out_disable_clock;
	}

	ret = reset_control_deassert(data->rst);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to deassert UART reset\n");
		goto out_disable_clock;
	}

	has_rtscts = device_property_read_bool(dev, "uart-has-rtscts");
	rs485_at_boot = device_property_read_bool(dev, "linux,rs485-enabled-at-boot-time");
	if (has_rtscts || !rs485_at_boot)
		writel(UART_R485R_DISABLED, up.port.membase + UART_R485R_REG);

	device_property_read_u32(dev, "current-speed", &data->baud);

	up.port.type = PORT_NOVATEK;
	up.port.iotype = UPIO_MEM32;
	up.port.regshift = UART_REGSHIFT;
	up.port.flags |= UPF_FIXED_PORT | UPF_FIXED_TYPE;
	up.bugs |= UART_BUG_NOMSR;

	up.port.set_divisor = novatek8250_set_divisor;
	up.port.set_termios = novatek8250_set_termios;
	up.port.serial_out = novatek8250_serial_out;
	up.port.startup = novatek8250_startup;
	up.port.shutdown = novatek8250_shutdown;
	up.port.handle_irq = novatek8250_handle_irq;
	up.port.throttle = novatek8250_throttle;
	up.port.unthrottle = novatek8250_unthrottle;

	if (has_rtscts) {
		up.port.flags |= UPF_HARD_FLOW;
	} else {
		up.port.rs485_config = novatek8250_rs485_config;
		up.port.rs485_supported = novatek8250_rs485_supported;
	}

	ret = novatek8250_init_dma(&up);
	if (ret)
		goto out_assert_reset;

	ret = serial8250_register_8250_port(&up);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register 8250 port\n");
		goto out_assert_reset;
	}
	data->line = ret;
	platform_set_drvdata(pdev, data);

	return 0;

out_assert_reset:
	reset_control_assert(data->rst);
	novatek8250_free_dma(dev, data);
out_disable_clock:
	clk_disable_unprepare(data->clk);

	return ret;
}

static void novatek8250_remove(struct platform_device *pdev)
{
	struct novatek8250_data *data = platform_get_drvdata(pdev);
	int ret;

	serial8250_unregister_port(data->line);

	ret = reset_control_assert(data->rst);
	if (!ret) {
		WRITE_ONCE(data->rx_running, false);
		data->tx_running = false;
	}
	if (data->rx_running || data->tx_running) {
		dev_err(&pdev->dev, "stalled DMA: retaining buffers and clock\n");
		return;
	}

	novatek8250_free_dma(&pdev->dev, data);
	clk_disable_unprepare(data->clk);
}

#ifdef CONFIG_PM_SLEEP
static int novatek8250_suspend(struct device *dev)
{
	struct novatek8250_data *data = dev_get_drvdata(dev);
	struct uart_8250_port *up = serial8250_get_port(data->line);
	ulong flags;

	uart_port_lock_irqsave(&up->port, &flags);
	data->suspending = true;
	novatek8250_stop_rx_dma(up, true);
	novatek8250_stop_tx_dma(up, true);
	uart_port_unlock_irqrestore(&up->port, flags);

	timer_delete_sync(&data->timer);
	if (data->faulted) {
		uart_port_lock_irqsave(&up->port, &flags);
		data->suspending = false;
		uart_port_unlock_irqrestore(&up->port, flags);
		return -EIO;
	}

	serial8250_suspend_port(data->line);
	if (up->port.suspended)
		clk_disable_unprepare(data->clk);

	return 0;
}

static int novatek8250_resume(struct device *dev)
{
	struct novatek8250_data *data = dev_get_drvdata(dev);
	struct uart_8250_port *up = serial8250_get_port(data->line);
	ulong flags;
	int ret;

	if (up->port.suspended) {
		ret = clk_prepare_enable(data->clk);
		if (ret)
			return ret;
	}

	serial8250_resume_port(data->line);

	uart_port_lock_irqsave(&up->port, &flags);
	data->suspending = false;
	if (data->opened && up->dma && !data->faulted) {
		novatek8250_rx_dma(up);
		if (novatek8250_tx_dma(up))
			serial8250_set_THRI(up);

		mod_timer(&data->timer, jiffies + msecs_to_jiffies(UART_DMA_POLL_MS));
	}
	uart_port_unlock_irqrestore(&up->port, flags);

	return 0;
}
#endif

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

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek 8250-compatible UART driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SERIAL_8250");
