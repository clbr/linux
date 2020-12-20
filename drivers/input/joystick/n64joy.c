// SPDX-License-Identifier: GPL-2.0
/*
 * Support for the four N64 controllers.
 *
 * Copyright (c) 2020 Lauri Kasanen
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/limits.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/timer.h>

#include <asm/addrspace.h>
#include <asm/io.h>

MODULE_AUTHOR("Lauri Kasanen <cand@gmx.com>");
MODULE_DESCRIPTION("Driver for N64 controllers");
MODULE_LICENSE("GPL");

#define PIF_RAM 0x1fc007c0
#define REG_BASE ((u32 *) CKSEG1ADDR(0xa4800000))

#define SI_DRAM_REG 0
#define SI_READ_REG 1
#define SI_WRITE_REG 4
#define SI_STATUS_REG 6

#define SI_STATUS_DMA_BUSY  (1 << 0)
#define SI_STATUS_IO_BUSY   (1 << 1)

#define N64_CONTROLLER_ID 0x0500

static struct input_dev *n64joy_dev[4];
static const char *n64joy_phys[4] = {
	"n64joy/port0",
	"n64joy/port1",
	"n64joy/port2",
	"n64joy/port3",
};

static u8 n64joy_opened;
static DEFINE_MUTEX(n64joy_mutex);
static struct timer_list timer;

static u64 si_buf[8] ____cacheline_aligned;

struct joydata {
	unsigned: 16; // unused
	unsigned err: 2;
	unsigned: 14; // unused

	union {
		u32 data;

		struct {
			unsigned a: 1;
			unsigned b: 1;
			unsigned z: 1;
			unsigned start: 1;
			unsigned up: 1;
			unsigned down: 1;
			unsigned left: 1;
			unsigned right: 1;
			unsigned: 2; // unused
			unsigned l: 1;
			unsigned r: 1;
			unsigned c_up: 1;
			unsigned c_down: 1;
			unsigned c_left: 1;
			unsigned c_right: 1;
			signed x: 8;
			signed y: 8;
		};
	};
};

static void n64joy_write_reg(const u8 reg, const u32 value)
{
	__raw_writel(value, REG_BASE + reg);
}

static u32 n64joy_read_reg(const u8 reg)
{
	return __raw_readl(REG_BASE + reg);
}

static void n64joy_wait_si_dma(void)
{
	while (n64joy_read_reg(SI_STATUS_REG) & (SI_STATUS_DMA_BUSY | SI_STATUS_IO_BUSY))
		;
}

static void n64joy_exec_pif(const u64 in[8])
{
	unsigned long flags;

	dma_cache_wback_inv((unsigned long) in, 8 * 8);
	dma_cache_inv((unsigned long) si_buf, 8 * 8);

	local_irq_save(flags);

	n64joy_wait_si_dma();

	barrier();
	n64joy_write_reg(SI_DRAM_REG, virt_to_phys(in));
	barrier();
	n64joy_write_reg(SI_WRITE_REG, PIF_RAM);
	barrier();

	n64joy_wait_si_dma();

	barrier();
	n64joy_write_reg(SI_DRAM_REG, virt_to_phys(si_buf));
	barrier();
	n64joy_write_reg(SI_READ_REG, PIF_RAM);
	barrier();

	n64joy_wait_si_dma();

	local_irq_restore(flags);
}

static const u64 polldata[] ____cacheline_aligned = {
	0xff010401ffffffff,
	0xff010401ffffffff,
	0xff010401ffffffff,
	0xff010401ffffffff,
	0xfe00000000000000,
	0,
	0,
	1
};

static void n64joy_poll(struct timer_list *t)
{
	const struct joydata *data;
	u32 i;

	n64joy_exec_pif(polldata);

	data = (struct joydata *) si_buf;

	for (i = 0; i < 4; i++) {
		if (!n64joy_dev[i])
			continue;

		// d-pad
		input_report_key(n64joy_dev[i], BTN_DPAD_UP, data[i].up);
		input_report_key(n64joy_dev[i], BTN_DPAD_DOWN, data[i].down);
		input_report_key(n64joy_dev[i], BTN_DPAD_LEFT, data[i].left);
		input_report_key(n64joy_dev[i], BTN_DPAD_RIGHT, data[i].right);

		// c buttons
		input_report_key(n64joy_dev[i], BTN_FORWARD, data[i].c_up);
		input_report_key(n64joy_dev[i], BTN_BACK, data[i].c_down);
		input_report_key(n64joy_dev[i], BTN_LEFT, data[i].c_left);
		input_report_key(n64joy_dev[i], BTN_RIGHT, data[i].c_right);

		// matching buttons
		input_report_key(n64joy_dev[i], BTN_START, data[i].start);
		input_report_key(n64joy_dev[i], BTN_Z, data[i].z);

		// remaining ones: a, b, l, r
		input_report_key(n64joy_dev[i], BTN_0, data[i].a);
		input_report_key(n64joy_dev[i], BTN_1, data[i].b);
		input_report_key(n64joy_dev[i], BTN_2, data[i].l);
		input_report_key(n64joy_dev[i], BTN_3, data[i].r);

		input_report_abs(n64joy_dev[i], ABS_X, data[i].x);
		input_report_abs(n64joy_dev[i], ABS_Y, data[i].y);

		input_sync(n64joy_dev[i]);
	}

	mod_timer(&timer, jiffies + msecs_to_jiffies(16));
}

static int n64joy_open(struct input_dev *dev)
{
	int err;

	err = mutex_lock_interruptible(&n64joy_mutex);
	if (err)
		return err;

	if (!n64joy_opened) {
		// Could use the vblank irq, but it's not important if the poll
		// point slightly changes.
		timer_setup(&timer, n64joy_poll, 0);
		mod_timer(&timer, jiffies + msecs_to_jiffies(16));
	}

	n64joy_opened++;

	mutex_unlock(&n64joy_mutex);
	return err;
}

static void n64joy_close(struct input_dev *dev)
{
	mutex_lock(&n64joy_mutex);
	if (!--n64joy_opened)
		del_timer_sync(&timer);
	mutex_unlock(&n64joy_mutex);
}

static const u64 __initconst scandata[] ____cacheline_aligned = {
	0xff010300ffffffff,
	0xff010300ffffffff,
	0xff010300ffffffff,
	0xff010300ffffffff,
	0xfe00000000000000,
	0,
	0,
	1
};

static int __init n64joy_init(void)
{
	const struct joydata *data;
	int err = 0;
	u32 i, j, found = 0;

	// The controllers are not hotpluggable, so we can scan in init
	n64joy_exec_pif(scandata);

	data = (struct joydata *) si_buf;

	memset(n64joy_dev, 0, 4 * sizeof(void *));

	for (i = 0; i < 4; i++) {
		if (!data[i].err && data[i].data >> 16 == N64_CONTROLLER_ID) {
			found++;

			n64joy_dev[i] = input_allocate_device();
			if (!n64joy_dev[i]) {
				err = -ENOMEM;
				goto fail;
			}

			n64joy_dev[i]->name = "N64 controller";
			n64joy_dev[i]->phys = n64joy_phys[i];
			n64joy_dev[i]->id.bustype = BUS_HOST;
			n64joy_dev[i]->id.vendor = 0;
			n64joy_dev[i]->id.product = data[i].data >> 16;
			n64joy_dev[i]->id.version = 0;

			n64joy_dev[i]->open = n64joy_open;
			n64joy_dev[i]->close = n64joy_close;

			n64joy_dev[i]->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
			n64joy_dev[i]->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y);

			// d-pad
			n64joy_dev[i]->keybit[BIT_WORD(BTN_DPAD_UP)] = BIT_MASK(BTN_DPAD_UP) |
				BIT_MASK(BTN_DPAD_DOWN) | BIT_MASK(BTN_DPAD_LEFT) |
				BIT_MASK(BTN_DPAD_RIGHT);
			// c buttons
			n64joy_dev[i]->keybit[BIT_WORD(BTN_LEFT)] |= BIT_MASK(BTN_LEFT) |
				BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_FORWARD) | BIT_MASK(BTN_BACK);
			// matching buttons
			n64joy_dev[i]->keybit[BIT_WORD(BTN_GAMEPAD)] |= BIT_MASK(BTN_START) |
				BIT_MASK(BTN_Z);
			// remaining ones: a, b, l, r
			n64joy_dev[i]->keybit[BIT_WORD(BTN_0)] |= BIT_MASK(BTN_0) |
				BIT_MASK(BTN_1) | BIT_MASK(BTN_2) | BIT_MASK(BTN_3);

			for (j = 0; j < 2; j++) {
				input_set_abs_params(n64joy_dev[i], ABS_X + j,
						     S8_MIN, S8_MAX, 0, 0);
			}

			err = input_register_device(n64joy_dev[i]);
			if (err) {
				input_free_device(n64joy_dev[i]);
				goto fail;
			}
		}
	}

	pr_info("n64joy: %u controller(s) connected\n", found);

	if (!found)
		return -ENODEV;

	return 0;
fail:
	for (i = 0; i < 4; i++) {
		if (!n64joy_dev[i])
			continue;
		input_unregister_device(n64joy_dev[i]);
	}
	return err;
}

module_init(n64joy_init);
