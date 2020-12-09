// SPDX-License-Identifier: GPL-2.0
/*
 *  N64 IRQ
 *
 *  Copyright (C) 2020 Lauri Kasanen
 */
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/irq_cpu.h>

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;

	if (pending & CAUSEF_IP7)
		do_IRQ(TIMER_IRQ);
	else if (pending & CAUSEF_IP4)
		do_IRQ(PRENMI_IRQ);
	else if (pending & CAUSEF_IP2)
		do_IRQ(RCP_IRQ);
	else if (pending & CAUSEF_IP0)
		do_IRQ(MIPS_SOFTINT0_IRQ);
	else if (pending & CAUSEF_IP1)
		do_IRQ(MIPS_SOFTINT1_IRQ);
	else
		spurious_interrupt();
}

void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
}
