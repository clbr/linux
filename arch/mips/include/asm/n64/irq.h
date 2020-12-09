/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interrupt numbers for N64
 *
 * Copyright (C) 2020 Lauri Kasanen
 */
#ifndef __N64_IRQ_H
#define __N64_IRQ_H

#define NR_IRQS 8

/*
 * CPU core Interrupt Numbers
 */
#define MIPS_CPU_IRQ_BASE	0
#define MIPS_CPU_IRQ(x)		(MIPS_CPU_IRQ_BASE + (x))
#define MIPS_SOFTINT0_IRQ	MIPS_CPU_IRQ(0)
#define MIPS_SOFTINT1_IRQ	MIPS_CPU_IRQ(1)
#define RCP_IRQ			MIPS_CPU_IRQ(2)
#define CART_IRQ		MIPS_CPU_IRQ(3)
#define PRENMI_IRQ		MIPS_CPU_IRQ(4)
#define RDBR_IRQ		MIPS_CPU_IRQ(5)
#define RDBW_IRQ		MIPS_CPU_IRQ(6)
#define TIMER_IRQ		MIPS_CPU_IRQ(7)

#endif /* __N64_IRQ_H */
