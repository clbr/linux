// SPDX-License-Identifier: GPL-2.0
/*
 *  Nintendo 64 init.
 *
 *  Copyright (C) 2020	Lauri Kasanen
 */
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include <asm/bootinfo.h>
#include <asm/time.h>

#define IO_MEM_RESOURCE_START	0UL
#define IO_MEM_RESOURCE_END	0x1fffffffUL

static void __init iomem_resource_init(void)
{
	iomem_resource.start = IO_MEM_RESOURCE_START;
	iomem_resource.end = IO_MEM_RESOURCE_END;
}

const char *get_system_type(void)
{
	return "Nintendo 64";
}

void __init prom_init(void)
{
	int argc, i;
	const char **argv;

	argc = fw_arg0;
	argv = (const char **)fw_arg1;

	for (i = 1; i < argc; i++) {
		strlcat(arcs_cmdline, argv[i], COMMAND_LINE_SIZE);
		if (i < (argc - 1))
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}
}

void __init prom_free_prom_memory(void)
{
}

static int __init n64_platform_init(void)
{
	platform_device_register_simple("n64rdp", -1, NULL, 0);
	platform_device_register_simple("n64audio", -1, NULL, 0);

	return 0;
}

arch_initcall(n64_platform_init);

void __init plat_mem_setup(void)
{
	iomem_resource_init();
	memblock_add(0x0, 8 * 1024 * 1024); // Bootloader blocks the 4mb config
}

void __init plat_time_init(void)
{
	// 93.75 MHz cpu, count register runs at half rate
	mips_hpt_frequency = 93750000 / 2;
}
