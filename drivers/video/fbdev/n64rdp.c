// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the N64's RDP
 *
 * Copyright (c) 2020 Lauri Kasanen
 *
 * Based on simplefb.c, which was:
 * Copyright (c) 2013, Stephen Warren
 *
 * Based on q40fb.c, which was:
 * Copyright (C) 2001 Richard Zidlicky <rz@linux-m68k.org>
 *
 * Also based on offb.c, which was:
 * Copyright (C) 1997 Geert Uytterhoeven
 * Copyright (C) 1996 Paul Mackerras
 */

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/parser.h>

#include <asm/addrspace.h>

static const struct fb_fix_screeninfo n64rdp_fix = {
	.id		= "default",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.accel		= FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo n64rdp_var = {
	.height		= -1,
	.width		= -1,
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

#define PSEUDO_PALETTE_SIZE 16

static int n64rdp_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static const struct fb_ops n64rdp_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= n64rdp_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

struct n64rdp_par {
	u32 palette[PSEUDO_PALETTE_SIZE];
	dma_addr_t physaddr;
};

#define REG_BASE ((u32 *) CKSEG1ADDR(0xA4400000))

static void n64rdp_write_reg(const u8 reg, const u32 value)
{
	__raw_writel(value, REG_BASE + reg);
}

#define W 320
#define H 240

static const u32 ntsc_320[] __initconst = {
	0x00013212, 0x00000000, 0x00000140, 0x00000200,
	0x00000000, 0x03e52239, 0x0000020d, 0x00000c15,
	0x0c150c15, 0x006c02ec, 0x002501ff, 0x000e0204,
	0x00000200, 0x00000400
};

static int __init n64rdp_probe(struct platform_device *pdev)
{
	int ret;
	u32 i;
	struct fb_info *info;
	struct n64rdp_par *par;
	dma_addr_t addr;

	info = framebuffer_alloc(sizeof(struct n64rdp_par), &pdev->dev);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);

	par = info->par;

	info->fix = n64rdp_fix;
	info->screen_base = dma_alloc_coherent(&pdev->dev, W * H * 2, &addr,
					       GFP_DMA | GFP_KERNEL);
	if (!info->screen_base)
		return -ENOMEM;

	info->fix.smem_start = par->physaddr = addr;
	info->fix.smem_len = W * H * 2;
	info->fix.line_length = W * 2;

	info->var = n64rdp_var;
	info->var.xres = W;
	info->var.yres = H;
	info->var.xres_virtual = W;
	info->var.yres_virtual = H;
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield) {11, 5};
	info->var.green = (struct fb_bitfield) {6, 5};
	info->var.blue = (struct fb_bitfield) {1, 5};
	info->var.transp = (struct fb_bitfield) {0, 1};

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		ret = -ENOMEM;
		goto error_fb_release;
	}
	info->apertures->ranges[0].base = info->fix.smem_start;
	info->apertures->ranges[0].size = info->fix.smem_len;

	info->fbops = &n64rdp_ops;
	info->flags = FBINFO_DEFAULT;
	info->pseudo_palette = par->palette;

	dev_info(&pdev->dev, "framebuffer at 0x%lx, 0x%x bytes, mapped to 0x%p\n",
			     info->fix.smem_start, info->fix.smem_len,
			     info->screen_base);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register n64rdp: %d\n", ret);
		goto error_fb_release;
	}

	for (i = 0; i < ARRAY_SIZE(ntsc_320); i++) {
		if (i == 1)
			n64rdp_write_reg(i, par->physaddr);
		else
			n64rdp_write_reg(i, ntsc_320[i]);
	}

	return 0;

error_fb_release:
	framebuffer_release(info);
	return ret;
}

static struct platform_driver n64rdp_driver = {
	.driver = {
		.name = "n64rdp",
	},
};

static int __init n64rdp_init(void)
{
	int ret;

	ret = platform_driver_probe(&n64rdp_driver, n64rdp_probe);

	return ret;
}

fs_initcall(n64rdp_init);

MODULE_AUTHOR("Lauri Kasanen <cand@gmx.com>");
MODULE_DESCRIPTION("Driver for the N64's display");
MODULE_LICENSE("GPL v2");
