// SPDX-License-Identifier: GPL-2.0
/*
 * Support for the N64 cart.
 *
 * Copyright (c) 2020 Lauri Kasanen
 */

#include <linux/bitops.h>
#include <linux/blk-mq.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/addrspace.h>
#include <asm/io.h>

MODULE_AUTHOR("Lauri Kasanen <cand@gmx.com>");
MODULE_DESCRIPTION("Driver for the N64 cart");
MODULE_LICENSE("GPL");

#define BUFSIZE (64 * 1024)

static unsigned int start, size;
static int major;
static struct request_queue *queue;
static struct blk_mq_tag_set tag_set;
static struct gendisk *disk;

static void *buf;
static dma_addr_t dma_addr;

static DEFINE_SPINLOCK(n64cart_lock);

#define REG_BASE ((u32 *) CKSEG1ADDR(0xA4600000))

#define PI_DRAM_REG 0
#define PI_CART_REG 1
#define PI_READ_REG 2
#define PI_WRITE_REG 3
#define PI_STATUS_REG 4

#define PI_STATUS_DMA_BUSY (1 << 0)
#define PI_STATUS_IO_BUSY (1 << 1)

static void n64cart_write_reg(const u8 reg, const u32 value)
{
	__raw_writel(value, REG_BASE + reg);
}

static u32 n64cart_read_reg(const u8 reg)
{
	return __raw_readl(REG_BASE + reg);
}

static void n64cart_wait_dma(void)
{
	while (n64cart_read_reg(PI_STATUS_REG) &
		(PI_STATUS_DMA_BUSY | PI_STATUS_IO_BUSY))
		;
}

static blk_status_t get_seg(struct request *req)
{
	u32 bstart = blk_rq_pos(req) * 512;
	u32 len = blk_rq_cur_bytes(req);
	void *dst = bio_data(req->bio);

	if (bstart + len > size || rq_data_dir(req) == WRITE)
		return BLK_STS_IOERR;

	bstart += start;

	while (len) {
		const u32 curlen = len < BUFSIZE ? len : BUFSIZE;

		dma_cache_inv((unsigned long) buf, curlen);

		n64cart_wait_dma();

		barrier();
		n64cart_write_reg(PI_DRAM_REG, dma_addr);
		barrier();
		n64cart_write_reg(PI_CART_REG, (bstart | 0x10000000) & 0x1FFFFFFF);
		barrier();
		n64cart_write_reg(PI_WRITE_REG, curlen - 1);
		barrier();

		n64cart_wait_dma();

		memcpy(dst, buf, curlen);

		len -= curlen;
		dst += curlen;
		bstart += curlen;
	}

	return BLK_STS_OK;
}

static blk_status_t n64cart_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
	unsigned long flags;
	struct request *req = bd->rq;
	blk_status_t err;

	blk_mq_start_request(req);

	spin_lock_irqsave(&n64cart_lock, flags);

	do {
		err = get_seg(req);
	} while (blk_update_request(req, err, blk_rq_cur_bytes(req)));

	spin_unlock_irqrestore(&n64cart_lock, flags);
	blk_mq_end_request(req, BLK_STS_OK);
	return BLK_STS_OK;
}

static const struct blk_mq_ops n64cart_mq_ops = {
	.queue_rq = n64cart_queue_rq,
};

static const struct block_device_operations n64cart_fops = {
	.owner		= THIS_MODULE,
};

static int __init n64cart_init(void)
{
	int err;

	if (!start || !size) {
		pr_err("n64cart: start and size not specified\n");
		return -ENODEV;
	}

	if (size & 4095) {
		pr_err("n64cart: size must be a multiple of 4K\n");
		return -ENODEV;
	}

	major = register_blkdev(0, "n64cart");
	if (major <= 0) {
		pr_err("n64cart: unable to get major number\n");
		return -EBUSY;
	}

	queue = blk_mq_init_sq_queue(&tag_set, &n64cart_mq_ops, 16,
				     BLK_MQ_F_SHOULD_MERGE);
	if (IS_ERR(queue)) {
		err = PTR_ERR(queue);
		goto fail_reg;
	}

	buf = kmalloc(BUFSIZE, GFP_DMA | GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
		goto fail_queue;
	}
	dma_addr = virt_to_phys(buf);

	disk = alloc_disk(1);
	if (!disk) {
		err = -ENOMEM;
		goto fail_dma;
	}

	disk->major = major;
	disk->first_minor = 0;
	disk->queue = queue;
	disk->flags = GENHD_FL_NO_PART_SCAN;
	disk->fops = &n64cart_fops;
	strcpy(disk->disk_name, "n64cart");

	set_capacity(disk, size / 512);

	blk_queue_flag_set(QUEUE_FLAG_NONROT, queue);
	blk_queue_physical_block_size(queue, 4096);
	blk_queue_logical_block_size(queue, 4096);

	add_disk(disk);

	pr_info("n64cart: %u kb disk\n", size / 1024);

	return 0;
fail_dma:
	kfree(buf);
fail_queue:
	blk_cleanup_queue(queue);
fail_reg:
	unregister_blkdev(major, "n64cart");
	return err;
}

module_param(start, uint, 0);
MODULE_PARM_DESC(start, "Start address of the cart block data");

module_param(size, uint, 0);
MODULE_PARM_DESC(size, "Size of the cart block data, in bytes");

module_init(n64cart_init);
