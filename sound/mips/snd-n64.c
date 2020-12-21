// SPDX-License-Identifier: GPL-2.0
/*
 *   Sound driver for Nintendo 64.
 *
 *   Copyright 2020 Lauri Kasanen
 */

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/module.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <asm/addrspace.h>
#include <asm/n64/irq.h>

MODULE_AUTHOR("Lauri Kasanen <cand@gmx.com>");
MODULE_DESCRIPTION("N64 Audio");
MODULE_LICENSE("GPL");

#define AI_NTSC_DACRATE 48681812
#define AI_STATUS_BUSY  (1 << 30)
#define AI_STATUS_FULL  (1 << 31)

#define REG_BASE ((u32 *) CKSEG1ADDR(0xa4500000))

#define AI_ADDR_REG 0
#define AI_LEN_REG 1
#define AI_CONTROL_REG 2
#define AI_STATUS_REG 3
#define AI_RATE_REG 4
#define AI_BITCLOCK_REG 5

#define MI_REG_BASE ((u32 *) CKSEG1ADDR(0xa4300000))

#define MI_INTR_REG 2
#define MI_MASK_REG 3

#define MI_INTR_AI 0x04

#define MI_MASK_CLR_AI 0x0010
#define MI_MASK_SET_AI 0x0020


struct n64audio_t {
	struct snd_card *card;

	void *ring_base;
	dma_addr_t ring_base_dma;

	struct {
		struct snd_pcm_substream *substream;
		int pos, nextpos;
		u32 writesize;
		u32 bufsize;
		spinlock_t lock;
	} chan;
};

static void n64audio_write_reg(const u8 reg, const u32 value)
{
	__raw_writel(value, REG_BASE + reg);
}

static void n64mi_write_reg(const u8 reg, const u32 value)
{
	__raw_writel(value, MI_REG_BASE + reg);
}

static u32 n64mi_read_reg(const u8 reg)
{
	return __raw_readl(MI_REG_BASE + reg);
}

static void n64audio_push(struct n64audio_t *priv, uint8_t irq)
{
	struct snd_pcm_runtime *runtime = priv->chan.substream->runtime;
	unsigned long flags;
	u32 count;

	count = priv->chan.writesize;
	count &= ~7;

	spin_lock_irqsave(&priv->chan.lock, flags);

	memcpy(priv->ring_base, runtime->dma_area + priv->chan.nextpos, count);

	n64audio_write_reg(AI_ADDR_REG, priv->ring_base_dma);
	n64audio_write_reg(AI_LEN_REG, count);

	priv->chan.nextpos += count;
	priv->chan.nextpos %= priv->chan.bufsize;
	if (irq)
		priv->chan.pos = priv->chan.nextpos;

	spin_unlock_irqrestore(&priv->chan.lock, flags);
}

static irqreturn_t n64audio_isr(int irq, void *dev_id)
{
	struct n64audio_t *priv = dev_id;

	// Check it's ours
	const u32 intrs = n64mi_read_reg(MI_INTR_REG);
	if (!(intrs & MI_INTR_AI))
		return IRQ_NONE;

	n64audio_write_reg(AI_STATUS_REG, 1);

	n64audio_push(priv, 1);
	snd_pcm_period_elapsed(priv->chan.substream);

	return IRQ_HANDLED;
}

static const struct snd_pcm_hardware n64audio_pcm_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats =          SNDRV_PCM_FMTBIT_S16_BE,
	.rates =            SNDRV_PCM_RATE_8000_48000,
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     2,
	.channels_max =     2,
	.buffer_bytes_max = 32768,
	.period_bytes_min = 1024,
	.period_bytes_max = 32768,
	.periods_min =      1,
	.periods_max =      128,
};

static int n64audio_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = n64audio_pcm_hw;
	return 0;
}

static int n64audio_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct n64audio_t *priv = substream->pcm->private_data;
	unsigned long flags;
	u32 rate;

	rate = ((2 * AI_NTSC_DACRATE / runtime->rate) + 1) / 2 - 1;

	n64audio_write_reg(AI_RATE_REG, rate);

	rate /= 66;
	if (rate > 16)
		rate = 16;
	n64audio_write_reg(AI_BITCLOCK_REG, rate - 1);

	spin_lock_irqsave(&priv->chan.lock, flags);

	/* Setup the pseudo-dma transfer pointers.  */
	priv->chan.pos = 0;
	priv->chan.nextpos = 0;
	priv->chan.substream = substream;
	priv->chan.writesize = snd_pcm_lib_period_bytes(substream);
	priv->chan.bufsize = snd_pcm_lib_buffer_bytes(substream);

	spin_unlock_irqrestore(&priv->chan.lock, flags);
	return 0;
}

static int n64audio_pcm_trigger(struct snd_pcm_substream *substream,
				int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		n64audio_push(substream->pcm->private_data, 0);
		n64audio_write_reg(AI_CONTROL_REG, 1);
		n64mi_write_reg(MI_MASK_REG, MI_MASK_SET_AI);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		n64audio_write_reg(AI_CONTROL_REG, 0);
		n64mi_write_reg(MI_MASK_REG, MI_MASK_CLR_AI);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t n64audio_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct n64audio_t *priv = substream->pcm->private_data;

	return bytes_to_frames(substream->runtime,
			       priv->chan.pos);
}

static int n64audio_pcm_close(struct snd_pcm_substream *substream)
{
	return 0; // Nothing to do, but the kernel crashes if close() doesn't exist
}

static const struct snd_pcm_ops n64audio_pcm_ops = {
	.open =		n64audio_pcm_open,
	.prepare =	n64audio_pcm_prepare,
	.trigger =	n64audio_pcm_trigger,
	.pointer =	n64audio_pcm_pointer,
	.close =	n64audio_pcm_close,
};

static int __init n64audio_probe(struct platform_device *pdev)
{
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct n64audio_t *priv;
	int err;

	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1,
			   SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		err = -ENOMEM;
		goto fail_card;
	}

	priv->card = card;
	priv->ring_base = dma_alloc_coherent(card->dev, 32 * 1024,
					     &priv->ring_base_dma,
					     GFP_DMA | GFP_KERNEL);
	if (!priv->ring_base)
		goto fail_alloc;

	if (request_irq(RCP_IRQ, n64audio_isr,
				IRQF_SHARED, "N64 Audio", priv)) {
		err = -EBUSY;
		goto fail_alloc;
	}

	spin_lock_init(&priv->chan.lock);

	err = snd_pcm_new(card, "N64 Audio", 0, 1, 0, &pcm);
	if (err < 0)
		goto fail_alloc;

	pcm->private_data = priv;
	strcpy(pcm->name, "N64 Audio");

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &n64audio_pcm_ops);
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);

	strcpy(card->driver, "N64 Audio");
	strcpy(card->shortname, "N64 Audio");
	strcpy(card->longname, "N64 Audio");

	err = snd_card_register(card);
	if (err < 0)
		goto fail_alloc;

	platform_set_drvdata(pdev, priv);

	return 0;

fail_alloc:
	kfree(priv);

fail_card:
	snd_card_free(card);
	return err;
}

static struct platform_driver n64audio_driver = {
	.driver = {
		.name = "n64audio",
	},
};

static int __init n64audio_init(void)
{
	int ret;

	ret = platform_driver_probe(&n64audio_driver, n64audio_probe);

	return ret;
}

fs_initcall(n64audio_init);
