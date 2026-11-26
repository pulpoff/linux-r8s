// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 990 DisplayPort audio (Galaxy S20 FE): the DP controller's SST1 audio on the link that exynos990-dp
 * brought up, programmed as stock's cal_9830/displayport_reg.c (displayport_audio_enable / _disable /
 * _wait_buf_full) and sound/soc/samsung/dp_dma.c (the trigger order) do it. Samples reach the controller's audio
 * FIFO (DP + 0x5818) through PDMA0, a PL330 in BLK_BUS1, on request 28, as stock's dp_dma@0.
 *
 * The DP's registers belong to exynos990-dp, which holds their clocks while bound: they are mapped here without
 * being requested, from the node "samsung,dp" points at (reg 0 = the link, reg 1 = the combo PHY's PMA, where the
 * trained link rate is read, reg 3 = USBCON), and nothing is read before that driver is bound (the probe defers;
 * a device link unbinds this one first). The PMA is read only while it answers (the PMU's isolation off, USBCON's
 * PMA reset clear: a PMA read in reset hangs the bus). Playback starts only while the DP's video stream runs;
 * "HDMI Jack" follows it.
 *
 * One bit stock never sets but needs clearing here: SST1_AUDIO_CONTROL's AUD_DMA_IF_MODE_CONFIG is 1 at reset,
 * and with it set the controller never raises its DMA request (the PL330 waits on request 28 forever).
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <sound/dmaengine_pcm.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define SST1_FUNCTION_EN		0x001c
#define  SST1_LH_PWR_ON_STATUS		BIT(5)
#define  SST1_AUDIO_FIFO_FUNC_EN	BIT(2)
#define  SST1_AUDIO_FUNC_EN		BIT(1)
#define  SST1_VIDEO_FUNC_EN		BIT(0)
#define SST1_MAIN_CONTROL		0x5000
#define  MAUD_MODE			BIT(10)
#define SST1_MAIN_FIFO_CONTROL		0x5004
#define  CLEAR_AUDIO_FIFO		BIT(3)
#define SST1_MAUD_MASTER_MODE		0x505c
#define SST1_NAUD_MASTER_MODE		0x5060
#define SST1_MAUD_SFR			0x5064
#define SST1_NAUD_SFR			0x5068
#define SST1_AUDIO_CONTROL		0x5800
#define  DMA_REQ_GEN_EN			BIT(30)
#define  AUD_DMA_IF_MODE_CONFIG		BIT(25)
#define  DMA_BURST_SEL			GENMASK(20, 18)
#define  AUDIO_BIT_MAPPING		GENMASK(17, 16)
#define  PCM_SIZE			GENMASK(14, 13)
#define  AUDIO_CH_STATUS_SAME		BIT(5)
#define SST1_AUDIO_ENABLE		0x5804
#define  AUDIO_EN			BIT(0)
#define SST1_AUDIO_MASTER_TIMING_GEN	0x5808
#define  AUDIO_MASTER_TIME_GEN		BIT(0)
#define SST1_AUDIO_DMA_REQUEST		0x580c
#define  AUD_DMA_FORCE_REQ_VAL		BIT(17)
#define  AUD_DMA_FORCE_REQ_SEL		BIT(16)
#define SST1_AUDIO_FIFO			0x5818
#define SST1_AUDIO_BIST_CONTROL		0x5830
#define  AUD_BIST_EN			BIT(0)
#define SST1_AUDIO_BIST_CH_STATUS0	0x5834
#define SST1_AUDIO_BIST_CH_STATUS1	0x5838
#define SST1_AUDIO_BUFFER_CONTROL	0x583c
#define  MASTER_AUDIO_CH_COUNT		GENMASK(16, 12)
#define SST1_AUDIO_CH_1_4_REMAP		0x5840
#define SST1_AUDIO_CH_5_8_REMAP		0x5844
#define SST1_AUDIO_CH_1_2_STATUS0	0x5860
#define SST1_AUDIO_CH_1_2_STATUS1	0x5864
/* the IEC 60958 channel status, in both the BIST and the stream copies */
#define  CS_CLK_ACCUR			GENMASK(29, 28)
#define  CS_FS_FREQ			GENMASK(27, 24)
#define  CS_CH_NUM			GENMASK(23, 20)
#define  CS_SOURCE_NUM			GENMASK(19, 16)
#define  CS_WORD_LENGTH			GENMASK(3, 1)
#define  CS_WORD_MAX			BIT(0)
#define SST1_INFOFRAME_UPDATE		0x5c04
#define SST1_INFOFRAME_SEND		0x5c08
#define  AUDIO_INFO			BIT(3)
#define SST1_INFOFRAME_AUDIO0		0x5c50
#define SST1_INFOFRAME_AUDIO1		0x5c54
#define SST1_INFOFRAME_AUDIO2		0x5c58

/* the combo PHY's PMA: the link rate exynos990-dp trained */
#define CMN_REG00B9			0x02e4
#define  DP_TX_LINK_BW			GENMASK(5, 4)
/* the PMA answers only with USBCON's PMA_APB_SW_RST clear and the PMU's USB/DP PHY isolation off */
#define USBCON_PHY_RESET		0x0048
#define  PMA_APB_SW_RST			BIT(0)
#define PMU_USBDP_PHY_CONTROL		0x0704
#define  PHY_ISOLATION_OFF		BIT(0)

#define PCM_16				0
#define PCM_20				1
#define PCM_24				2
/* AUDIO_BIT_MAPPING: a 32-bit container as it is, or two 16-bit samples to a word */
#define MAPPING_NORMAL			0
#define MAPPING_PACKED2			2
/* CS_CLK_ACCUR: stock says NOT_MATCH, the audio clock being made from the link's */
#define CLK_ACCUR_NOT_MATCH		3

#define JACK_POLL_MS			500

struct dp_audio {
	struct device *dev;
	void __iomem *regs, *pma, *usbcon;
	struct regmap *pmu;
	u32 khz;			/* the link rate, read at prepare */
	struct snd_dmaengine_dai_dma_data dma;
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component comps[3];
	struct snd_soc_jack jack;
	struct snd_soc_jack_pin jack_pin;
	struct delayed_work poll;
	bool jack_ready, plugged;
	spinlock_t lock;		/* the audio registers: prepare/trigger against the jack poll */
	unsigned int rate, channels, pcm_size, mapping, burst;
};

static void dpa_update(struct dp_audio *a, u32 reg, u32 mask, u32 val)
{
	u32 v = readl(a->regs + reg);

	writel((v & ~mask) | (val & mask), a->regs + reg);
}

/* exynos990-dp's video stream is running, with its LH channel */
static bool dpa_link_up(struct dp_audio *a)
{
	u32 v = readl(a->regs + SST1_FUNCTION_EN);

	return (v & (SST1_VIDEO_FUNC_EN | SST1_LH_PWR_ON_STATUS)) == (SST1_VIDEO_FUNC_EN | SST1_LH_PWR_ON_STATUS);
}

/* the trained link's symbol clock, 0 while the PMA would not answer */
static u32 dpa_link_khz(struct dp_audio *a)
{
	unsigned int pmu = 0;
	u32 bw;

	regmap_read(a->pmu, PMU_USBDP_PHY_CONTROL, &pmu);
	if (!(pmu & PHY_ISOLATION_OFF) || (readl(a->usbcon + USBCON_PHY_RESET) & PMA_APB_SW_RST))
		return 0;
	bw = FIELD_GET(DP_TX_LINK_BW, readl(a->pma + CMN_REG00B9));
	return bw == 3 ? 810000 : bw == 2 ? 540000 : bw == 1 ? 270000 : 162000;
}

/* IEC 60958 sampling frequency */
static int dpa_cs_fs(unsigned int rate)
{
	switch (rate) {
	case 32000:
		return 0x3;
	case 44100:
		return 0x0;
	case 48000:
		return 0x2;
	case 88200:
		return 0x8;
	case 96000:
		return 0xa;
	case 176400:
		return 0xc;
	case 192000:
		return 0xe;
	}
	return -EINVAL;
}

/* displayport_audio_enable() with its infoframe, timing generator last */
static void dpa_start(struct dp_audio *a)
{
	u32 khz = a->khz;
	u32 cs0 = FIELD_PREP(CS_CLK_ACCUR, CLK_ACCUR_NOT_MATCH) | FIELD_PREP(CS_FS_FREQ, dpa_cs_fs(a->rate)) |
		  FIELD_PREP(CS_CH_NUM, a->channels) | FIELD_PREP(CS_SOURCE_NUM, a->channels);
	u32 cs1 = a->pcm_size == PCM_24 ? FIELD_PREP(CS_WORD_LENGTH, 5) | CS_WORD_MAX :
		  FIELD_PREP(CS_WORD_LENGTH, a->pcm_size == PCM_20 ? 5 : 1);

	/*
	 * Asynchronous: Maud/Naud = 512 fs / the link's symbol clock (stock's audio_async_m_n, computed), and the
	 * sample clock made from the link: fs = LS/2 * MAUD_MASTER / NAUD_MASTER.
	 */
	dpa_update(a, SST1_MAIN_CONTROL, MAUD_MODE, 0);
	writel(DIV_ROUND_CLOSEST_ULL(512ULL * a->rate * 32768, khz * 1000ULL), a->regs + SST1_MAUD_SFR);
	writel(32768, a->regs + SST1_NAUD_SFR);
	dpa_update(a, SST1_FUNCTION_EN, SST1_AUDIO_FUNC_EN, SST1_AUDIO_FUNC_EN);
	dpa_update(a, SST1_AUDIO_CONTROL, AUD_DMA_IF_MODE_CONFIG | DMA_BURST_SEL | PCM_SIZE | AUDIO_BIT_MAPPING,
		   FIELD_PREP(DMA_BURST_SEL, a->burst - 1) | FIELD_PREP(PCM_SIZE, a->pcm_size) |
		   FIELD_PREP(AUDIO_BIT_MAPPING, a->mapping));
	dpa_update(a, SST1_AUDIO_BUFFER_CONTROL, MASTER_AUDIO_CH_COUNT,
		   FIELD_PREP(MASTER_AUDIO_CH_COUNT, a->channels - 1));
	dpa_update(a, SST1_MAIN_FIFO_CONTROL, CLEAR_AUDIO_FIFO, 0);
	dpa_update(a, SST1_FUNCTION_EN, SST1_AUDIO_FIFO_FUNC_EN, SST1_AUDIO_FIFO_FUNC_EN);
	writel(a->rate, a->regs + SST1_MAUD_MASTER_MODE);
	writel(khz * 500, a->regs + SST1_NAUD_MASTER_MODE);
	/* stock's order: FL, FR, then C and LFE swapped */
	writel(3 << 24 | 4 << 16 | 2 << 8 | 1, a->regs + SST1_AUDIO_CH_1_4_REMAP);
	writel(8 << 24 | 7 << 16 | 6 << 8 | 5, a->regs + SST1_AUDIO_CH_5_8_REMAP);
	dpa_update(a, SST1_AUDIO_ENABLE, AUDIO_EN, AUDIO_EN);
	dpa_update(a, SST1_AUDIO_BIST_CH_STATUS0, CS_CLK_ACCUR | CS_FS_FREQ | CS_CH_NUM | CS_SOURCE_NUM, cs0);
	dpa_update(a, SST1_AUDIO_CH_1_2_STATUS0, CS_CLK_ACCUR | CS_FS_FREQ | CS_CH_NUM | CS_SOURCE_NUM, cs0);
	dpa_update(a, SST1_AUDIO_BIST_CH_STATUS1, CS_WORD_LENGTH | CS_WORD_MAX, cs1);
	dpa_update(a, SST1_AUDIO_CH_1_2_STATUS1, CS_WORD_LENGTH | CS_WORD_MAX, cs1);
	dpa_update(a, SST1_AUDIO_CONTROL, AUDIO_CH_STATUS_SAME, AUDIO_CH_STATUS_SAME);
	dpa_update(a, SST1_AUDIO_MASTER_TIMING_GEN, AUDIO_MASTER_TIME_GEN, AUDIO_MASTER_TIME_GEN);

	/* the audio InfoFrame: PB1 the channel count - 1 (coding and size from the stream), PB4 0 = FL/FR */
	writel(a->channels - 1, a->regs + SST1_INFOFRAME_AUDIO0);
	writel(0, a->regs + SST1_INFOFRAME_AUDIO1);
	writel(0, a->regs + SST1_INFOFRAME_AUDIO2);
	dpa_update(a, SST1_INFOFRAME_UPDATE, AUDIO_INFO, AUDIO_INFO);
	dpa_update(a, SST1_INFOFRAME_SEND, AUDIO_INFO, AUDIO_INFO);
}

/* displayport_audio_wait_buf_full(), then displayport_audio_disable() (atomic: udelay as stock) */
static void dpa_stop(struct dp_audio *a)
{
	dpa_update(a, SST1_AUDIO_MASTER_TIMING_GEN, AUDIO_MASTER_TIME_GEN, 0);
	dpa_update(a, SST1_AUDIO_CONTROL, DMA_REQ_GEN_EN, 0);
	dpa_update(a, SST1_AUDIO_BIST_CONTROL, AUD_BIST_EN, 0);
	if (readl(a->regs + SST1_AUDIO_ENABLE) & AUDIO_EN) {
		udelay(1000);
		dpa_update(a, SST1_AUDIO_ENABLE, AUDIO_EN, 0);
		dpa_update(a, SST1_FUNCTION_EN, SST1_AUDIO_FIFO_FUNC_EN, 0);
		dpa_update(a, SST1_MAIN_FIFO_CONTROL, CLEAR_AUDIO_FIFO, CLEAR_AUDIO_FIFO);
	}
	dpa_update(a, SST1_INFOFRAME_UPDATE, AUDIO_INFO, 0);
	dpa_update(a, SST1_INFOFRAME_SEND, AUDIO_INFO, 0);
}

static int dpa_dai_probe(struct snd_soc_dai *dai)
{
	struct dp_audio *a = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &a->dma, NULL);
	return 0;
}

static int dpa_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	return dpa_link_up(snd_soc_dai_get_drvdata(dai)) ? 0 : -ENODEV;
}

static int dpa_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct dp_audio *a = snd_soc_dai_get_drvdata(dai);
	unsigned int width = params_physical_width(params);

	switch (params_width(params)) {
	case 16:
		a->pcm_size = PCM_16;
		break;
	case 24:
		a->pcm_size = PCM_24;
		break;
	default:
		return -EINVAL;
	}
	if (dpa_cs_fs(params_rate(params)) < 0)
		return -EINVAL;
	a->rate = params_rate(params);
	a->channels = params_channels(params);
	a->mapping = width == 32 ? MAPPING_NORMAL : MAPPING_PACKED2;
	/* a burst is one frame of words, as stock asks the PL330 for */
	a->burst = width * a->channels / 32;
	a->dma.maxburst = a->burst;
	return 0;
}

static int dpa_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct dp_audio *a = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	if (!dpa_link_up(a))
		return -ENODEV;
	a->khz = dpa_link_khz(a);
	if (!a->khz)
		return -ENODEV;
	spin_lock_irqsave(&a->lock, flags);
	dpa_stop(a);
	spin_unlock_irqrestore(&a->lock, flags);
	return 0;
}

/* after the DMA starts and before it stops (ASoC's default order), as stock's dma_trigger() */
static int dpa_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct dp_audio *a = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	spin_lock_irqsave(&a->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * Stock's displayport_reg_set_dma_force_req_low() around the start: the request held low drops a
		 * handshake left half done by an earlier stream (0x580c read REQ and ACK both set while the PL330
		 * waited on request 28 for good; this pulse got it going again).
		 */
		dpa_update(a, SST1_AUDIO_DMA_REQUEST, AUD_DMA_FORCE_REQ_SEL | AUD_DMA_FORCE_REQ_VAL,
			   AUD_DMA_FORCE_REQ_SEL);
		dpa_start(a);
		dpa_update(a, SST1_AUDIO_CONTROL, DMA_REQ_GEN_EN, DMA_REQ_GEN_EN);
		udelay(100);
		dpa_update(a, SST1_AUDIO_DMA_REQUEST, AUD_DMA_FORCE_REQ_SEL | AUD_DMA_FORCE_REQ_VAL, 0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dpa_stop(a);
		break;
	default:
		spin_unlock_irqrestore(&a->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&a->lock, flags);
	return 0;
}

static const struct snd_soc_dai_ops dpa_dai_ops = {
	.probe = dpa_dai_probe,
	.startup = dpa_startup,
	.hw_params = dpa_hw_params,
	.prepare = dpa_prepare,
	.trigger = dpa_trigger,
};

#define DPA_RATES	(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
			 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			 SNDRV_PCM_RATE_192000)
#define DPA_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver dpa_dai = {
	.name = "exynos990-dp-audio",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = DPA_RATES,
		.formats = DPA_FORMATS,
	},
	.ops = &dpa_dai_ops,
};

static const struct snd_soc_component_driver dpa_component = {
	.name = "exynos990-dp-audio",
	.legacy_dai_naming = 1,
};

static const struct snd_pcm_hardware dpa_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = DPA_FORMATS,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 128,
	.fifo_size = 32,
};

static const struct snd_dmaengine_pcm_config dpa_pcm_config = {
	.pcm_hardware = &dpa_pcm_hw,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.prealloc_buffer_size = 256 * 1024,
};

/* "HDMI Jack": in while the DP's video stream runs (a sink that has a picture can play) */
static void dpa_poll(struct work_struct *work)
{
	struct dp_audio *a = container_of(work, struct dp_audio, poll.work);
	bool up = dpa_link_up(a);

	if (up != a->plugged) {
		a->plugged = up;
		snd_soc_jack_report(&a->jack, up ? SND_JACK_LINEOUT : 0, SND_JACK_LINEOUT);
	}
	schedule_delayed_work(&a->poll, msecs_to_jiffies(JACK_POLL_MS));
}

static int dpa_card_late_probe(struct snd_soc_card *card)
{
	struct dp_audio *a = snd_soc_card_get_drvdata(card);
	int ret;

	a->jack_pin.pin = "HDMI";
	a->jack_pin.mask = SND_JACK_LINEOUT;
	ret = snd_soc_card_jack_new_pins(card, "HDMI Jack", SND_JACK_LINEOUT, &a->jack, &a->jack_pin, 1);
	if (ret)
		return ret;
	a->jack_ready = true;
	schedule_delayed_work(&a->poll, 0);
	return 0;
}

static const struct snd_soc_dapm_widget dpa_widgets[] = {
	SND_SOC_DAPM_LINE("HDMI", NULL),
};

static int dpa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *dp_pdev;
	struct device_node *dp_np;
	struct resource res;
	struct dp_audio *a;
	int ret;

	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	a->dev = dev;
	spin_lock_init(&a->lock);
	INIT_DELAYED_WORK(&a->poll, dpa_poll);

	dp_np = of_parse_phandle(dev->of_node, "samsung,dp", 0);
	if (!dp_np)
		return dev_err_probe(dev, -EINVAL, "no samsung,dp\n");
	/* the DP's clocks are on only while exynos990-dp is bound: nothing is read before */
	dp_pdev = of_find_device_by_node(dp_np);
	if (!dp_pdev || !dp_pdev->dev.driver) {
		if (dp_pdev)
			put_device(&dp_pdev->dev);
		of_node_put(dp_np);
		return dev_err_probe(dev, -EPROBE_DEFER, "waiting for exynos990-dp\n");
	}
	if (!device_link_add(dev, &dp_pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		dev_warn(dev, "no device link to %s\n", dev_name(&dp_pdev->dev));
	put_device(&dp_pdev->dev);
	a->pmu = syscon_regmap_lookup_by_phandle(dp_np, "samsung,pmu-syscon");
	ret = of_address_to_resource(dp_np, 0, &res);
	/* not requested: exynos990-dp owns the link, the PMA and USBCON */
	a->regs = ret ? NULL : devm_ioremap(dev, res.start, resource_size(&res));
	a->pma = of_iomap(dp_np, 1);
	a->usbcon = of_iomap(dp_np, 3);
	of_node_put(dp_np);
	if (IS_ERR(a->pmu) || !a->regs || !a->pma || !a->usbcon) {
		if (a->pma)
			iounmap(a->pma);
		if (a->usbcon)
			iounmap(a->usbcon);
		return dev_err_probe(dev, IS_ERR(a->pmu) ? PTR_ERR(a->pmu) : -ENOMEM, "DP registers\n");
	}
	platform_set_drvdata(pdev, a);

	a->dma.addr = res.start + SST1_AUDIO_FIFO;
	a->dma.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	a->dma.maxburst = 1;

	ret = devm_snd_soc_register_component(dev, &dpa_component, &dpa_dai, 1);
	if (ret)
		goto err;
	/* the PL330 may not be there yet: a deferral brings us back */
	ret = devm_snd_dmaengine_pcm_register(dev, &dpa_pcm_config, 0);
	if (ret)
		goto err;

	a->comps[0].of_node = dev->of_node;		/* CPU: this DAI */
	a->comps[1].name = "snd-soc-dummy";		/* codec: the sink */
	a->comps[1].dai_name = "snd-soc-dummy-dai";
	a->comps[2].of_node = dev->of_node;		/* platform: dmaengine PCM on PDMA0 */
	a->link.name = "HDMI";
	a->link.stream_name = "HDMI";
	a->link.playback_only = 1;
	a->link.cpus = &a->comps[0];
	a->link.num_cpus = 1;
	a->link.codecs = &a->comps[1];
	a->link.num_codecs = 1;
	a->link.platforms = &a->comps[2];
	a->link.num_platforms = 1;
	a->card.name = "S20HDMI";
	a->card.owner = THIS_MODULE;
	a->card.dev = dev;
	a->card.dai_link = &a->link;
	a->card.num_links = 1;
	a->card.dapm_widgets = dpa_widgets;
	a->card.num_dapm_widgets = ARRAY_SIZE(dpa_widgets);
	a->card.late_probe = dpa_card_late_probe;
	snd_soc_card_set_drvdata(&a->card, a);
	ret = devm_snd_soc_register_card(dev, &a->card);
	if (ret)
		goto err;
	dev_info(dev, "card S20HDMI on PDMA0, link %s\n", dpa_link_up(a) ? "up" : "down");
	return 0;
err:
	iounmap(a->pma);
	iounmap(a->usbcon);
	return dev_err_probe(dev, ret, "registering the card\n");
}

static void dpa_remove(struct platform_device *pdev)
{
	struct dp_audio *a = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&a->poll);
	iounmap(a->pma);
	iounmap(a->usbcon);
}

static const struct of_device_id dpa_of_match[] = {
	{ .compatible = "samsung,exynos990-dp-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, dpa_of_match);

static struct platform_driver dpa_driver = {
	.probe = dpa_probe,
	.remove = dpa_remove,
	.driver = {
		.name = "exynos990-dp-audio",
		.of_match_table = dpa_of_match,
	},
};
module_platform_driver(dpa_driver);

MODULE_DESCRIPTION("Exynos 990 DisplayPort audio over PDMA0");
MODULE_LICENSE("GPL");
