// SPDX-License-Identifier: GPL-2.0
//
// Exynos 990 VTS serial LIF: the digital microphones' way into ABOX.
//
// The PDM microphones hang off the VTS (voice trigger) block. Its DMIC_AUD
// decimators turn each PDM line into a left/right PCM pair, and the serial
// LIF sends the pairs to ABOX UAIF6 as a DSP_A TDM stream, providing BCLK
// and the frame sync. This drives that path from the AP, without the VTS
// firmware.
//
// Everything runs from the VTS RC oscillator, as on stock: 24.576 MHz, so
// DMIC_AUD gets 12.288 MHz (/256 = 48 kHz), the PDM clock on the pads
// 3.072 MHz, and the serial LIF BCLK 48 kHz x 2 x 16 bit.
//
// Two slots only. With more, the slots come out rotated by a whole number of
// decimator pairs, and by how much depends on the channel counts used
// before: the LIF keeps that phase across stop/start, draining and refilling,
// and only a change of the channel count moves it. Two slots are one pair,
// so they always line up; "Mic Select" puts either mic or both in them.
// S-Boot leaves the VTS power domain on, every CMU_VTS gate open and the
// VTS pad retention released.

#include <linux/bitfield.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* serial LIF */
#define SLIF_CTRL			0x100
#define SLIF_CTRL_MASTER_IF_EN		BIT(0)
#define SLIF_CTRL_SPU_EN		BIT(8)
#define SLIF_CONFIG_MASTER		0x104
#define SLIF_CFG_OP_C			GENMASK(2, 0)	/* channels - 1 */
#define SLIF_CFG_OP_D_16BIT		BIT(3)		/* clear: 24 bit */
#define SLIF_CFG_WS_MODE		BIT(4)
#define SLIF_CFG_WS_POLAR		BIT(8)
#define SLIF_CFG_DIFF_MSIF_STR		GENMASK(15, 12)
#define SLIF_CONFIG_DONE		0x10c
#define SLIF_DONE_ALL			BIT(0)
#define SLIF_DONE_MASTER		BIT(4)
#define SLIF_INPUT_EN			0x114
#define SLIF_INPUT_EN_ALL		GENMASK(2, 0)
/*
 * Inputs stay muted this long after a start: the decimators' first samples
 * are a near full-scale click and their high-pass filter then settles.
 */
#define SLIF_START_MUTE_MS		50
#define SLIF_CHANNEL_MAP		0x138	/* 4 bits per slot: its source */
#define SLIF_CHANNEL_MAP_DEFAULT	0x76543210
#define SLIF_CHANNEL_MAP_PAIR		GENMASK(7, 0)	/* slots 0 and 1 */

/* DMIC_AUD decimators, one per PDM line, each a left/right pair */
#define DMIC_AUD_NUM			3
#define DMIC_AUD_ENABLE			0x0
#define DMIC_AUD_ENABLE_ON		0xa0030000
#define DMIC_AUD_ENABLE_OFF		0x20030000
#define DMIC_AUD_CONTROL		0x4
#define DMIC_AUD_CONTROL_DEFAULT	0x8c63004c	/* HPF on, stock gain, 48 kHz */
#define DMIC_AUD_GAIN_MODE		0x8
#define DMIC_AUD_GAIN_MODE_DEFAULT	0x1717
#define DMIC_AUD_MAX_SCALE_GAIN		0x10
#define DMIC_AUD_MAX_SCALE_DEFAULT	0x85

/* VTS SFR at 0x15511000 */
#define VTS_DMIC_IF_ENABLE		0x00
#define VTS_DMIC_IF_AUD_ALL		GENMASK(8, 6)
#define VTS_SEL_PAD_AUD			0x10
#define VTS_SEL_DIV2_CLK		0x14

/* CMU_VTS dividers at 0x15501800 */
#define CMU_DIV_DMIC_AUD		0x04
#define CMU_DIV_DMIC_AUD_DIV2		0x08
#define CMU_DIV_DMIC_AUD_PAD		0x0c
#define CMU_DIV_SERIAL_LIF		0x1c
#define CMU_DIV_BUSY			BIT(16)

#define VTS_RCO_RATE			24576000UL
#define DMIC_AUD_RATE			12288000UL
#define DMIC_PAD_RATE			3072000UL

struct vts_slif {
	struct device *dev;
	void __iomem *sfr;
	void __iomem *dmic[DMIC_AUD_NUM];
	void __iomem *vts;
	void __iomem *cmu;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_on;
	struct pinctrl_state *pins_off;
	struct regulator_bulk_data supplies[2];
	unsigned int fmt;
	unsigned int mic;		/* index into slif_mic_map */
	struct hrtimer unmute;
};

/*
 * What the first two slots carry. DMIC_AUD0 (PDM line 0, the main mic at the
 * bottom) fills sources 0/1, DMIC_AUD1 (line 1, the sub mic at the top) 2/3.
 */
static const char * const slif_mic_texts[] = { "Main", "Sub", "Main + Sub" };
static const u8 slif_mic_map[] = { 0x10, 0x32, 0x20 };
static SOC_ENUM_SINGLE_EXT_DECL(slif_mic_enum, slif_mic_texts);

static void slif_write_map(struct vts_slif *slif)
{
	writel((SLIF_CHANNEL_MAP_DEFAULT & ~SLIF_CHANNEL_MAP_PAIR) |
	       slif_mic_map[slif->mic], slif->sfr + SLIF_CHANNEL_MAP);
}

static int slif_mic_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct vts_slif *slif = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.enumerated.item[0] = slif->mic;
	return 0;
}

/* the map can change while the LIF runs: the next frame takes it */
static int slif_mic_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct vts_slif *slif = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mic = ucontrol->value.enumerated.item[0];

	if (mic >= ARRAY_SIZE(slif_mic_map))
		return -EINVAL;
	if (mic == slif->mic)
		return 0;
	slif->mic = mic;
	slif_write_map(slif);
	return 1;
}

static const struct snd_kcontrol_new slif_controls[] = {
	SOC_ENUM_EXT("Mic Select", slif_mic_enum, slif_mic_get, slif_mic_put),
};

static void slif_update(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

static int slif_cmu_div(struct vts_slif *slif, u32 reg, unsigned long ratio)
{
	u32 val;

	writel(ratio - 1, slif->cmu + reg);
	return readl_poll_timeout(slif->cmu + reg, val, !(val & CMU_DIV_BUSY),
				  1, 1000);
}

static int slif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct vts_slif *slif = snd_soc_dai_get_drvdata(dai);

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A)
		return -EINVAL;
	/* the serial LIF always drives BCLK and the frame sync */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBP_CFP)
		return -EINVAL;

	slif->fmt = fmt;
	return 0;
}

static int slif_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct vts_slif *slif = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(slif->supplies), slif->supplies);
	if (ret)
		return ret;
	ret = pinctrl_select_state(slif->pinctrl, slif->pins_on);
	if (ret)
		regulator_bulk_disable(ARRAY_SIZE(slif->supplies),
				       slif->supplies);
	return ret;
}

static void slif_shutdown(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct vts_slif *slif = snd_soc_dai_get_drvdata(dai);

	pinctrl_select_state(slif->pinctrl, slif->pins_off);
	regulator_bulk_disable(ARRAY_SIZE(slif->supplies), slif->supplies);
}

static int slif_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct vts_slif *slif = snd_soc_dai_get_drvdata(dai);
	unsigned int ch = params_channels(params);
	unsigned long bclk = params_rate(params) * ch * 16;
	u32 cfg;
	int ret;

	if (params_width(params) != 16 || VTS_RCO_RATE % bclk) {
		dev_err(slif->dev, "%u ch x %d bit: no exact BCLK\n", ch,
			params_width(params));
		return -EINVAL;
	}

	ret = slif_cmu_div(slif, CMU_DIV_DMIC_AUD, VTS_RCO_RATE / DMIC_AUD_RATE);
	ret = ret ?: slif_cmu_div(slif, CMU_DIV_DMIC_AUD_PAD,
				  DMIC_AUD_RATE / DMIC_PAD_RATE);
	ret = ret ?: slif_cmu_div(slif, CMU_DIV_DMIC_AUD_DIV2,
				  DMIC_AUD_RATE / DMIC_PAD_RATE);
	ret = ret ?: slif_cmu_div(slif, CMU_DIV_SERIAL_LIF, VTS_RCO_RATE / bclk);
	if (ret)
		return ret;

	cfg = readl(slif->sfr + SLIF_CONFIG_MASTER);
	cfg &= ~(SLIF_CFG_OP_C | SLIF_CFG_WS_POLAR);
	cfg |= FIELD_PREP(SLIF_CFG_OP_C, ch - 1) | SLIF_CFG_OP_D_16BIT;
	switch (slif->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
		cfg |= SLIF_CFG_WS_POLAR;
		break;
	}
	writel(cfg, slif->sfr + SLIF_CONFIG_MASTER);

	dev_dbg(slif->dev, "%u ch x 16 bit, BCLK %lu Hz, CONFIG_MASTER 0x%08x\n",
		ch, bclk, cfg);
	return 0;
}

static enum hrtimer_restart slif_unmute(struct hrtimer *timer)
{
	struct vts_slif *slif = container_of(timer, struct vts_slif, unmute);

	writel(SLIF_INPUT_EN_ALL, slif->sfr + SLIF_INPUT_EN);
	return HRTIMER_NORESTART;
}

/*
 * The vendor's order: config done, pads, DMIC_IF, decimators, LIF; the
 * inputs follow once the decimators have settled.
 */
static void slif_enable(struct vts_slif *slif)
{
	int i;

	slif_update(slif->sfr + SLIF_CONFIG_DONE,
		    SLIF_DONE_MASTER | SLIF_DONE_ALL,
		    SLIF_DONE_MASTER | SLIF_DONE_ALL);
	writel(0x7, slif->vts + VTS_SEL_PAD_AUD);
	writel(0x38, slif->vts + VTS_SEL_DIV2_CLK);
	slif_update(slif->vts + VTS_DMIC_IF_ENABLE, VTS_DMIC_IF_AUD_ALL,
		    VTS_DMIC_IF_AUD_ALL);
	for (i = 0; i < DMIC_AUD_NUM; i++)
		writel(DMIC_AUD_ENABLE_ON, slif->dmic[i] + DMIC_AUD_ENABLE);
	writel(0, slif->sfr + SLIF_INPUT_EN);
	writel(SLIF_CTRL_MASTER_IF_EN | SLIF_CTRL_SPU_EN,
	       slif->sfr + SLIF_CTRL);
	hrtimer_start(&slif->unmute, ms_to_ktime(SLIF_START_MUTE_MS),
		      HRTIMER_MODE_REL);
}

static void slif_disable(struct vts_slif *slif)
{
	int i;

	hrtimer_cancel(&slif->unmute);
	writel(0, slif->sfr + SLIF_CTRL);
	writel(0, slif->sfr + SLIF_INPUT_EN);
	for (i = 0; i < DMIC_AUD_NUM; i++)
		writel(DMIC_AUD_ENABLE_OFF, slif->dmic[i] + DMIC_AUD_ENABLE);
	slif_update(slif->vts + VTS_DMIC_IF_ENABLE, VTS_DMIC_IF_AUD_ALL, 0);
	writel(0, slif->vts + VTS_SEL_PAD_AUD);
	writel(0, slif->vts + VTS_SEL_DIV2_CLK);
	writel(0, slif->sfr + SLIF_CONFIG_DONE);
}

static int slif_trigger(struct snd_pcm_substream *substream, int cmd,
			struct snd_soc_dai *dai)
{
	struct vts_slif *slif = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		slif_enable(slif);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		slif_disable(slif);
		return 0;
	}
	return -EINVAL;
}

static const struct snd_soc_dai_ops slif_dai_ops = {
	.set_fmt	= slif_set_fmt,
	.startup	= slif_startup,
	.shutdown	= slif_shutdown,
	.hw_params	= slif_hw_params,
	.trigger	= slif_trigger,
};

static struct snd_soc_dai_driver slif_dai = {
	.name = "vts-slif",
	.capture = {
		.stream_name = "Serial LIF Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &slif_dai_ops,
};

static const struct snd_soc_component_driver slif_component = {
	.name = "exynos990-vts-slif",
	.controls = slif_controls,
	.num_controls = ARRAY_SIZE(slif_controls),
};

static int slif_probe(struct platform_device *pdev)
{
	static const char * const dmic_names[] = { "dmic0", "dmic1", "dmic2" };
	struct device *dev = &pdev->dev;
	struct vts_slif *slif;
	int i, ret;

	slif = devm_kzalloc(dev, sizeof(*slif), GFP_KERNEL);
	if (!slif)
		return -ENOMEM;
	slif->dev = dev;
	platform_set_drvdata(pdev, slif);
	hrtimer_setup(&slif->unmute, slif_unmute, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);

	slif->sfr = devm_platform_ioremap_resource_byname(pdev, "sfr");
	if (IS_ERR(slif->sfr))
		return PTR_ERR(slif->sfr);
	for (i = 0; i < DMIC_AUD_NUM; i++) {
		slif->dmic[i] = devm_platform_ioremap_resource_byname(pdev,
								      dmic_names[i]);
		if (IS_ERR(slif->dmic[i]))
			return PTR_ERR(slif->dmic[i]);
	}
	slif->vts = devm_platform_ioremap_resource_byname(pdev, "vts");
	if (IS_ERR(slif->vts))
		return PTR_ERR(slif->vts);
	slif->cmu = devm_platform_ioremap_resource_byname(pdev, "cmu");
	if (IS_ERR(slif->cmu))
		return PTR_ERR(slif->cmu);

	slif->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(slif->pinctrl))
		return dev_err_probe(dev, PTR_ERR(slif->pinctrl), "no pinctrl\n");
	slif->pins_on = pinctrl_lookup_state(slif->pinctrl, "default");
	if (IS_ERR(slif->pins_on))
		return dev_err_probe(dev, PTR_ERR(slif->pins_on),
				     "no default pin state\n");
	slif->pins_off = pinctrl_lookup_state(slif->pinctrl, "idle");
	if (IS_ERR(slif->pins_off))
		return dev_err_probe(dev, PTR_ERR(slif->pins_off),
				     "no idle pin state\n");

	/* the microphones' supplies, one per PDM line in use */
	slif->supplies[0].supply = "mic0";
	slif->supplies[1].supply = "mic1";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(slif->supplies),
				      slif->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "no mic supplies\n");

	/* stock defaults: gains, HPF, 48 kHz decimation; the main mic */
	for (i = 0; i < DMIC_AUD_NUM; i++) {
		writel(DMIC_AUD_ENABLE_OFF, slif->dmic[i] + DMIC_AUD_ENABLE);
		writel(DMIC_AUD_GAIN_MODE_DEFAULT,
		       slif->dmic[i] + DMIC_AUD_GAIN_MODE);
		writel(DMIC_AUD_MAX_SCALE_DEFAULT,
		       slif->dmic[i] + DMIC_AUD_MAX_SCALE_GAIN);
		writel(DMIC_AUD_CONTROL_DEFAULT,
		       slif->dmic[i] + DMIC_AUD_CONTROL);
	}
	slif_write_map(slif);
	writel(0, slif->sfr + SLIF_CTRL);
	pinctrl_select_state(slif->pinctrl, slif->pins_off);

	return devm_snd_soc_register_component(dev, &slif_component, &slif_dai, 1);
}

static const struct of_device_id slif_of_match[] = {
	{ .compatible = "samsung,exynos990-vts-slif" },
	{ }
};
MODULE_DEVICE_TABLE(of, slif_of_match);

static struct platform_driver slif_driver = {
	.driver = {
		.name = "exynos990-vts-slif",
		.of_match_table = slif_of_match,
	},
	.probe = slif_probe,
};
module_platform_driver(slif_driver);

MODULE_DESCRIPTION("Exynos 990 VTS serial LIF (digital microphones to ABOX)");
MODULE_LICENSE("GPL");
