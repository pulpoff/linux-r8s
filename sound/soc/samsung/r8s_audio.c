// SPDX-License-Identifier: GPL-2.0
//
// Galaxy S20 FE (r8s) sound card: two CS35L40 amps on ABOX UAIF2, the
// digital microphones on ABOX UAIF6.
//
// The phone has no codec: the earpiece and loudspeaker amps share one DSP_A
// TDM bus that ABOX clocks (earpiece amp on slot 0, loudspeaker amp on slot
// 1, as stock's left/right amps), and the amps run their PLLs from BCLK.
// The PDM microphones reach ABOX through the VTS serial LIF, which clocks
// that DSP_A bus itself (stock: "bitclock-master", "bitclock-inversion").

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/cs35l41.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

static int r8s_amp_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	unsigned int bclk = params_rate(params) * params_channels(params) *
			    params_physical_width(params);
	struct snd_soc_dai *dai;
	unsigned int slot;
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, dai) {
		/* codec 0 = earpiece (slot 0), codec 1 = loudspeaker (slot 1) */
		slot = i;
		ret = snd_soc_component_set_sysclk(dai->component,
						   CS35L41_CLKID_SCLK, 0, bclk,
						   SND_SOC_CLOCK_IN);
		ret = ret ?: snd_soc_dai_set_sysclk(dai, CS35L41_CLKID_SCLK,
						    bclk, SND_SOC_CLOCK_IN);
		ret = ret ?: snd_soc_dai_set_channel_map(dai, 0, NULL, 1, &slot);
		if (ret) {
			dev_err(rtd->dev, "amp %d clock/slot setup: %d\n", i, ret);
			return ret;
		}
	}
	return 0;
}

static const struct snd_soc_ops r8s_amp_ops = {
	.hw_params = r8s_amp_hw_params,
};

SND_SOC_DAILINK_DEFS(speaker,
	DAILINK_COMP_ARRAY(COMP_CPU("abox-uaif2")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "cs35l41-pcm"),
			   COMP_CODEC(NULL, "cs35l41-pcm")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(mic,
	DAILINK_COMP_ARRAY(COMP_CPU("abox-uaif6")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "vts-slif")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link r8s_dai_links[] = {
	{
		.name = "Speaker",
		.stream_name = "Speaker",
		/* stock: format = "dsp_a", no inversion; ABOX provides the clocks */
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.playback_only = 1,
		.ops = &r8s_amp_ops,
		SND_SOC_DAILINK_REG(speaker),
	},
	{
		.name = "Mic",
		.stream_name = "Mic",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(mic),
	},
};

static struct snd_soc_card r8s_card = {
	.name = "S20FE",
	.owner = THIS_MODULE,
	.dai_link = r8s_dai_links,
	.num_links = ARRAY_SIZE(r8s_dai_links),
};

static int r8s_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *spk = &r8s_dai_links[0];
	struct snd_soc_dai_link *mic = &r8s_dai_links[1];
	struct device_node *cpu;

	cpu = of_parse_phandle(dev->of_node, "samsung,audio-cpu", 0);
	if (!cpu)
		return dev_err_probe(dev, -EINVAL, "missing samsung,audio-cpu\n");
	spk->cpus->of_node = cpu;
	spk->platforms->of_node = cpu;
	for (int i = 0; i < spk->num_codecs; i++) {
		spk->codecs[i].of_node = of_parse_phandle(dev->of_node,
							  "samsung,speaker-amps",
							  i);
		if (!spk->codecs[i].of_node)
			return dev_err_probe(dev, -EINVAL,
					     "missing samsung,speaker-amps %d\n",
					     i);
	}

	mic->cpus->of_node = cpu;
	mic->platforms->of_node = cpu;
	mic->codecs->of_node = of_parse_phandle(dev->of_node, "samsung,mic-codec", 0);
	if (!mic->codecs->of_node)
		return dev_err_probe(dev, -EINVAL, "missing samsung,mic-codec\n");

	r8s_card.dev = dev;
	return devm_snd_soc_register_card(dev, &r8s_card);
}

static const struct of_device_id r8s_audio_of_match[] = {
	{ .compatible = "samsung,r8s-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, r8s_audio_of_match);

static struct platform_driver r8s_audio_driver = {
	.driver = {
		.name = "r8s-audio",
		.of_match_table = r8s_audio_of_match,
	},
	.probe = r8s_audio_probe,
};
module_platform_driver(r8s_audio_driver);

MODULE_DESCRIPTION("Galaxy S20 FE sound card (CS35L40 amps and DMICs on ABOX)");
MODULE_LICENSE("GPL");
