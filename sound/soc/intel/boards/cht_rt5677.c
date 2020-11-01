// SPDX-License-Identifier: GPL-2.0-only
/*
 *  cht_rt5677.c - ASoc Machine driver for Intel Cherrytrail platforms with an
 *  RT5677 audio codec.
 *
 *  The driver supports Lenovo Yoga Book YB1-X90/X91 tablets and has some
 *  things hardcoded for these devices because there are no other known
 *  devices using such a configuration.
 *
 *  Copyright (C) 2026 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 *  Based on the mainline cht_bsw_rt5672.c and on the cht_bl_dpcm_rt5677.c
 *  from Lenovo Android kernel.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/dmi.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-acpi.h>
#include <sound/soc.h>
#include "../../codecs/rt5677.h"
#include "../../codecs/ts3a227e.h"
#include "../atom/sst-atom-controls.h"

#define RT5677_I2C_DEFAULT	"i2c-rt5677"

/* The platform clock #3 outputs 19.2Mhz clock to codec as I2S MCLK */
#define CHT_PLAT_CLK_3_HZ	19200000
#define CHT_CODEC_DAI		"rt5677-aif1"

struct cht_rt5677_private {
	struct snd_soc_card card;
	char codec_name[SND_ACPI_I2C_ID_LEN];
	struct snd_soc_jack jack;
	struct clk *mclk;
	struct gpio_desc *gpio_spk_en1;
	struct gpio_desc *gpio_spk_en2;
	struct gpio_desc *gpio_hp_en;
};

static int cht_rt5677_platform_clock_enable(struct snd_soc_card *card,
					struct snd_soc_dai *codec_dai)
{
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);
	int ret;

	ret = clk_prepare_enable(ctx->mclk);
	if (ret < 0) {
		dev_err(card->dev, "could not configure MCLK state: %d\n", ret);
		return ret;
	}

	/* set codec PLL source to the 19.2MHz platform clock (MCLK) */
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5677_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, 48000 * 512);
	if (ret < 0) {
		dev_err(card->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	/* set codec sysclk source to PLL */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_PLL1,
				     48000 * 512, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "can't set codec sysclk: %d\n", ret);
		return ret;
	}

	return 0;
}

static void cht_rt5677_platform_clock_disable(struct snd_soc_card *card,
					  struct snd_soc_dai *codec_dai)
{
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);

	/* Set codec sysclk source to its internal clock because codec
	 * PLL will be off when idle and MCLK will also be off by ACPI
	 * when codec is runtime suspended. Codec needs clock for jack
	 * detection and button press.
	 */
	snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_RCCLK,
			       48000 * 512, SND_SOC_CLOCK_IN);

	clk_disable_unprepare(ctx->mclk);
}

static int cht_rt5677_platform_clock_control(struct snd_soc_dapm_widget *w,
					 struct snd_kcontrol *kctl, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct snd_soc_dai *codec_dai;

	codec_dai = snd_soc_card_get_codec_dai(card, CHT_CODEC_DAI);
	if (!codec_dai) {
		dev_err(card->dev,
			"Codec dai not found; Unable to set platform clock\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_ON(event))
		return cht_rt5677_platform_clock_enable(card, codec_dai);

	cht_rt5677_platform_clock_disable(card, codec_dai);

	return 0;
}

static int cht_rt5677_hp_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kctl, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);

	gpiod_set_value_cansleep(ctx->gpio_hp_en, SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int cht_rt5677_spk_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kctl, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);

	gpiod_set_value_cansleep(ctx->gpio_spk_en1,
				 SND_SOC_DAPM_EVENT_ON(event));
	gpiod_set_value_cansleep(ctx->gpio_spk_en2,
				 SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget cht_rt5677_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", cht_rt5677_hp_event),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", cht_rt5677_spk_event),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			    cht_rt5677_platform_clock_control,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_rt5677_audio_map[] = {
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},
	{"DMIC L1", NULL, "Int Mic"},
	{"DMIC R1", NULL, "Int Mic"},
	{"Headphone", NULL, "LOUT1"},
	{"Headphone", NULL, "LOUT2"},
	{"Speaker", NULL, "LOUT1"},
	{"Speaker", NULL, "LOUT2"},

	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx"},
	{"codec_in1", NULL, "ssp2 Rx"},
	{"ssp2 Rx", NULL, "AIF1 Capture"},
	{"Headphone", NULL, "Platform Clock"},
	{"Speaker", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new cht_rt5677_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int cht_rt5677_aif1_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	/* set codec PLL source to the 19.2MHz platform clock (MCLK) */
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5677_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	/* set codec sysclk source to PLL */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_PLL1,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk: %d\n", ret);
		return ret;
	}

	/* Default mode for SSP configuration is TDM 4 slot */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBC_CFC);
	if (ret < 0) {
		dev_err(codec_dai->dev, "can't set format to TDM %d\n", ret);
		return ret;
	}

	/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4, 25);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec TDM slot %d\n", ret);
		return ret;
	}

	return 0;
}

static int cht_rt5677_codec_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(runtime, 0);
	struct snd_soc_component *component = codec_dai->component;
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	int ret = 0;

	/* Enable codec ASRC function for Stereo DAC/Stereo1 ADC/DMIC/I2S1.
	 * The ASRC clock source is clk_i2s1_asrc.
	 */
	rt5677_sel_asrc_clk_src(component, RT5677_DA_STEREO_FILTER |
			RT5677_AD_STEREO1_FILTER | RT5677_I2S1_SOURCE,
			RT5677_CLK_SEL_I2S1_ASRC);
	/* Enable codec ASRC function for Mono ADC L.
	 * The ASRC clock source is clk_sys2_asrc.
	 */
	rt5677_sel_asrc_clk_src(component, RT5677_AD_MONO_L_FILTER,
				RT5677_CLK_SEL_SYS2);

	/*
	 * The firmware might enable the clock at
	 * boot (this information may or may not
	 * be reflected in the enable clock register).
	 * To change the rate we must disable the clock
	 * first to cover these cases. Due to common
	 * clock framework restrictions that do not allow
	 * to disable a clock that has not been enabled,
	 * we need to enable the clock first.
	 */
	ret = clk_prepare_enable(ctx->mclk);
	if (!ret)
		clk_disable_unprepare(ctx->mclk);

	ret = clk_set_rate(ctx->mclk, CHT_PLAT_CLK_3_HZ);
	if (ret) {
		dev_err(runtime->dev, "unable to set MCLK rate\n");
		return ret;
	}

	return 0;
}

static int cht_rt5677_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The DSP will convert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static struct snd_soc_jack_pin cht_rt5677_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int cht_rt5677_headset_init(struct snd_soc_component *component)
{
	struct snd_soc_card *card = component->card;
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack *jack = &ctx->jack;
	int jack_type;
	int ret;

	/*
	 * TS3A227E supports 4 butons headset detection
	 * KEY_MEDIA
	 * KEY_VOICECOMMAND
	 * KEY_VOLUMEUP
	 * KEY_VOLUMEDOWN
	 */
	jack_type = SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
		    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
		    SND_JACK_BTN_2 | SND_JACK_BTN_3;

	ret = snd_soc_card_jack_new_pins(card, "Headset Jack",
					 jack_type, jack,
					 cht_rt5677_jack_pins,
					 ARRAY_SIZE(cht_rt5677_jack_pins));
	if (ret) {
		dev_err(card->dev, "Headset Jack creation failed %d\n", ret);
		return ret;
	}

	return ts3a227e_enable_jack_detect(component, jack);
}

static int cht_rt5677_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static const struct snd_soc_ops cht_rt5677_aif1_ops = {
	.startup = cht_rt5677_aif1_startup,
};

static const struct snd_soc_ops cht_rt5677_be_ssp2_ops = {
	.hw_params = cht_rt5677_aif1_hw_params,
};

static struct snd_soc_aux_dev cht_rt5677_headset_dev = {
	.dlc = COMP_AUX("i2c-ts3a227e"),
	.init = cht_rt5677_headset_init,
};

SND_SOC_DAILINK_DEF(dummy,
		    DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(media,
		    DAILINK_COMP_ARRAY(COMP_CPU("media-cpu-dai")));

SND_SOC_DAILINK_DEF(deepbuffer,
		    DAILINK_COMP_ARRAY(COMP_CPU("deepbuffer-cpu-dai")));

SND_SOC_DAILINK_DEF(ssp2_port,
		    DAILINK_COMP_ARRAY(COMP_CPU("ssp2-port")));
SND_SOC_DAILINK_DEF(ssp2_codec,
		    DAILINK_COMP_ARRAY(COMP_CODEC(RT5677_I2C_DEFAULT,
						  "rt5677-aif1")));

SND_SOC_DAILINK_DEF(platform,
		    DAILINK_COMP_ARRAY(COMP_PLATFORM("sst-mfld-platform")));

static struct snd_soc_dai_link cht_rt5677_dailink[] = {
	/* Front End DAI links */
	[MERR_DPCM_AUDIO] = {
		.name = "Audio Port",
		.stream_name = "Audio",
		.nonatomic = true,
		.dynamic = 1,
		.ops = &cht_rt5677_aif1_ops,
		SND_SOC_DAILINK_REG(media, dummy, platform),
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deep-Buffer Audio Port",
		.stream_name = "Deep-Buffer Audio",
		.nonatomic = true,
		.dynamic = 1,
		.playback_only = 1,
		.ops = &cht_rt5677_aif1_ops,
		SND_SOC_DAILINK_REG(deepbuffer, dummy, platform),
	},

	/* Back End DAI links */
	{
		/* SSP2 - Codec */
		.name = "SSP2-Codec",
		.id = 0,
		.no_pcm = 1,
		.nonatomic = true,
		.init = cht_rt5677_codec_init,
		.be_hw_params_fixup = cht_rt5677_codec_fixup,
		.ops = &cht_rt5677_be_ssp2_ops,
		SND_SOC_DAILINK_REG(ssp2_port, ssp2_codec, platform),
	},
};

static const struct acpi_gpio_params speaker_enable_gpio = { 2, 0, false };
static const struct acpi_gpio_mapping cht_rt5677_gpios[] = {
	{ "speaker-enable-gpios", &speaker_enable_gpio, 1 },
	{ NULL }
};

static int snd_cht_rt5677_probe(struct platform_device *pdev)
{
	struct snd_soc_acpi_mach *mach = dev_get_platdata(&pdev->dev);
	struct acpi_device *adev;
	struct cht_rt5677_private *ctx;
	struct snd_soc_card *card;
	const char *platform_name;
	int ret = 0;
	int i;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	card = &ctx->card;
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	card->name = "cht-rt5677";
	card->dai_link = cht_rt5677_dailink,
	card->num_links = ARRAY_SIZE(cht_rt5677_dailink),
	card->aux_dev = &cht_rt5677_headset_dev,
	card->num_aux_devs = 1,
	card->dapm_widgets = cht_rt5677_dapm_widgets,
	card->num_dapm_widgets = ARRAY_SIZE(cht_rt5677_dapm_widgets),
	card->dapm_routes = cht_rt5677_audio_map,
	card->num_dapm_routes = ARRAY_SIZE(cht_rt5677_audio_map),
	card->controls = cht_rt5677_controls,
	card->num_controls = ARRAY_SIZE(cht_rt5677_controls),

	strscpy(ctx->codec_name, RT5677_I2C_DEFAULT);

	/* fixup codec name based on HID if ACPI node is present */
	adev = acpi_dev_get_first_match_dev(mach->id, NULL, -1);
	if (adev) {
		snprintf(ctx->codec_name, sizeof(ctx->codec_name),
			 "i2c-%s", acpi_dev_name(adev));

		acpi_dev_put(adev);
		for (i = 0; i < ARRAY_SIZE(cht_rt5677_dailink); i++) {
			if (cht_rt5677_dailink[i].codecs->name &&
			    !strcmp(cht_rt5677_dailink[i].codecs->name,
				    RT5677_I2C_DEFAULT)) {
				cht_rt5677_dailink[i].codecs->name = ctx->codec_name;
				break;
			}
		}
	}

	struct device *codec_dev __free(put_device) =
		bus_find_device_by_name(&i2c_bus_type, NULL, ctx->codec_name);

	if (!codec_dev)
		return -EPROBE_DEFER;

	if (adev) {
		ret = devm_acpi_dev_add_driver_gpios(codec_dev,
							 cht_rt5677_gpios);
		if (ret)
			dev_warn(&pdev->dev,
				 "Unable to add GPIO mapping table: %d\n",
				 ret);
	}

	ctx->gpio_spk_en1 = gpiod_get(codec_dev, "speaker-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_spk_en1)) {
		ret = PTR_ERR(ctx->gpio_spk_en1);
		return dev_err_probe(&pdev->dev, ret, "getting speaker enable GPIO\n");
	}

	ctx->gpio_spk_en2 = gpiod_get(codec_dev, "speaker-enable2", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_spk_en2)) {
		ret = PTR_ERR(ctx->gpio_spk_en2);
		dev_err_probe(codec_dev, ret, "getting speaker enable 2 GPIO\n");
		goto out_put_spken_gpio;
	}

	ctx->gpio_hp_en = gpiod_get(codec_dev, "headphone-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_hp_en)) {
		ret = PTR_ERR(ctx->gpio_hp_en);
		dev_err_probe(codec_dev, ret, "getting headphone enable GPIO\n");
		goto out_put_spken2_gpio;
	}

	/* override platform name, if required */
	platform_name = mach->mach_params.platform;

	ret = snd_soc_fixup_dai_links_platform_name(card, platform_name);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "fixing up dai links platform name\n");
		goto out_put_hpen_gpio;
	}

	ctx->mclk = devm_clk_get(&pdev->dev, "pmc_plt_clk_3");
	if (IS_ERR(ctx->mclk)) {
		ret = PTR_ERR(ctx->mclk);
		dev_err_probe(&pdev->dev, ret, "getting MCLK from pmc_plt_clk_3\n");
		goto out_put_hpen_gpio;
	}

	snd_soc_card_set_drvdata(card, ctx);

	/* register the soc card */
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "registering card\n");
		goto out_put_hpen_gpio;
	}

	return 0;

out_put_hpen_gpio:
	gpiod_put(ctx->gpio_hp_en);
out_put_spken2_gpio:
	gpiod_put(ctx->gpio_spk_en2);
out_put_spken_gpio:
	gpiod_put(ctx->gpio_spk_en1);

	return ret;
}

static void snd_cht_rt5677_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct cht_rt5677_private *ctx = snd_soc_card_get_drvdata(card);
	int i;

	/*
	 * Reset the codec name in the global dailink array back to the default
	 * to avoid a use-after-free on driver rebind (unbind/bind):
	 * ctx->codec_name is about to be freed together with ctx (devm), but
	 * cht_rt5677_dailink[] is a static global that persists across binds.
	 */
	for (i = 0; i < ARRAY_SIZE(cht_rt5677_dailink); i++) {
		if (cht_rt5677_dailink[i].codecs->name == ctx->codec_name) {
			cht_rt5677_dailink[i].codecs->name = RT5677_I2C_DEFAULT;
			break;
		}
	}

	gpiod_put(ctx->gpio_hp_en);
	gpiod_put(ctx->gpio_spk_en2);
	gpiod_put(ctx->gpio_spk_en1);
}

static struct platform_driver snd_cht_rt5677_driver = {
	.driver = {
		.name = "cht-rt5677",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_cht_rt5677_probe,
	.remove = snd_cht_rt5677_remove,
};

module_platform_driver(snd_cht_rt5677_driver);

MODULE_DESCRIPTION("ASoC Intel Cherrytrail with RT5677 machine driver");
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht-rt5677");
