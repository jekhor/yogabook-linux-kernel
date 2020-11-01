/*
 *  cht_bsw_rt5677.c - ASoc Machine driver for Intel Cherryview-based platforms
 *                     Cherrytrail and Braswell, with RT5677 codec.
 *
 *  Copyright (C) 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 *  Based on cht_bsw_rt5672.c:
 *  Copyright (C) 2014 Intel Corp
 *  Author: Subhransu S. Prusty <subhransu.s.prusty@intel.com>
 *          Mengdong Lin <mengdong.lin@intel.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#define DEBUG

#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/dmi.h>
#include <linux/spi/spi.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-acpi.h>
#include "../../codecs/rt5677.h"
#include "../../codecs/ts3a227e.h"
#include "../atom/sst-atom-controls.h"


/* The platform clock #3 outputs 19.2Mhz clock to codec as I2S MCLK */
#define CHT_PLAT_CLK_3_HZ	19200000
#define CHT_CODEC_DAI	"rt5677-aif1"

struct cht_mc_private {
	char codec_name[SND_ACPI_I2C_ID_LEN];
	struct snd_soc_jack jack;
	struct clk *mclk;
	struct gpio_desc *gpio_spk_en1;
	struct gpio_desc *gpio_spk_en2;
	struct gpio_desc *gpio_hp_en;
};

static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret;

	dev_dbg(card->dev, "Setting platform clock\n");

	codec_dai = snd_soc_card_get_codec_dai(card, CHT_CODEC_DAI);
	if (!codec_dai) {
		dev_err(card->dev, "Codec dai not found; Unable to set platform clock\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (ctx->mclk) {
			ret = clk_prepare_enable(ctx->mclk);
			if (ret < 0) {
				dev_err(card->dev,
					"could not configure MCLK state");
				return ret;
			}
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
	} else {
		/* Set codec sysclk source to its internal clock because codec
		 * PLL will be off when idle and MCLK will also be off by ACPI
		 * when codec is runtime suspended. Codec needs clock for jack
		 * detection and button press.
		 */
		snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_RCCLK,
				       48000 * 512, SND_SOC_CLOCK_IN);

		if (ctx->mclk)
			clk_disable_unprepare(ctx->mclk);
	}
	return 0;
}

static int cht_yb_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);

	dev_dbg(card->dev, "HP event: %s\n",
		SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		msleep(20);
		gpiod_set_value_cansleep(ctx->gpio_hp_en, 1);
		msleep(50);
	} else {
		gpiod_set_value_cansleep(ctx->gpio_hp_en, 0);
	}

	return 0;
}

static int cht_yb_spk_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);

	dev_dbg(card->dev, "SPK event: %s\n",
		SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");

	/* Black magic from android kernel */
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpiod_set_value_cansleep(ctx->gpio_spk_en1, 1);
		udelay(2);
		gpiod_set_value_cansleep(ctx->gpio_spk_en1, 0);
		udelay(2);
		gpiod_set_value_cansleep(ctx->gpio_spk_en1, 1);
		udelay(2);
		gpiod_set_value_cansleep(ctx->gpio_spk_en1, 0);
		udelay(2);
	}

	gpiod_set_value_cansleep(ctx->gpio_spk_en1,
			SND_SOC_DAPM_EVENT_ON(event));
	gpiod_set_value_cansleep(ctx->gpio_spk_en2,
			SND_SOC_DAPM_EVENT_ON(event));
	msleep(50);

	return 0;
}


static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", cht_yb_hp_event),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", cht_yb_spk_event),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_audio_map[] = {
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

//	{"ssp0 Tx", NULL, "modem_out"},
//	{"modem_in", NULL, "ssp0 Rx" },

//	{ "ssp1 Tx", NULL, "bt_fm_out"},
//	{ "bt_fm_in", NULL, "ssp1 Rx" },

	{"Headphone", NULL, "Platform Clock"},
	{"Speaker", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
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
	/*
	 * Default mode for SSP configuration is TDM 4 slot
	 */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBS_CFS);
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

static struct gpiod_lookup_table cht_yb_gpios_table = {
	/* .dev_id is set during probe */
	.table = {
		GPIO_LOOKUP("rt5677", 2, "speaker-enable2", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("rt5677", 4, "headphone-enable", GPIO_ACTIVE_HIGH),
		{ },
	},
};

#if 0
static int cht_yb_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		card->dapm.bias_level = level;
		pr_debug("card(%s)->bias_level %u\n", card->name,
				card->dapm.bias_level);
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		ret =  -EINVAL;
	}

	return ret;
}
#endif

static int cht_yb_jack_event(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct snd_soc_jack *jack = (struct snd_soc_jack *)data;
	struct snd_soc_dapm_context *dapm = &jack->card->dapm;

	if (event & SND_JACK_MICROPHONE) {
		snd_soc_dapm_force_enable_pin(dapm, "MICBIAS1");
		snd_soc_dapm_sync(dapm);
	} else {
		snd_soc_dapm_disable_pin(dapm, "MICBIAS1");
		snd_soc_dapm_sync(dapm);
	}

	return 0;
}

static struct notifier_block cht_yb_jack_nb = {
	.notifier_call = cht_yb_jack_event,
};

static int cht_codec_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(runtime, 0);
	struct snd_soc_component *component = codec_dai->component;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_jack *jack = &ctx->jack;

	printk(KERN_INFO "%s\n", __func__);

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

	cht_yb_gpios_table.dev_id = dev_name(component->dev);
	gpiod_add_lookup_table(&cht_yb_gpios_table);

	ctx->gpio_spk_en1 = devm_gpiod_get(component->dev, "speaker-enable",
					  GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_spk_en1)) {
		dev_err(component->dev, "Can't find speaker enable GPIO\n");
		return PTR_ERR(ctx->gpio_spk_en1);
	}

	ctx->gpio_spk_en2 = devm_gpiod_get(component->dev, "speaker-enable2",
					  GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_spk_en2)) {
		dev_err(component->dev, "Can't find speaker enable 2 GPIO\n");
		return PTR_ERR(ctx->gpio_spk_en2);
	}

	ctx->gpio_hp_en = devm_gpiod_get(component->dev, "headphone-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_hp_en)) {
		dev_err(component->dev, "Can't find headphone enable GPIO\n");
		return PTR_ERR(ctx->gpio_hp_en);
	}


	snd_soc_jack_notifier_register(jack, &cht_yb_jack_nb);

	if (ctx->mclk) {
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
	}

	return 0;
}

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	dev_dbg(rtd->dev, "cht_codec_fixup()\n");

	/* The DSP will convert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/*
	 * set SSP2 to 24-bit
	 * Looks like strange black magic because ssp2-port supports S16LE
	 * format only, taken from Intel's code
	 */
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static struct snd_soc_jack_pin cht_yb_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Speaker",
		.mask = SND_JACK_HEADPHONE,
		.invert = true,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Int Mic",
		.mask = SND_JACK_MICROPHONE,
		.invert = true,
	},
};

static int cht_yb_headset_init(struct snd_soc_component *component)
{
	struct snd_soc_card *card = component->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack *jack = &ctx->jack;
	int jack_type;
	int ret;

	/*
	 * TI supports 4 butons headset detection
	 * KEY_MEDIA
	 * KEY_VOICECOMMAND
	 * KEY_VOLUMEUP
	 * KEY_VOLUMEDOWN
	 */
	jack_type = SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
		    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
		    SND_JACK_BTN_2 | SND_JACK_BTN_3;

	ret = snd_soc_card_jack_new(card, "Headset Jack", jack_type,
				    jack, cht_yb_jack_pins, ARRAY_SIZE(cht_yb_jack_pins));
	if (ret) {
		dev_err(card->dev, "Headset Jack creation failed %d\n", ret);
		return ret;
	}

	return ts3a227e_enable_jack_detect(component, jack);
}

static int cht_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static const struct snd_soc_ops cht_aif1_ops = {
	.startup = cht_aif1_startup,
};

static const struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_aux_dev cht_yb_headset_dev = {
	.dlc = COMP_AUX("i2c-ts3a227e.0"),
	.init = cht_yb_headset_init,
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
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-10EC5677:00", "rt5677-aif1")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("sst-mfld-platform")));

static struct snd_soc_dai_link cht_dailink[] = {
	/* Front End DAI links */
	[MERR_DPCM_AUDIO] = {
		.name = "Audio Port",
		.stream_name = "Audio",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_aif1_ops,
		SND_SOC_DAILINK_REG(media, dummy, platform),
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deep-Buffer Audio Port",
		.stream_name = "Deep-Buffer Audio",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &cht_aif1_ops,
		SND_SOC_DAILINK_REG(deepbuffer, dummy, platform),
	},

	/* Back End DAI links */
	{
		/* SSP2 - Codec */
		.name = "SSP2-Codec",
		.id = 0,
		.no_pcm = 1,
		.nonatomic = true,
		.init = cht_codec_init,
		.be_hw_params_fixup = cht_codec_fixup,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_be_ssp2_ops,
		SND_SOC_DAILINK_REG(ssp2_port, ssp2_codec, platform),
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_cht = {
	.name = "cht-yogabook",
	.driver_name = "cht-yogabook",
	.owner = THIS_MODULE,
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.aux_dev = &cht_yb_headset_dev,
	.num_aux_devs = 1,
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static void cht_codec_register_spidev(void)
{
	struct spi_board_info rt5677_board_info;
	int ret;

	memset(&rt5677_board_info, 0, sizeof(rt5677_board_info));
	strncpy(rt5677_board_info.modalias, "rt5677", sizeof(rt5677_board_info.modalias));
	rt5677_board_info.irq = 0;
	rt5677_board_info.bus_num = 1;
	rt5677_board_info.chip_select = 0;
	rt5677_board_info.max_speed_hz = 5000000;
	ret = spi_register_board_info(&rt5677_board_info, 1);
}

#define RT5677_I2C_DEFAULT	"i2c-10EC5677:00"

static const struct acpi_gpio_params speaker_enable_gpio = { 2, 0, false };
static const struct acpi_gpio_mapping cht_yb_gpios[] = {
	{ "speaker-enable-gpios", &speaker_enable_gpio, 1 },
	{ NULL }
};

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct cht_mc_private *drv;
	struct snd_soc_acpi_mach *mach = pdev->dev.platform_data;
	const char *platform_name;
	struct acpi_device *adev;
	struct device *codec_dev;
	int i;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv)
		return -ENOMEM;

	strcpy(drv->codec_name, RT5677_I2C_DEFAULT);

	/* fixup codec name based on HID */
	adev = acpi_dev_get_first_match_dev(mach->id, NULL, -1);
	if (adev) {
		snprintf(drv->codec_name, sizeof(drv->codec_name),
			 "i2c-%s", acpi_dev_name(adev));
		dev_info(&pdev->dev, "real codec name: %s\n", drv->codec_name);

		put_device(&adev->dev);
		for (i = 0; i < ARRAY_SIZE(cht_dailink); i++) {
			if (!strcmp(cht_dailink[i].codecs->name,
				    RT5677_I2C_DEFAULT)) {
				cht_dailink[i].codecs->name = drv->codec_name;
				break;
			}
		}
	}

	codec_dev = bus_find_device_by_name(&i2c_bus_type, NULL,
			drv->codec_name);
	if (!codec_dev)
		return -EPROBE_DEFER;

	dev_info(&pdev->dev, "Is ACPI devnode: %d\n", is_acpi_device_node(codec_dev->fwnode));
	dev_info(&pdev->dev, "ACPI handle: %p\n", ACPI_HANDLE(codec_dev));

	ret_val = devm_acpi_dev_add_driver_gpios(codec_dev, cht_yb_gpios);
	if(ret_val)
		dev_warn(&pdev->dev, "Unable to add GPIO mapping table: %d\n",
			 ret_val);

	/* override plaform name, if required */
	snd_soc_card_cht.dev = &pdev->dev;
	platform_name = mach->mach_params.platform;

	ret_val = snd_soc_fixup_dai_links_platform_name(&snd_soc_card_cht,
							platform_name);
	if (ret_val) {
		dev_err(&pdev->dev, "snd_soc_fixup_dai_links_platform_name failed: %d\n",
				ret_val);
		return ret_val;
	}

	drv->mclk = devm_clk_get(&pdev->dev, "pmc_plt_clk_3");
	if (IS_ERR(drv->mclk)) {
		dev_err(&pdev->dev,
			"Failed to get MCLK from pmc_plt_clk_3: %ld\n",
			PTR_ERR(drv->mclk));
		return PTR_ERR(drv->mclk);
	}
	snd_soc_card_set_drvdata(&snd_soc_card_cht, drv);

	/* register the soc card */
	snd_soc_card_cht.dev = &pdev->dev;
	ret_val = devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_cht);
	if (ret_val) {
		dev_err(&pdev->dev,
			"snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_cht);

	cht_codec_register_spidev();

	return ret_val;
}

static struct platform_driver snd_cht_mc_driver = {
	.driver = {
		.name = "cht-yogabook",
	},
	.probe = snd_cht_mc_probe,
};

static const struct dmi_system_id yb_dmi_device_table[] = {
	{
		.ident = "Lenovo YogaBook",
		/* YB1-X91L/F and YB1-X90L/F */
		.matches = {
			DMI_MATCH(DMI_PRODUCT_NAME, "Lenovo YB1-X9")
		}
	},
	{}
};

static struct i2c_client *ts3a227e_client;

static const struct property_entry ts3a227e_props[] = {
	/* Got from Lenovo Android kernel code drop */
	PROPERTY_ENTRY_U32("ti,micbias", 7),
	{}
};

static const struct software_node ts3a227e_node = {
	.properties = ts3a227e_props,
};

static int cht_yb_register_ts3a227e(void)
{
	struct device *codec_dev;
	struct acpi_device *adev;
	struct i2c_board_info board_info = {
		.type = "ts3a227e",
		.dev_name = "ts3a227e.0",
		.swnode = &ts3a227e_node,
	};
	int ret;

	codec_dev = bus_find_device_by_name(&i2c_bus_type, NULL,
			RT5677_I2C_DEFAULT);
	if (!codec_dev) {
		pr_err("Failed to find codec device to get ts3a227e settings\n");
		return -ENODEV;
	}

	adev = ACPI_COMPANION(codec_dev);

	ret = acpi_dev_gpio_irq_get(adev, 1);
	if (ret < 0) {
		pr_err("cht_yogabook: Error requesting irq at index 1 for ts3a227e: %d\n",
			ret);
		goto error;
	}

	board_info.irq = ret;
	pr_debug("IRQ = %d\n", board_info.irq);

	ts3a227e_client = i2c_acpi_new_device(codec_dev, 1, &board_info);
	if (IS_ERR(ts3a227e_client)) {
		ret = PTR_ERR(ts3a227e_client);
		if (ret != -EPROBE_DEFER)
			pr_err("Error creating i2c client for ts3a227e: %d\n", ret);
		goto error;
	}

	return 0;

error:
	return ret;
}

static int __init cht_yb_init(void)
{
	int ret;

	if (!dmi_check_system(yb_dmi_device_table)) {
		return -ENODEV;
	}

	ret = cht_yb_register_ts3a227e();
	if (ret)
		return ret;

	return platform_driver_register(&snd_cht_mc_driver);
}
module_init(cht_yb_init);

static void __exit cht_yb_exit(void)
{
	platform_driver_unregister(&snd_cht_mc_driver);
	i2c_unregister_device(ts3a227e_client);
}
module_exit(cht_yb_exit);

MODULE_DESCRIPTION("Lenovo Yoga Book X1-9x machine driver");
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht-yogabook");
