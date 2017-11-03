/*
 * Asahi Kaesi AK4621 Alsa SoC driver
 *
 * Copyright (C) 2016-2017, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/soc.h>

/*
 * Register addresses
 */
#define REG_POWER_DOWN		(0X0)
#define REG_RESET		(0X1)
#define REG_CLOCK_AND_FORMAT	(0X2)
#define REG_DEEM_AND_VOLUME	(0X3)
#define REG_RESERVED_1		(0x4)
#define REG_RESERVED_2		(0x5)
#define REG_LCH_DAAT		(0X6)
#define REG_RCH_DAAT		(0X7)
#define REG_LCH_EXTENSION_DAAT	(0X8)
#define REG_RCH_EXTENSION_DAAT	(0X9)
#define REG_LASTREG		REG_RCH_EXTENSION_DAAT

//TODO: if this isn't needed remove it!
static const struct reg_default ak4621_reg_defaults[] = {
	{ REG_POWER_DOWN,		0x07 },
	{ REG_RESET,			0x00 },
	{ REG_CLOCK_AND_FORMAT,		0x40 },
	{ REG_DEEM_AND_VOLUME,		0x01 },
	{ REG_LCH_DAAT,			0xFF },
	{ REG_RCH_DAAT,			0xFF },
	{ REG_LCH_EXTENSION_DAAT,	0x0F },
	{ REG_RCH_EXTENSION_DAAT,	0x0F },
};

struct ak4621_private {
	struct regmap *regmap;
	int gpio_pdn;

};

static int ak4621_probe(struct snd_soc_codec *codec)
{
	struct ak4621_private *ak4621 = snd_soc_codec_get_drvdata(codec);

	codec->control_data = ak4621->regmap;

	/*
	 * Hard code for 48k operation
	 *   Clock mode = Normal speed
	 *   Master clock freq = 512fs
	 *   Audio data mode = Mode 3 (24bit I2S in and out, LRCK = L/H)
	 */
	regmap_write(ak4621->regmap, REG_CLOCK_AND_FORMAT, 0x64);
	/* release DAC and ADC from reset */
	regmap_write(ak4621->regmap, REG_RESET, 0x03);

	return 0;
}

static int ak4621_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static const struct snd_kcontrol_new ak4621_snd_controls[] = {
	SOC_DOUBLE_R("PCM Playback Volume", 0x6, 0x7, 0, 0xFF, 0),
};

static const struct snd_soc_dai_ops ak4621_dai_ops = {
	//.hw_params = ak4621_pcm_hw_params,
};

static struct snd_soc_dai_driver ak4621_dai = {
	.name = "ak4621",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &ak4621_dai_ops,
	.symmetric_rates = 1,
	.symmetric_channels = 1,
	.symmetric_samplebits = 1,
};

static struct snd_soc_codec_driver ak4621_driver = {
	.probe = ak4621_probe,
	.remove = ak4621_remove,
	//TODO: need .suspend and .resume?
	.controls = ak4621_snd_controls,
	.num_controls = ARRAY_SIZE(ak4621_snd_controls),
	//TODO: figure out what DPAM routes and widges are and do we need them?
};

static const struct regmap_range ak4621_rw_yes =
	regmap_reg_range(0, REG_LASTREG);
static const struct regmap_range ak4621_rw_no =
	regmap_reg_range(REG_RESERVED_1, REG_RESERVED_2);
static const struct regmap_access_table ak4621_rw_range = {
	.yes_ranges = &ak4621_rw_yes,
	.n_yes_ranges = 1,
	.no_ranges = &ak4621_rw_no,
	.n_no_ranges = 1,
};

static const struct regmap_config ak4621_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_LASTREG,
	.write_flag_mask = 0xA0,
	.cache_type = REGCACHE_FLAT,
	.wr_table = &ak4621_rw_range,
	.rd_table = &ak4621_rw_range,
	.reg_defaults = ak4621_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4621_reg_defaults),
};

static int ak4621_spi_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct ak4621_private *ak4621;
	int ret;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ak4621 = devm_kzalloc(&spi->dev, sizeof(*ak4621),
			      GFP_KERNEL);
	if (ak4621 == NULL)
		return -ENOMEM;

	ak4621->regmap = devm_regmap_init_spi(spi, &ak4621_regmap);
	if (IS_ERR(ak4621->regmap)) {
		ret = PTR_ERR(ak4621->regmap);
		return ret;
	}

	ak4621->gpio_pdn = of_get_named_gpio(np, "gpio-pdn", 0);
	if (gpio_is_valid(ak4621->gpio_pdn)) {
		ret = devm_gpio_request_one(&spi->dev, ak4621->gpio_pdn,
					    GPIOF_OUT_INIT_HIGH, "AK4621 PDN");
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to request PDN gpio: %d\n", ret);
			return ret;
		}

		/* 
		 * toggle PDN line to reset chip per datasheet.
		 * Datasheet requires only 150ns, but stretch it longer in case
		 * hardware is adding softening on the edges.
		 */
		gpio_set_value(ak4621->gpio_pdn, 0);
		usleep_range(100, 1000);
		gpio_set_value(ak4621->gpio_pdn, 1);
	} else {
		dev_err(&spi->dev, "PDN gpio not specified or is invalid.");
		return -EINVAL;
	}

	/*
	 *TODO: Find if there is a way to verify the presence of the AK4621
	 *return -ENODEV if we can't find the AK4621
	 */

	spi_set_drvdata(spi, ak4621);

	ret = snd_soc_register_codec(&spi->dev, &ak4621_driver, &ak4621_dai, 1);
	if (ret) {
		dev_err(&spi->dev, "Couldn't register codec with ASoC\n");
		return ret;
	}

	return 0;
}

static int ak4621_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	return 0;
}

static const struct of_device_id ak4621_of_match[] = {
	{ .compatible = "asahi-kasei,ak4621", },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4621_of_match);

static const struct spi_device_id ak4621_id_table[] = {
	{ "ak4621", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ak4621_id_table);

static struct spi_driver ak4621_spi_driver = {
	.driver  = {
		.name   = "ak4621",
		.owner  = THIS_MODULE,
		.of_match_table = ak4621_of_match,
	},
	.id_table = ak4621_id_table,
	.probe  = ak4621_spi_probe,
	.remove = ak4621_spi_remove,
};

module_spi_driver(ak4621_spi_driver);

MODULE_AUTHOR("Matthew Campbell <mcampbell@izotope.com>");
MODULE_DESCRIPTION("Asahi Kasei AK4621 ALSA SoC driver");
MODULE_LICENSE("GPL v2");
