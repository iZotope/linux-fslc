/*
 * That Corp that5173 Alsa SoC driver
 *
 * Copyright (C) 2016-2017, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *          Jonah Petri <jpetri@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "that5173.h"

// TODO: expose GPOs to the GPIO subsystem
// TODO: expose the zero crossing update as an Alsa switch control

#define MAGIC_WORD (0xBEEF)
#define MAX_DAISY_CNT (255)
// TODO? support configurations where DOUT is not connected back to the micro

#define THAT5173_GPO_FIELD_MASK  (0xF)
#define THAT5173_GAINMODE  (1 << 4)
#define THAT5173_GPOMODE   (1 << 5)
#define THAT5173_GAIN_MASK (0x1F00)
#define THAT5173_GAIN(X)   (((X) << 8) & THAT5173_GAIN_MASK)

#define THAT5173_NUM_GPO   (4) /* number of GPOs per a chip */

struct that5173_state {
	/* general purpose outputs */
	int gpo; /* bitfield of GPO value bit0 = gpo0, bit1 = gpo1, etc */
	/*
	 * update modes for gpo and gain
	 * 0 = immediate update
	 * 1 = update on zero crossing
	 */
	int gainmode : 1;
	int gpomode  : 1;
	int gain; /* state of gain register, not db gain */
};

struct that5173_private {
	struct mutex mutex;
	struct spi_device *spi;
	int daisy_cnt; /* count of 5173s in daisy chain, 1 for no daisy chain */
	struct that5173_state *state;
	u16 *tx;
	u16 *rx;
	int gpio_rst;
	struct gpio_chip gpio_chip;
};

static void state_to_reg(struct that5173_state *const st, u16 *reg)
{
	*reg = st->gpo & THAT5173_GPO_FIELD_MASK;

	if (st->gainmode)
		*reg |= THAT5173_GAINMODE;
	if (st->gpomode)
		*reg |= THAT5173_GPOMODE;
	*reg |= THAT5173_GAIN(st->gain);
}

/*
 * This call is not synchronized. You must wrap it in mutexes where you call it.
 * This allows you to update the priv state and flush to hardware safely.
 * It will post a warning if called wihout prv->mutex locked.
 */
static int xfer_state(struct spi_device *spi)
{
	struct that5173_private *prv = spi_get_drvdata(spi);
	int ret;
	int i;
	u16 *preg;
	struct spi_transfer xfers[] = {
		{
			.len = (prv->daisy_cnt + 1) * sizeof(*prv->tx),
			.tx_buf = prv->tx,
			.rx_buf = prv->rx,
		},
	};

	ret = mutex_is_locked(&prv->mutex);
	if (!ret) {
		WARN_ON(1);
		dev_err(&spi->dev, "BUG: xfer_state called without mutex locked\n");
		return -EPERM;
	}

	prv->tx[0] = MAGIC_WORD;
	/* invert ordering of regs so first chip (closet to the CPU)
		in daisy is state[0] */
	preg = &prv->tx[prv->daisy_cnt];
	for (i = 0; i < prv->daisy_cnt; i++) {
		state_to_reg(&prv->state[i], preg--);
	};

	ret = spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	if (prv->rx[prv->daisy_cnt] != MAGIC_WORD) {
		dev_err(&spi->dev, "IO error: no response on set state\n");
		return -EIO;
	}

	return 0;
}

int detect_daisy_cnt(struct spi_device *spi)
{
	struct that5173_private *prv = spi_get_drvdata(spi);
	int ret;
	int i;
	u16 tx = 0;
	u16 rx;
	struct spi_transfer xfers[] = {
		{
			.len = sizeof(tx),
			.tx_buf = &tx,
			.rx_buf = &rx,
			.cs_change = 1,
		},
	};

	mutex_lock(&prv->mutex);
	tx = MAGIC_WORD;
	for (i = 0; i < MAX_DAISY_CNT; i++) {
		ret = spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
		if (ret)
			return ret;
		if (rx == MAGIC_WORD)
			break;
		// only send MAGIC_WORD in the first transaction
		tx = 0;
	}
	mutex_unlock(&prv->mutex);

	if (i == MAX_DAISY_CNT) {
		dev_err(&spi->dev, "Couldn't detect THAT5173 chip(s)\n");
		return -ENODEV;
	}

	prv->daisy_cnt = i;
	dev_info(&spi->dev, "Found %d chips in a daisy chain\n", prv->daisy_cnt);
	return 0;
}

static int that5173_probe(struct snd_soc_codec *codec)
{
	struct that5173_private *prv = snd_soc_codec_get_drvdata(codec);
	int ret;

	dev_info(codec->dev, "****************************Probing the that5173 into ASoC\n");

	mutex_lock(&prv->mutex);
	memset(prv->state, 0, sizeof(*prv->state) * prv->daisy_cnt);
	ret = xfer_state(prv->spi);
	mutex_unlock(&prv->mutex);

	if (ret) {
		dev_err(codec->dev, "Couldn't init to default state\n");
		return ret;
	}

	return 0;
}

static int that5173_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "Removing the that5173 from ASoC\n");
	return 0;
}

/* custom fuctions for info/put/get of preamp gain */
static int preamp_info_gain(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct that5173_private *prv = snd_soc_codec_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = prv->daisy_cnt;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 20;
	return 0;
}

static int preamp_get_gain(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct that5173_private *prv = snd_soc_codec_get_drvdata(codec);
	int i;

	mutex_lock(&prv->mutex);
	for (i = 0; i < prv->daisy_cnt; i++) {
		ucontrol->value.integer.value[i] = prv->state[i].gain;
	}
	mutex_unlock(&prv->mutex);

	return 0;
}

static int preamp_put_gain(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct that5173_private *prv = snd_soc_codec_get_drvdata(codec);
	int i;

	mutex_lock(&prv->mutex);
	for (i = 0; i < prv->daisy_cnt; i++) {
		prv->state[i].gain = ucontrol->value.integer.value[i];
	}
	xfer_state(prv->spi);
	mutex_unlock(&prv->mutex);

	return 0;
}

static DECLARE_TLV_DB_SCALE(gain_tlv, 0, 300, 0);

static const struct snd_kcontrol_new that5173_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Line Capture Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.tlv.p = gain_tlv,
		.info = preamp_info_gain,
		.get = preamp_get_gain,
		.put = preamp_put_gain,
	},
};

static struct snd_soc_codec_driver that5173_driver = {
	.probe = that5173_probe,
	.remove = that5173_remove,
	.controls = that5173_snd_controls,
	.num_controls = ARRAY_SIZE(that5173_snd_controls),
};


int that5173_set_gpo_values(struct snd_soc_codec *codec, u8 *gpo_buf, unsigned gpo_buf_len)
{
	struct that5173_private *prv = snd_soc_codec_get_drvdata(codec);
	int i;
	int gpo_ndx;
	int gpo_val;
	int result = 0;

	mutex_lock(&prv->mutex);
	for (i = 0; i < gpo_buf_len && i < prv->daisy_cnt; i++) {
		for (gpo_ndx = 0; gpo_ndx < THAT5173_NUM_GPO; gpo_ndx++) {
			gpo_val = gpo_buf[i] & THAT5173_GPO_MASK(gpo_ndx);

			if (gpo_val == THAT5173_SET_GPO(gpo_ndx))
				prv->state[i].gpo |= (1 << gpo_ndx);
			else if (gpo_val == THAT5173_CLEAR_GPO(gpo_ndx))
				prv->state[i].gpo &= ~(1 << gpo_ndx);
		}
	}
	result = xfer_state(prv->spi);
	mutex_unlock(&prv->mutex);
	return result;
}

EXPORT_SYMBOL_GPL(that5173_set_gpo_values);

static int that5173_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	/* todo: should this fail, not sure how gpiolib feels about output only*/
	return 0;
}

static int that5173_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	/* output only, always return success */
	return 0;
}

static int that5173_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct that5173_private *prv;
	int chip_ndx;
	int gpo_ndx;
	int ret;

	prv = container_of(gc, struct that5173_private, gpio_chip);

	mutex_lock(&prv->mutex);
	chip_ndx = off / THAT5173_NUM_GPO;
	gpo_ndx = off % THAT5173_NUM_GPO;
	if (prv->state[chip_ndx].gpo & (1 << gpo_ndx)) {
		ret = 1;
	} else {
		ret = 0;
	}
	mutex_unlock(&prv->mutex);

	return ret;
}

static void that5173_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct that5173_private *prv;
	int chip_ndx;
	int gpo_ndx;

	prv = container_of(gc, struct that5173_private, gpio_chip);

	mutex_lock(&prv->mutex);
	chip_ndx = off / THAT5173_NUM_GPO;
	gpo_ndx = off % THAT5173_NUM_GPO;
	if (val) {
		prv->state[chip_ndx].gpo |= 1 << gpo_ndx;
	} else {
		prv->state[chip_ndx].gpo &= ~(1 << gpo_ndx);
	}
	xfer_state(prv->spi);
	mutex_unlock(&prv->mutex);
}

static void that5173_setup_gpio(struct that5173_private *prv)
{
	struct gpio_chip *gc;

	gc = &prv->gpio_chip;

	gc->label = dev_name(&prv->spi->dev);

	gc->direction_input  = that5173_gpio_direction_input;
	gc->direction_output = that5173_gpio_direction_output;
	gc->get = that5173_gpio_get_value;
	gc->set = that5173_gpio_set_value;
	gc->can_sleep = true;

	gc->base = -1;
	gc->ngpio = THAT5173_NUM_GPO * prv->daisy_cnt;
	gc->dev = &prv->spi->dev;;
	gc->owner = THIS_MODULE;
}

static int that5173_spi_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct that5173_private *that5173;
	int ret;

	dev_info(&spi->dev, "In the dev probe\n");

	spi->bits_per_word=16;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	that5173 = devm_kzalloc(&spi->dev, sizeof(*that5173), GFP_KERNEL);
	if (that5173 == NULL)
		return -ENOMEM;

	that5173->spi = spi;
	mutex_init(&that5173->mutex);
	spi_set_drvdata(spi, that5173);

	ret = of_property_read_u32(np, "daisy-count", &that5173->daisy_cnt);
	if (ret) {
		detect_daisy_cnt(spi);
	} else {
		dev_info(&spi->dev, "daisy-count set by device tree to %d\n",
					that5173->daisy_cnt);
	}

	that5173->state = devm_kcalloc(&spi->dev, that5173->daisy_cnt,
					sizeof(*that5173->state), GFP_KERNEL);
	if (that5173 == NULL)
		return -ENOMEM;

	/* 1 extra tx word (2 bytes) for magic word */
	that5173->tx = devm_kcalloc(&spi->dev, that5173->daisy_cnt + 1,
					sizeof(*that5173->tx), GFP_KERNEL);
	if (that5173 == NULL)
		return -ENOMEM;

	that5173->rx = devm_kcalloc(&spi->dev, that5173->daisy_cnt + 1,
					sizeof(*that5173->rx), GFP_KERNEL);
	if (that5173 == NULL)
		return -ENOMEM;

	/* setup GPIO reset line */
	that5173->gpio_rst = of_get_named_gpio(np, "gpio-rst", 0);
	if (!gpio_is_valid(that5173->gpio_rst)) {
		dev_warn(&spi->dev, "Couldn't parse 'gpio-rst' property from device tree, will not perform reset.\n");
		that5173->gpio_rst = -1;
	} else {
		ret = gpio_request_one(that5173->gpio_rst, GPIOF_OUT_INIT_HIGH,
						dev_name(&spi->dev));
		if (ret) {
			dev_warn(&spi->dev, "Unable to request 'gpio-rst', %d. Will not perform reset.\n", ret);
			that5173->gpio_rst = -1;
		}
	}

	/* reset the device */
	if (gpio_is_valid(that5173->gpio_rst)) {
		gpio_set_value(that5173->gpio_rst, 0);
		gpio_set_value(that5173->gpio_rst, 1);
	}

	/* Sync the device state */
	mutex_lock(&that5173->mutex);
	ret = xfer_state(spi);
	mutex_unlock(&that5173->mutex);
	if (ret) {
		dev_err(&spi->dev, "THAT5173 chip not found\n");
		ret = -ENODEV;
		goto err_free_gpio;
	}

	ret = snd_soc_register_codec(&spi->dev, &that5173_driver, NULL, 0);
	dev_info(&spi->dev, "Registered the codec with ASoC %d\n", ret);
	dev_info(&spi->dev, "devname '%s'\n", dev_name(&spi->dev));

	that5173_setup_gpio(that5173);
	ret = gpiochip_add(&that5173->gpio_chip);
	if (ret) {
		dev_err(&spi->dev, "Faild to add GPO chip, %d\n", ret);
		goto err_free_gpio;
	}

	return ret;

err_free_gpio:
	if(gpio_is_valid(that5173->gpio_rst))
		gpio_free(that5173->gpio_rst);
	return ret;
}

static int that5173_spi_remove(struct spi_device *spi)
{
	struct that5173_private *that5173 = spi_get_drvdata(spi);

	snd_soc_unregister_codec(&spi->dev);
	if(gpio_is_valid(that5173->gpio_rst))
		gpio_free(that5173->gpio_rst);

	gpiochip_remove(&that5173->gpio_chip);

	return 0;
}

static const struct of_device_id that5173_of_match[] = {
	{ .compatible = "that,that5173", },
	{ }
};
MODULE_DEVICE_TABLE(of, that5173_of_match);

static const struct spi_device_id that5173_id_table[] = {
	{ "that5173", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, that5173_id_table);

static struct spi_driver that5173_spi_driver = {
	.driver = {
		.name = "that5173",
		.owner = THIS_MODULE,
		.of_match_table = that5173_of_match,
	},
	.id_table = that5173_id_table,
	.probe = that5173_spi_probe,
	.remove = that5173_spi_remove,
};

module_spi_driver(that5173_spi_driver);

MODULE_AUTHOR("Matthew Campbell <mcampbell@izotope.com>");
MODULE_DESCRIPTION("THAT corp THAT5173 PGA ASoC codec driver");
MODULE_LICENSE("GPL v2");
