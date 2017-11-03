/*
 * Gamut AK4621 Alsa SoC driver
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
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "imx-audmux.h"
#include "../codecs/that5173.h"

// TODO add device tree property for board revision
// TODO add GPIO for Rev 0 for DRV_EN

struct gamut_ak4621_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
        int wgs_control_cache;
};

static int gamut_ak4621_wgs_info(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 26;
    uinfo->value.integer.step = 1;
    return 0;
}


static int gamut_ak4621_wgs_get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    struct gamut_ak4621_data *data = snd_kcontrol_chip(kcontrol);
    ucontrol->value.integer.value[0] = data->wgs_control_cache;
    return 0;
}

static int gamut_ak4621_wgs_put(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
        static const u8 WGS_GPO_OUTS[][6] = {
			{0,0,0,0,0,0},
			{0,0,0,1,0,0},
			{0,0,0,0,1,0},
			{0,0,0,1,1,0},
			{0,0,0,0,0,1},
			{0,0,0,0,1,1},
			{1,0,0,0,0,0},
			{1,0,0,1,0,0},
			{1,0,0,0,1,0},
			{1,0,0,1,1,0},
			{1,0,0,0,0,1},
			{1,0,0,0,1,1},
			{0,1,0,0,0,0},
			{0,1,0,1,0,0},
			{0,1,0,0,1,0},
			{1,1,0,0,1,0},
			{0,1,0,0,0,1},
			{0,1,0,0,1,1},
			{1,1,0,0,1,1},
			{1,0,1,0,0,0},
			{1,0,1,1,0,0},
			{1,1,1,1,0,0},
			{1,0,1,1,1,0},
			{0,0,1,0,0,1},
			{1,0,1,1,0,1},
			{1,1,1,1,0,1},
			{1,1,1,1,1,1},
        };
        struct gamut_ak4621_data *data = snd_kcontrol_chip(kcontrol); 
        u8 buf[(ARRAY_SIZE(*WGS_GPO_OUTS)+3)/4] = { 0 };
        int i;
        int result;
        int value = ucontrol->value.integer.value[0];
        for (i = 0; i < ARRAY_SIZE(*WGS_GPO_OUTS); i++) {
            buf[i/4] |= WGS_GPO_OUTS[value][i] ? THAT5173_SET_GPO(i%4) : THAT5173_CLEAR_GPO(i%4);
        }
        if (data->card.num_aux_devs < 1) {
            return -ENODEV;
        }
        result = that5173_set_gpo_values(data->card.rtd_aux[0].codec, buf, sizeof(buf));
        if (result == 0)
            data->wgs_control_cache = value;
        return result;
}

static DECLARE_TLV_DB_SCALE(gain_tlv, 0, 179, 0);

/* This is the control glue for the WGSpecial Preamp */
struct snd_kcontrol_new wgs_control = {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Mic Capture Volume",
        .index = 0,
        .access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .tlv.p = gain_tlv,
        .info = gamut_ak4621_wgs_info,
        .put = gamut_ak4621_wgs_put,
        .get = gamut_ak4621_wgs_get,
};

static int gamut_wgs_register_control(const struct device *dev, struct gamut_ak4621_data *data) 
{
    int err;
    if (!data->card.snd_card) 
        dev_err(dev, "data->card.snd_card is null\n");
    
    err = snd_ctl_add(data->card.snd_card, snd_ctl_new1(&wgs_control, data));
    return err;
}
    
static int gamut_ak4621_dai_init(struct snd_soc_pcm_runtime *rtd)
{
/*
	struct gamut_ak4621_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, SGTL5000_SYSCLK,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}
*/
	return 0;
}

static int gamut_ak4621_audmux_config(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int int_port, ext_port;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN, // set synchnous mode (use TXFS and TCLK for RXD and TXD)
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_TFSDIR |           // TFS output to CODEC
			IMX_AUDMUX_V2_PTCR_TFSEL(int_port) |  // Source it from the internal port
			IMX_AUDMUX_V2_PTCR_TCLKDIR |          // TCLK output to CODEC
			IMX_AUDMUX_V2_PTCR_TCSEL(int_port) |  // source it from internal port
			IMX_AUDMUX_V2_PTCR_SYN,               // set synchnous mode (use TXFS and TCLK for RXD and TXD)
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port)); // connect RXD to int port
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_aux_dev imx_aux_devs[] = {
	{
		.name = "that5173",
		// TODO: this should at least be in the devtree
		.codec_name = "spi1.0",
	},
};

int my_probe(struct snd_soc_card *card)
{
	dev_info(card->dev, "****************************In early probe\n");

	// if Rev 0 turn on DRV_EN

	// set div ctrl pins for PLL, don't worry about rate yet, this is to
	// get the power going




	return 0;
}

int my_late_probe(struct snd_soc_card *card)
{
	dev_info(card->dev, "****************************In late probe\n");
	return 0;
}

static int gamut_ak4621_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
	struct gamut_ak4621_data *data = NULL;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (strstr(cpu_np->name, "ssi")) {
		ret = gamut_ak4621_audmux_config(pdev);
		if (ret)
			goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	//TODO: would be nice to verifiy the codec_cp is really backed by a device
	//NOTE: if you have a device tree node, but no driver to back it currently
	// it will cause a kernel oops if you try to load this module... not so good.

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->dai.name = "name_ak4621";
	data->dai.stream_name = "stream-name_ak4621";
	data->dai.codec_dai_name = "ak4621";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = cpu_np;
	data->dai.platform_of_node = cpu_np;
	data->dai.init = &gamut_ak4621_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	data->card.name = "ak4621";
// remove audio routing device tree support for the moment...
#if 0
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
#endif
	data->card.probe = my_probe;
	data->card.late_probe = my_late_probe;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.num_aux_devs = 1;
	data->card.aux_dev = imx_aux_devs;
	//data->card.dapm_widgets = imx_sgtl5000_dapm_widgets;
	//data->card.num_dapm_widgets = ARRAY_SIZE(imx_sgtl5000_dapm_widgets);
	data->card.owner = THIS_MODULE;

        
	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(cpu_np);

        ret = gamut_wgs_register_control(&pdev->dev, data);
        if (ret) {
            dev_err(&pdev->dev, "gamut_wgs_register_control failed (%d)\n", ret);
            goto fail;
        }
        
	return 0;

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id gamut_ak4621_dt_ids[] = {
	{ .compatible = "izotope,gamut-audio-ak4621", },
	{},
};
MODULE_DEVICE_TABLE(of, gamut_ak4621_dt_ids);

static struct platform_driver gamut_ak4621_driver = {
	.driver = {
		.name	= "gamut-ak4621",
		.owner = THIS_MODULE,
		.of_match_table = gamut_ak4621_dt_ids,
	},
	.probe		= gamut_ak4621_probe,
	// need remove to unregister card?
};
module_platform_driver(gamut_ak4621_driver);

MODULE_AUTHOR("Matthew Campbell <mcampbell@izotope.com>");
MODULE_DESCRIPTION("iZotope Gamut ak4621 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gamut-ak4621");
