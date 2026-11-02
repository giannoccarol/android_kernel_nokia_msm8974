/*
 * tas2552-i2s.c - ALSA SoC Texas Instruments TAS2552 mono audio amplifier,
 * digital (I2S) input, for the 3.4 ASoC core.
 *
 * Port of the mainline Linux driver (sound/soc/codecs/tas2552.c):
 *
 * Copyright (C) 2014 Texas Instruments Incorporated -  http://www.ti.com
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Changes for this tree: 3.4 codec API (snd_soc_codec, regmap through
 * SND_SOC_REGMAP), integer enable GPIO and optional supplies from DT,
 * power handled through the bias level instead of runtime PM, software
 * shutdown left only while the serial clocks run, a soft reset before
 * every cache sync, and a status read-back after the stream starts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "tas2552-i2s.h"

/*
 * Power-on values. They match a register dump of the Lumia 1520 part
 * (VER_NUM 0x08) taken right after its enable pin went high, which also
 * agrees with the mainline table except BOOST_APT_CTRL (0x00 here, 0x0f
 * there). The cache sync skips registers equal to these values, so they
 * must be the real reset values.
 */
static const struct reg_default tas2552_reg_defs[] = {
	{TAS2552_CFG_1, 0x22},
	{TAS2552_CFG_2, 0xef},
	{TAS2552_CFG_3, 0x80},
	{TAS2552_DOUT, 0x00},
	{TAS2552_SER_CTRL_1, 0x00},
	{TAS2552_SER_CTRL_2, 0x00},
	{TAS2552_OUTPUT_DATA, 0xc0},
	{TAS2552_PLL_CTRL_1, 0x10},
	{TAS2552_PLL_CTRL_2, 0x00},
	{TAS2552_PLL_CTRL_3, 0x00},
	{TAS2552_BTIP, 0x8f},
	{TAS2552_BTS_CTRL, 0x80},
	{TAS2552_RESERVED_0D, 0xbe},
	{TAS2552_LIMIT_RATE_HYS, 0x08},
	{TAS2552_LIMIT_RELEASE, 0x04},
	{TAS2552_LIMIT_INT_COUNT, 0x00},
	{TAS2552_PDM_CFG, 0x01},
	{TAS2552_PGA_GAIN, 0x00},
	{TAS2552_EDGE_RATE_CTRL, 0x40},
	{TAS2552_BOOST_APT_CTRL, 0x00},
};

#define TAS2552_NUM_SUPPLIES	3
static const char * const tas2552_supply_names[TAS2552_NUM_SUPPLIES] = {
	"vbat",		/* vbat voltage */
	"iovdd",	/* I/O Voltage */
	"avdd",		/* Analog DAC Voltage */
};

struct tas2552_data {
	struct snd_soc_codec *codec;
	struct regmap *regmap;
	struct i2c_client *tas2552_client;
	struct regulator_bulk_data supplies[TAS2552_NUM_SUPPLIES];
	int num_supplies;
	int enable_gpio;
	bool powered;
	struct mutex power_lock;
	struct delayed_work status_work;

	unsigned int pll_clkin;
	int pll_clk_id;
	unsigned int pdm_clk;
	int pdm_clk_id;

	unsigned int dai_fmt;
	unsigned int tdm_delay;
};

static bool tas2552_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2552_DEVICE_STATUS:
	case TAS2552_VER_NUM:
	case 0x17:
	case TAS2552_VBOOST_DATA:
	case TAS2552_VBAT_DATA:
		return true;
	default:
		return false;
	}
}

/*
 * Power sequence (datasheet): supplies, then the enable pin, then I2C.
 * The part comes up in software shutdown; it leaves it only once the
 * serial clocks run (tas2552_digital_mute).
 */
static int tas2552_power_on(struct tas2552_data *tas2552)
{
	struct device *dev = &tas2552->tas2552_client->dev;
	int ret = 0;

	mutex_lock(&tas2552->power_lock);
	if (tas2552->powered)
		goto out;

	if (tas2552->num_supplies) {
		ret = regulator_bulk_enable(tas2552->num_supplies,
					    tas2552->supplies);
		if (ret) {
			dev_err(dev, "Failed to enable supplies: %d\n", ret);
			goto out;
		}
	}

	if (gpio_is_valid(tas2552->enable_gpio)) {
		gpio_set_value_cansleep(tas2552->enable_gpio, 1);
		usleep_range(2000, 2500);
	}

	/* Start from the reset values the cache sync expects */
	ret = i2c_smbus_write_byte_data(tas2552->tas2552_client,
					TAS2552_CFG_1, TAS2552_DEV_RESET);
	if (ret < 0) {
		dev_err(dev, "No answer from the amplifier: %d\n", ret);
		goto fail;
	}
	usleep_range(1000, 1500);

	regcache_cache_only(tas2552->regmap, false);
	regcache_mark_dirty(tas2552->regmap);
	ret = regcache_sync(tas2552->regmap);
	if (ret) {
		dev_err(dev, "Failed to restore registers: %d\n", ret);
		regcache_cache_only(tas2552->regmap, true);
		regcache_mark_dirty(tas2552->regmap);
		goto fail;
	}

	tas2552->powered = true;
	dev_dbg(dev, "powered on\n");
	goto out;

fail:
	if (gpio_is_valid(tas2552->enable_gpio))
		gpio_set_value_cansleep(tas2552->enable_gpio, 0);
	if (tas2552->num_supplies)
		regulator_bulk_disable(tas2552->num_supplies,
				       tas2552->supplies);
out:
	mutex_unlock(&tas2552->power_lock);
	return ret;
}

static void tas2552_power_off(struct tas2552_data *tas2552)
{
	cancel_delayed_work_sync(&tas2552->status_work);

	mutex_lock(&tas2552->power_lock);
	if (!tas2552->powered)
		goto out;

	regmap_update_bits(tas2552->regmap, TAS2552_CFG_1,
			   TAS2552_MUTE | TAS2552_SWS,
			   TAS2552_MUTE | TAS2552_SWS);

	/* Without an enable pin the part stays on, in software shutdown */
	if (!gpio_is_valid(tas2552->enable_gpio))
		goto out;

	regcache_cache_only(tas2552->regmap, true);
	regcache_mark_dirty(tas2552->regmap);
	gpio_set_value_cansleep(tas2552->enable_gpio, 0);

	if (tas2552->num_supplies)
		regulator_bulk_disable(tas2552->num_supplies,
				       tas2552->supplies);

	tas2552->powered = false;
	dev_dbg(&tas2552->tas2552_client->dev, "powered off\n");
out:
	mutex_unlock(&tas2552->power_lock);
}

static void tas2552_status_work(struct work_struct *work)
{
	struct tas2552_data *tas2552 = container_of(work, struct tas2552_data,
						    status_work.work);
	static const u8 regs[] = {
		TAS2552_DEVICE_STATUS, TAS2552_CFG_1, TAS2552_CFG_2,
		TAS2552_CFG_3, TAS2552_SER_CTRL_1, TAS2552_PLL_CTRL_1,
		TAS2552_PLL_CTRL_2, TAS2552_PLL_CTRL_3, TAS2552_PGA_GAIN,
		TAS2552_VBOOST_DATA, TAS2552_VBAT_DATA,
	};
	int val[ARRAY_SIZE(regs)];
	int i;

	/* Read the chip itself, not the cache */
	mutex_lock(&tas2552->power_lock);
	if (!tas2552->powered) {
		mutex_unlock(&tas2552->power_lock);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		val[i] = i2c_smbus_read_byte_data(tas2552->tas2552_client,
						  regs[i]);
	mutex_unlock(&tas2552->power_lock);

	dev_info(&tas2552->tas2552_client->dev,
		 "playing: status %02x cfg1 %02x cfg2 %02x cfg3 %02x ser1 %02x pll %02x %02x %02x gain %02x vboost %02x vbat %02x\n",
		 val[0], val[1], val[2], val[3], val[4], val[5], val[6],
		 val[7], val[8], val[9], val[10]);
}

static int tas2552_classd_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Same values as the mainline post-PMU sequence */
		snd_soc_write(codec, TAS2552_RESERVED_0D, 0xc0);
		snd_soc_update_bits(codec, TAS2552_LIMIT_RATE_HYS, (1 << 5),
				    (1 << 5));
		snd_soc_update_bits(codec, TAS2552_CFG_2, 1, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, TAS2552_CFG_1,
				    TAS2552_MUTE | TAS2552_SWS,
				    TAS2552_MUTE | TAS2552_SWS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TAS2552_CFG_2, 1, 1);
		snd_soc_update_bits(codec, TAS2552_LIMIT_RATE_HYS, (1 << 5), 0);
		snd_soc_write(codec, TAS2552_RESERVED_0D, 0xbe);
		break;
	}
	return 0;
}

/*
 * The analog input is not wired on the boards this driver serves: the
 * input mux of the mainline driver is fixed to the digital input.
 */
static const struct snd_soc_dapm_widget tas2552_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("TAS2552 DAC IN", "TAS2552 Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("TAS2552 DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("TAS2552 ClassD", TAS2552_CFG_2, 7, 0, NULL, 0,
			       tas2552_classd_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("TAS2552 PLL", TAS2552_CFG_2, 3, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("TAS2552 OUT"),
};

static const struct snd_soc_dapm_route tas2552_audio_map[] = {
	{"TAS2552 DAC", NULL, "TAS2552 DAC IN"},
	{"TAS2552 ClassD", NULL, "TAS2552 DAC"},
	{"TAS2552 ClassD", NULL, "TAS2552 PLL"},
	{"TAS2552 OUT", NULL, "TAS2552 ClassD"},
};

static unsigned int tas2552_bclk(struct tas2552_data *tas2552,
				 struct snd_pcm_hw_params *params)
{
	int width = snd_pcm_format_width(params_format(params));

	/* 16-bit data in 16-bit slots, anything wider in 32-bit slots */
	return params_rate(params) * (width <= 16 ? 32 : 64) +
	       tas2552->tdm_delay;
}

static int tas2552_setup_pll(struct snd_soc_codec *codec,
			     struct snd_pcm_hw_params *params)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	bool bypass_pll = false;
	unsigned int pll_clk = params_rate(params) * 512;
	unsigned int pll_clkin = tas2552->pll_clkin;
	u8 pll_enable;

	if (!pll_clkin) {
		if (tas2552->pll_clk_id != TAS2552_PLL_CLKIN_BCLK)
			return -EINVAL;

		pll_clkin = tas2552_bclk(tas2552, params);
	}

	pll_enable = snd_soc_read(codec, TAS2552_CFG_2) & TAS2552_PLL_ENABLE;
	snd_soc_update_bits(codec, TAS2552_CFG_2, TAS2552_PLL_ENABLE, 0);

	if (pll_clkin == pll_clk)
		bypass_pll = true;

	if (bypass_pll) {
		/* By pass the PLL configuration */
		snd_soc_update_bits(codec, TAS2552_PLL_CTRL_2,
				    TAS2552_PLL_BYPASS, TAS2552_PLL_BYPASS);
	} else {
		/* Fill in the PLL control registers for J & D
		 * pll_clk = (.5 * pll_clkin * J.D) / 2^p
		 * Need to fill in J and D here based on incoming freq
		 */
		unsigned int d, q, t;
		u8 j;
		u8 pll_sel = (tas2552->pll_clk_id << 3) & TAS2552_PLL_SRC_MASK;
		u8 p = snd_soc_read(codec, TAS2552_PLL_CTRL_1);

		p = (p >> 7);

recalc:
		t = (pll_clk * 2) << p;
		j = t / pll_clkin;
		d = t % pll_clkin;
		t = pll_clkin / 10000;
		q = d / (t + 1);
		d = q + ((9999 - pll_clkin % 10000) * (d / t - q)) / 10000;

		if (d && (pll_clkin < 512000 || pll_clkin > 9200000)) {
			if (tas2552->pll_clk_id == TAS2552_PLL_CLKIN_BCLK) {
				pll_clkin = 1800000;
				pll_sel = (TAS2552_PLL_CLKIN_1_8_FIXED << 3) &
							TAS2552_PLL_SRC_MASK;
			} else {
				pll_clkin = tas2552_bclk(tas2552, params);
				pll_sel = (TAS2552_PLL_CLKIN_BCLK << 3) &
							TAS2552_PLL_SRC_MASK;
			}
			goto recalc;
		}

		snd_soc_update_bits(codec, TAS2552_CFG_1, TAS2552_PLL_SRC_MASK,
				    pll_sel);

		snd_soc_update_bits(codec, TAS2552_PLL_CTRL_1,
				    TAS2552_PLL_J_MASK, j);
		/* Will clear the PLL_BYPASS bit */
		snd_soc_write(codec, TAS2552_PLL_CTRL_2,
			      TAS2552_PLL_D_UPPER(d));
		snd_soc_write(codec, TAS2552_PLL_CTRL_3,
			      TAS2552_PLL_D_LOWER(d));

		dev_dbg(codec->dev, "PLL: in %u Hz, out %u Hz, J %u D %u P %u\n",
			pll_clkin, pll_clk, j, d, p);
	}

	/* Restore PLL status */
	snd_soc_update_bits(codec, TAS2552_CFG_2, TAS2552_PLL_ENABLE,
			    pll_enable);

	return 0;
}

static int tas2552_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	int width = snd_pcm_format_width(params_format(params));
	int cpf;
	u8 ser_ctrl1_reg, wclk_rate;

	switch (width) {
	case 16:
		ser_ctrl1_reg = TAS2552_WORDLENGTH_16BIT;
		cpf = 32 + tas2552->tdm_delay;
		break;
	case 20:
		ser_ctrl1_reg = TAS2552_WORDLENGTH_20BIT;
		cpf = 64 + tas2552->tdm_delay;
		break;
	case 24:
		ser_ctrl1_reg = TAS2552_WORDLENGTH_24BIT;
		cpf = 64 + tas2552->tdm_delay;
		break;
	case 32:
		ser_ctrl1_reg = TAS2552_WORDLENGTH_32BIT;
		cpf = 64 + tas2552->tdm_delay;
		break;
	default:
		dev_err(codec->dev, "Not supported sample size: %d\n", width);
		return -EINVAL;
	}

	if (cpf <= 32)
		ser_ctrl1_reg |= TAS2552_CLKSPERFRAME_32;
	else if (cpf <= 64)
		ser_ctrl1_reg |= TAS2552_CLKSPERFRAME_64;
	else if (cpf <= 128)
		ser_ctrl1_reg |= TAS2552_CLKSPERFRAME_128;
	else
		ser_ctrl1_reg |= TAS2552_CLKSPERFRAME_256;

	snd_soc_update_bits(codec, TAS2552_SER_CTRL_1,
			    TAS2552_WORDLENGTH_MASK | TAS2552_CLKSPERFRAME_MASK,
			    ser_ctrl1_reg);

	switch (params_rate(params)) {
	case 8000:
		wclk_rate = TAS2552_WCLK_FREQ_8KHZ;
		break;
	case 11025:
	case 12000:
		wclk_rate = TAS2552_WCLK_FREQ_11_12KHZ;
		break;
	case 16000:
		wclk_rate = TAS2552_WCLK_FREQ_16KHZ;
		break;
	case 22050:
	case 24000:
		wclk_rate = TAS2552_WCLK_FREQ_22_24KHZ;
		break;
	case 32000:
		wclk_rate = TAS2552_WCLK_FREQ_32KHZ;
		break;
	case 44100:
	case 48000:
		wclk_rate = TAS2552_WCLK_FREQ_44_48KHZ;
		break;
	case 88200:
	case 96000:
		wclk_rate = TAS2552_WCLK_FREQ_88_96KHZ;
		break;
	case 176400:
	case 192000:
		wclk_rate = TAS2552_WCLK_FREQ_176_192KHZ;
		break;
	default:
		dev_err(codec->dev, "Not supported sample rate: %d\n",
			params_rate(params));
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_CFG_3, TAS2552_WCLK_FREQ_MASK,
			    wclk_rate);

	dev_dbg(codec->dev, "hw_params: %u Hz, %d bit, %d channels\n",
		params_rate(params), width, params_channels(params));

	return tas2552_setup_pll(codec, params);
}

#define TAS2552_DAI_FMT_MASK	(TAS2552_BCLKDIR | \
				 TAS2552_WCLKDIR | \
				 TAS2552_DATAFORMAT_MASK)
static int tas2552_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	int delay = 0;

	/* TDM slot selection only valid in DSP_A/_B mode */
	if (tas2552->dai_fmt == SND_SOC_DAIFMT_DSP_A)
		delay += (tas2552->tdm_delay + 1);
	else if (tas2552->dai_fmt == SND_SOC_DAIFMT_DSP_B)
		delay += tas2552->tdm_delay;

	/* Configure data delay */
	snd_soc_write(codec, TAS2552_SER_CTRL_2, delay);

	return 0;
}

static int tas2552_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	u8 serial_format;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		serial_format = 0x00;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		serial_format = TAS2552_WCLKDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		serial_format = TAS2552_BCLKDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		serial_format = (TAS2552_BCLKDIR | TAS2552_WCLKDIR);
		break;
	default:
		dev_vdbg(codec->dev, "DAI Format master is not found\n");
		return -EINVAL;
	}

	switch (fmt & (SND_SOC_DAIFMT_FORMAT_MASK |
		       SND_SOC_DAIFMT_INV_MASK)) {
	case (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF):
		break;
	case (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF):
	case (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF):
		serial_format |= TAS2552_DATAFORMAT_DSP;
		break;
	case (SND_SOC_DAIFMT_RIGHT_J | SND_SOC_DAIFMT_NB_NF):
		serial_format |= TAS2552_DATAFORMAT_RIGHT_J;
		break;
	case (SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF):
		serial_format |= TAS2552_DATAFORMAT_LEFT_J;
		break;
	default:
		dev_vdbg(codec->dev, "DAI Format is not found\n");
		return -EINVAL;
	}
	tas2552->dai_fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	snd_soc_update_bits(codec, TAS2552_SER_CTRL_1, TAS2552_DAI_FMT_MASK,
			    serial_format);
	return 0;
}

static int tas2552_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	u8 reg, mask, val;

	switch (clk_id) {
	case TAS2552_PLL_CLKIN_MCLK:
	case TAS2552_PLL_CLKIN_IVCLKIN:
		if (freq < 512000 || freq > 24576000) {
			/* out of range PLL_CLKIN, fall back to use BCLK */
			dev_warn(codec->dev, "Out of range PLL_CLKIN: %u\n",
				 freq);
			clk_id = TAS2552_PLL_CLKIN_BCLK;
			freq = 0;
		}
		/* fall through */
	case TAS2552_PLL_CLKIN_BCLK:
	case TAS2552_PLL_CLKIN_1_8_FIXED:
		mask = TAS2552_PLL_SRC_MASK;
		val = (clk_id << 3) & mask; /* bit 4:5 in the register */
		reg = TAS2552_CFG_1;
		tas2552->pll_clk_id = clk_id;
		tas2552->pll_clkin = freq;
		break;
	case TAS2552_PDM_CLK_PLL:
	case TAS2552_PDM_CLK_IVCLKIN:
	case TAS2552_PDM_CLK_BCLK:
	case TAS2552_PDM_CLK_MCLK:
		mask = TAS2552_PDM_CLK_SEL_MASK;
		val = (clk_id >> 1) & mask; /* bit 0:1 in the register */
		reg = TAS2552_PDM_CFG;
		tas2552->pdm_clk_id = clk_id;
		tas2552->pdm_clk = freq;
		break;
	default:
		dev_err(codec->dev, "Invalid clk id: %d\n", clk_id);
		return -EINVAL;
	}

	snd_soc_update_bits(codec, reg, mask, val);

	return 0;
}

static void tas2552_shutdown_amp(struct snd_soc_codec *codec)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

	cancel_delayed_work(&tas2552->status_work);

	/* Mute, let the volume ramp down, then software shutdown */
	if (snd_soc_update_bits(codec, TAS2552_CFG_1, TAS2552_MUTE,
				TAS2552_MUTE) > 0)
		msleep(10);
	snd_soc_update_bits(codec, TAS2552_CFG_1, TAS2552_SWS, TAS2552_SWS);
}

/*
 * The 3.4 core unmutes at the end of prepare, after the CPU DAI has
 * started the port, so the bit and word clocks already run here: leave
 * software shutdown now, as the datasheet asks, rather than when DAPM
 * powers the class-D stage (before the port starts).
 */
static int tas2552_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

	if (mute) {
		tas2552_shutdown_amp(codec);
	} else {
		/*
		 * DAPM enabled the PLL before its BCLK reference existed:
		 * restart it now that the clock runs, then leave shutdown.
		 */
		if (snd_soc_read(codec, TAS2552_CFG_2) & TAS2552_PLL_ENABLE) {
			snd_soc_update_bits(codec, TAS2552_CFG_2,
					    TAS2552_PLL_ENABLE, 0);
			snd_soc_update_bits(codec, TAS2552_CFG_2,
					    TAS2552_PLL_ENABLE,
					    TAS2552_PLL_ENABLE);
			usleep_range(1000, 1500);
		}
		snd_soc_update_bits(codec, TAS2552_CFG_1,
				    TAS2552_SWS | TAS2552_MUTE, 0);
		schedule_delayed_work(&tas2552->status_work,
				      msecs_to_jiffies(300));
	}

	return 0;
}

/* Called before the CPU DAI stops the port and its clocks */
static int tas2552_hw_free(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	tas2552_shutdown_amp(dai->codec);
	return 0;
}

static const struct snd_soc_dai_ops tas2552_speaker_dai_ops = {
	.hw_params	= tas2552_hw_params,
	.prepare	= tas2552_prepare,
	.hw_free	= tas2552_hw_free,
	.set_sysclk	= tas2552_set_dai_sysclk,
	.set_fmt	= tas2552_set_dai_fmt,
	.digital_mute	= tas2552_digital_mute,
};

/* Formats supported by TAS2552 driver. */
#define TAS2552_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* TAS2552 dai structure. */
static struct snd_soc_dai_driver tas2552_dai[] = {
	{
		.name = "tas2552-amplifier",
		.playback = {
			.stream_name = "TAS2552 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = TAS2552_FORMATS,
		},
		.ops = &tas2552_speaker_dai_ops,
	},
};

/*
 * DAC digital volumes. From -7 to 24 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, -700, 100, 0);

static const char * const tas2552_din_source_select[] = {
	"Muted",
	"Left",
	"Right",
	"Left + Right average",
};
static SOC_ENUM_SINGLE_DECL(tas2552_din_source_enum,
			    TAS2552_CFG_3, 3,
			    tas2552_din_source_select);

static const struct snd_kcontrol_new tas2552_snd_controls[] = {
	SOC_SINGLE_TLV("TAS2552 Speaker Volume",
			 TAS2552_PGA_GAIN, 0, 0x1f, 0, dac_tlv),
	SOC_ENUM("TAS2552 DIN Source", tas2552_din_source_enum),
};

static int tas2552_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = tas2552_power_on(tas2552);
			if (ret)
				return ret;
		}
		break;
	case SND_SOC_BIAS_OFF:
		tas2552_power_off(tas2552);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static int tas2552_codec_probe(struct snd_soc_codec *codec)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);
	unsigned int ver = 0;
	int ret;

	tas2552->codec = codec;
	codec->control_data = tas2552->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);
	if (ret) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	/*
	 * Configuration from the mainline probe, kept in the cache and
	 * written at every power-up: muted, software shutdown, PLL from
	 * BCLK, digital input averaging both I2S channels, I/V data on the
	 * serial output, boost with auto pass-through, limiter on. Class-D
	 * and PLL stay off until DAPM needs them.
	 */
	snd_soc_update_bits(codec, TAS2552_CFG_1,
			    TAS2552_MUTE | TAS2552_SWS | TAS2552_PLL_SRC_MASK,
			    TAS2552_MUTE | TAS2552_SWS | TAS2552_PLL_SRC_BCLK);
	snd_soc_write(codec, TAS2552_CFG_3, TAS2552_I2S_OUT_SEL |
					    TAS2552_DIN_SRC_SEL_AVG_L_R |
					    TAS2552_WCLK_FREQ_44_48KHZ);
	snd_soc_write(codec, TAS2552_SER_CTRL_1, TAS2552_DATAFORMAT_I2S |
						 TAS2552_WORDLENGTH_24BIT |
						 TAS2552_CLKSPERFRAME_64);
	snd_soc_write(codec, TAS2552_OUTPUT_DATA,
		      TAS2552_PDM_DATA_SEL_V_I |
		      TAS2552_R_DATA_OUT(TAS2552_DATA_OUT_V_DATA));
	snd_soc_write(codec, TAS2552_BOOST_APT_CTRL, TAS2552_APT_DELAY_200 |
						     TAS2552_APT_THRESH_20_17);
	snd_soc_write(codec, TAS2552_CFG_2, TAS2552_BOOST_EN | TAS2552_APT_EN |
					    TAS2552_LIM_EN);

	/* Check once that the part answers, then let DAPM power it */
	ret = tas2552_power_on(tas2552);
	if (ret) {
		/* Keep the card usable: the earpiece and the rest still work */
		dev_err(codec->dev, "TAS2552 not responding (%d)\n", ret);
		return 0;
	}
	regmap_read(tas2552->regmap, TAS2552_VER_NUM, &ver);
	dev_info(codec->dev, "TAS2552 found, version 0x%02x\n", ver);
	tas2552_power_off(tas2552);

	return 0;
}

static int tas2552_codec_remove(struct snd_soc_codec *codec)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

	tas2552_power_off(tas2552);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_tas2552 = {
	.probe			= tas2552_codec_probe,
	.remove			= tas2552_codec_remove,
	.set_bias_level		= tas2552_set_bias_level,
	.idle_bias_off		= true,
	.controls		= tas2552_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2552_snd_controls),
	.dapm_widgets		= tas2552_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas2552_dapm_widgets),
	.dapm_routes		= tas2552_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas2552_audio_map),
};

static const struct regmap_config tas2552_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = TAS2552_MAX_REG,
	.volatile_reg = tas2552_volatile_reg,
	.reg_defaults = tas2552_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(tas2552_reg_defs),
	.cache_type = REGCACHE_RBTREE,
};

static int __devinit tas2552_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct tas2552_data *data;
	char prop[32];
	int ret;
	int i;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->tas2552_client = client;
	data->pll_clk_id = TAS2552_PLL_CLKIN_BCLK;
	mutex_init(&data->power_lock);
	INIT_DELAYED_WORK(&data->status_work, tas2552_status_work);

	data->enable_gpio = -EINVAL;
	if (np) {
		data->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
		if (data->enable_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (!gpio_is_valid(data->enable_gpio))
			data->enable_gpio = of_get_named_gpio(np,
							"ti,enable-gpio", 0);
		if (data->enable_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}
	if (gpio_is_valid(data->enable_gpio)) {
		/* Hardware shutdown until the first power-up */
		ret = gpio_request_one(data->enable_gpio, GPIOF_OUT_INIT_LOW,
				       "tas2552-enable");
		if (ret) {
			dev_err(dev, "Failed to request enable GPIO %d: %d\n",
				data->enable_gpio, ret);
			return ret;
		}
	} else {
		dev_warn(dev, "No enable GPIO: the part stays on\n");
	}

	/* Supplies are optional: on the Lumia 1520 they are always on */
	for (i = 0; i < TAS2552_NUM_SUPPLIES; i++) {
		snprintf(prop, sizeof(prop), "%s-supply",
			 tas2552_supply_names[i]);
		if (np && of_find_property(np, prop, NULL))
			data->supplies[data->num_supplies++].supply =
				tas2552_supply_names[i];
	}
	if (data->num_supplies) {
		ret = devm_regulator_bulk_get(dev, data->num_supplies,
					      data->supplies);
		if (ret) {
			dev_err(dev, "Failed to request supplies: %d\n", ret);
			goto err_gpio;
		}
	}

	data->regmap = devm_regmap_init_i2c(client, &tas2552_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		goto err_gpio;
	}
	/* The part is off: keep writes in the cache until it powers up */
	regcache_cache_only(data->regmap, true);
	regcache_mark_dirty(data->regmap);

	i2c_set_clientdata(client, data);

	ret = snd_soc_register_codec(dev, &soc_codec_dev_tas2552,
				     tas2552_dai, ARRAY_SIZE(tas2552_dai));
	if (ret < 0) {
		dev_err(dev, "Failed to register codec: %d\n", ret);
		goto err_gpio;
	}

	return 0;

err_gpio:
	if (gpio_is_valid(data->enable_gpio))
		gpio_free(data->enable_gpio);
	return ret;
}

static int __devexit tas2552_i2c_remove(struct i2c_client *client)
{
	struct tas2552_data *data = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	cancel_delayed_work_sync(&data->status_work);
	if (gpio_is_valid(data->enable_gpio)) {
		gpio_set_value_cansleep(data->enable_gpio, 0);
		gpio_free(data->enable_gpio);
	}
	return 0;
}

static const struct i2c_device_id tas2552_id[] = {
	{ "tas2552", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2552_id);

#ifdef CONFIG_OF
static const struct of_device_id tas2552_of_match[] = {
	{ .compatible = "ti,tas2552", },
	{},
};
MODULE_DEVICE_TABLE(of, tas2552_of_match);
#endif

static struct i2c_driver tas2552_i2c_driver = {
	.driver = {
		.name = "tas2552",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tas2552_of_match),
	},
	.probe = tas2552_probe,
	.remove = __devexit_p(tas2552_i2c_remove),
	.id_table = tas2552_id,
};

module_i2c_driver(tas2552_i2c_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TAS2552 Audio amplifier driver, I2S input");
MODULE_LICENSE("GPL");
