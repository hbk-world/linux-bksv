/*
 * Copyright (C) 2010 Juergen Beisert, Pengutronix
 *
 * This code is based on:
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DRIVER_NAME "mxsfb"

/**
 * @file
 * @brief LCDIF driver for i.MX23 and i.MX28
 *
 * The LCDIF support four modes of operation
 * - MPU interface (to drive smart displays) -> not supported yet
 * - VSYNC interface (like MPU interface plus Vsync) -> not supported yet
 * - Dotclock interface (to drive LC displays with RGB data and sync signals)
 * - DVI (to drive ITU-R BT656)  -> not supported yet
 *
 * This driver depends on a correct setup of the pins used for this purpose
 * (platform specific).
 *
 * For the developer: Don't forget to set the data bus width to the display
 * in the imx_fb_videomode structure. You will else end up with ugly colours.
 * If you fight against jitter you can vary the clock delay. This is a feature
 * of the i.MX28 and you can vary it between 2 ns ... 8 ns in 2 ns steps. Give
 * the required value in the imx_fb_videomode structure.
 */

 /*
  * 2017-03-13
  * Lars Thestrup:
  * Based on patch for iMX6UL and iMX7D by Qiang Li
  * who made MPU interface for displays using "Command-Parameter interface mode""
  * 
  * Added support for display COM24H2P39ULC with Himax HX8347-A01 controller
  * on iMX7D
  * This display is 2.4" QVGA 240 x RGB x 320 Portrait, 
  *   using "Register-Content Interface Mode"
  * Only functions relevant for this display in this mode is tested.
  * If used on iMX7Dsabre board, the ethernet port 2 and HDMI display is disabled
  * Based on kernel 4.1.38
  */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
 

#include <linux/busfreq-imx.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>

#include <asm/cacheflush.h>
#include "mxc/mxc_dispdrv.h"
#include "mxsfb.h"

//#define LCD_RESET //uncomment this, if u-boot don't reset the display

static struct mpu_match_lcd mpu_lcd_db[] = {
#ifdef CONFIG_FB_MXS_ST7789S_QVGA
	{
	 "ST7789S-QVGA",
	 {mpu_st7789s_get_lcd_videomode, mpu_st7789s_lcd_setup, mpu_st7789s_lcd_poweroff}
	},
#endif
#ifdef CONFIG_FB_MXS_ST7735R_SQUARE
	{
	 "ST7735R-SQUARE",
	 {mpu_st7735r_get_lcd_videomode, mpu_st7735r_lcd_setup, mpu_st7735r_lcd_poweroff}
	},
#endif
#ifdef CONFIG_FB_MXS_COM24H2P39ULC
	{
	 "COM24H2P39ULC",
	 {mpu_com24h2p39_get_lcd_videomode, mpu_com24h2p39_lcd_setup, mpu_com24h2p39_lcd_poweroff}
	},
#endif
	{
	"", {NULL, NULL}
	}
};

#define mxsfb_is_v3(host) (host->devdata->ipversion == 3)
#define mxsfb_is_v4(host) (host->devdata->ipversion == 4)

static const struct mxsfb_devdata mxsfb_devdata[] = {
	[MXSFB_V3] = {
		.transfer_count = LCDC_V3_TRANSFER_COUNT,
		.data = LCDC_V3_DATA,
		.cur_buf = LCDC_V3_CUR_BUF,
		.next_buf = LCDC_V3_NEXT_BUF,
		.debug0 = LCDC_V3_DEBUG0,
		.hs_wdth_mask = 0xff,
		.hs_wdth_shift = 24,
		.ipversion = 3,
	},
	[MXSFB_V4] = {
		.transfer_count = LCDC_V4_TRANSFER_COUNT,
		.data = LCDC_V4_DATA,
		.cur_buf = LCDC_V4_CUR_BUF,
		.next_buf = LCDC_V4_NEXT_BUF,
		.debug0 = LCDC_V4_DEBUG0,
		.hs_wdth_mask = 0x3fff,
		.hs_wdth_shift = 18,
		.ipversion = 4,
	},
};

static int mxsfb_map_videomem(struct fb_info *info);
static int mxsfb_unmap_videomem(struct fb_info *info);
static int mxsfb_set_par(struct fb_info *fb_info);
#ifndef LCD_RESET
static int FirstReset=1;
#endif

static ssize_t set_inhibit_updates(struct device *pdev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
    int ret;
    unsigned long val;

    ret = kstrtoul(buf, 10, &val);
    if (ret) {
        return ret;
	}

	dev_warn(pdev, "Inhibit display updates = %ld", val);

	host->inhibit_updates = val;
    return count;
}

static ssize_t get_inhibit_updates(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", host->inhibit_updates ? 1 : 0);
}

static DEVICE_ATTR(inhibit_updates, S_IWUSR | S_IRUSR, get_inhibit_updates, set_inhibit_updates);

static struct attribute *mxsfb_attr[] = {
    &dev_attr_inhibit_updates.attr,
    NULL
};
 
 static const struct attribute_group mxsfb_attr_group = {
    .attrs = mxsfb_attr,
};

/* enable lcdif pix clock */
static inline void clk_enable_pix(struct mxsfb_info *host)
{
	if (!host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_prepare_enable(host->clk_pix);
		host->clk_pix_enabled = true;
	}
}

/* disable lcdif pix clock */
static inline void clk_disable_pix(struct mxsfb_info *host)
{
	if (host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_disable_unprepare(host->clk_pix);
		host->clk_pix_enabled = false;
	}
}

/* enable lcdif axi clock */
static inline void clk_enable_axi(struct mxsfb_info *host)
{
	if (!host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_prepare_enable(host->clk_axi);
		host->clk_axi_enabled = true;
	}
}

/* disable lcdif axi clock */
static inline void clk_disable_axi(struct mxsfb_info *host)
{
	if (host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_disable_unprepare(host->clk_axi);
		host->clk_axi_enabled = false;
	}
}

/* enable DISP axi clock */
static inline void clk_enable_disp_axi(struct mxsfb_info *host)
{
	if (!host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_prepare_enable(host->clk_disp_axi);
		host->clk_disp_axi_enabled = true;
	}
}

/* disable DISP axi clock */
static inline void clk_disable_disp_axi(struct mxsfb_info *host)
{
	if (host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_disable_unprepare(host->clk_disp_axi);
		host->clk_disp_axi_enabled = false;
	}
}

/* mask and shift depends on architecture */
static inline u32 set_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val & host->devdata->hs_wdth_mask) <<
		host->devdata->hs_wdth_shift;
}

static inline u32 get_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val >> host->devdata->hs_wdth_shift) &
		host->devdata->hs_wdth_mask;
}

static const struct fb_bitfield def_rgb565[] = {
	[RED] = {
		.offset = 11,
		.length = 5,
	},
	[GREEN] = {
		.offset = 5,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 5,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb666[] = {
	[RED] = {
		.offset = 16,
		.length = 6,
	},
	[GREEN] = {
		.offset = 8,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 6,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb888[] = {
	[RED] = {
		.offset = 16,
		.length = 8,
	},
	[GREEN] = {
		.offset = 8,
		.length = 8,
	},
	[BLUE] = {
		.offset = 0,
		.length = 8,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

#define bitfield_is_equal(f1, f2)  (!memcmp(&(f1), &(f2), sizeof(f1)))

static inline bool pixfmt_is_equal(struct fb_var_screeninfo *var,
				   const struct fb_bitfield *f)
{
	if (bitfield_is_equal(var->red, f[RED]) &&
	    bitfield_is_equal(var->green, f[GREEN]) &&
	    bitfield_is_equal(var->blue, f[BLUE]))
		return true;

	return false;
}

static inline unsigned chan_to_field(unsigned chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

int mxsfb_mpu_wait_for_ready(struct mxsfb_info *host)
{
	unsigned int tmp;
	int timeout = 0;

	// Check for running
	tmp = readl(host->base + LCDC_CTRL);
	while(tmp & CTRL_RUN) {
		mdelay(1);
		timeout ++;
		if (timeout >= 100) {
			dev_err(&host->pdev->dev, "mxsfb_mpu_wait_for_ready timeout!\n");
			return -ETIME;
		}
		tmp = readl(host->base + LCDC_CTRL);
	}

	return 0;
}

/*
 *Command-Parameter interface mode
 */
unsigned int mxsfb_mpu_access(struct mxsfb_info *host, int mode, int rw, int data)
{
	unsigned int tmp, wordlen, ret = 0;

	if (mxsfb_mpu_wait_for_ready(host) != 0)
		return 0;

	writel(CTRL_MASTER,	                   host->base + LCDC_CTRL  + REG_CLR);
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK, host->base + LCDC_CTRL1 + REG_CLR);

	tmp = readl(host->base + LCDC_CTRL);
	wordlen = CTRL_GET_WORD_LENGTH(tmp);
	writel(CTRL_WORD_LENGTH_MASK,          host->base + LCDC_CTRL  + REG_CLR);

	writel(CTRL_YCBCR422_INPUT | CTRL_INPUT_DATA_SWIZZLE_MASK,
			                               host->base + LCDC_CTRL + REG_CLR);

	switch (host->mpu_lcd_sigs->interface_width)
	{
		case 8:
			writel((0x1 << CTRL1_BYTE_PACKING_FORMAT_OFFSET), 
			         host->base + LCDC_CTRL1 + REG_SET);
			writel(CTRL_WORD_LENGTH_8BIT, 
			         host->base + LCDC_CTRL  + REG_SET);
			break;
		case 16:
			writel((0x3 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
			         host->base + LCDC_CTRL1 + REG_SET);
			writel(CTRL_WORD_LENGTH_16BIT, 
			          host->base + LCDC_CTRL  + REG_SET);
			break;
	}

	tmp = readl(host->base + host->devdata->transfer_count);
	tmp &=     ~(TRANSFER_COUNT_V_COUNT_MASK    | 
	             TRANSFER_COUNT_H_COUNT_MASK);
	tmp |= (1 << TRANSFER_COUNT_V_COUNT_OFFSET) | 
	       (1 << TRANSFER_COUNT_H_COUNT_OFFSET);
	writel(tmp, host->base + host->devdata->transfer_count);

	if(mode == MPU_CMD)
	{
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 0);
		else
            if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
				writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL_DATA_SELECT, host->base + LCDC_CTRL + REG_CLR);
	}
	else
	{
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);
		else
            if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
				writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_DATA_SELECT, host->base + LCDC_CTRL + REG_SET);
	}

	if(rw == MPU_READ)
	{
		writel(CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_MASK,
			    	host->base + LCDC_V4_CTRL2 + REG_CLR);
		writel((0x1 << CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_OFFSET),
		            host->base + LCDC_V4_CTRL2 + REG_SET);

		writel(CTRL_READ_WRITEB, 
		            host->base + LCDC_CTRL + REG_SET);
		writel(CTRL_RUN,
		             host->base + LCDC_CTRL + REG_SET);
	}
	else
	{
		writel(CTRL_READ_WRITEB,
		             host->base + LCDC_CTRL + REG_CLR);
		writel(CTRL_RUN,                
		             host->base + LCDC_CTRL + REG_SET);

		writel(data,   
		             host->base + host->devdata->data);
	}

	tmp = readl(host->base + LCDC_CTRL);
	while(tmp & CTRL_RUN)
	{
		if(rw == MPU_READ)
			ret = readl(host->base + host->devdata->data);

		tmp = readl(host->base + LCDC_CTRL);
	}

	writel(CTRL_MASTER,
	             host->base + LCDC_CTRL + REG_SET);

	writel(CTRL_WORD_LENGTH_MASK,
	             host->base + LCDC_CTRL + REG_CLR);
	writel((wordlen << CTRL_WORD_LENGTH_OFFSET), 
	             host->base + LCDC_CTRL + REG_SET);  // xx bits valid output data
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK,	
	             host->base + LCDC_CTRL1 + REG_CLR);
	writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET), 
	             host->base + LCDC_CTRL1 + REG_SET);  // 32 bits valid input data

	writel(CTRL_MASTER, 
	             host->base + LCDC_CTRL + REG_SET);

	// For idle, set LCD_RS to high
	if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
		gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);
	else
        if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
			writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_SET);
	writel(CTRL_DATA_SELECT, host->base + LCDC_CTRL + REG_SET);

	return ret;
}
/*
 * Dump the CPU LCD controller
 */
void dump_setup(struct mxsfb_info *host)
{
	unsigned int val;
	val = readl(host->base + LCDC_CTRL);
    pr_err("LCDC_CTRL=%08x\n",val);
	val = readl(host->base + LCDC_CTRL1);
    pr_err("LCDC_CTRL1=%08x\n",val);
	val = readl(host->base + LCDC_V4_CTRL2);
    pr_err("LCDC_V4_CTRL2=%08x\n",val);
	val = readl(host->base + LCDC_V4_TRANSFER_COUNT);
    pr_err("LCDC_V4_TRANSFER_COUNT=%08x\n",val);
	val = readl(host->base + LCDC_V4_CUR_BUF);
    pr_err("LCDC_V4_CUR_BUF=%08x\n",val);
	val = readl(host->base + LCDC_V4_NEXT_BUF);
    pr_err("LCDC_V4_NEXT_BUF=%08x\n",val);
	val = readl(host->base + LCDC_TIMING);
    pr_err("LCDC_TIMING=%08x\n",val);
	val = readl(host->base + LCDC_VDCTRL0);
    pr_err("LCDC_VDCTRL0=%08x\n",val);
	val = readl(host->base + LCDC_VDCTRL1);
    pr_err("LCDC_VDCTRL1=%08x\n",val);
	val = readl(host->base + LCDC_VDCTRL2);
    pr_err("LCDC_VDCTRL2=%08x\n",val);
	val = readl(host->base + LCDC_VDCTRL3);
    pr_err("LCDC_VDCTRL3=%08x\n",val);
	val = readl(host->base + LCDC_VDCTRL4);
    pr_err("LCDC_VDCTRL4=%08x\n",val);
	val = readl(host->base + LCDC_DVICTRL0);
    pr_err("LCDC_DVICTRL0=%08x\n",val);
	val = readl(host->base + LCDC_DVICTRL1);
    pr_err("LCDC_DVICTRL1=%08x\n",val);
	val = readl(host->base + LCDC_DVICTRL2);
    pr_err("LCDC_DVICTRL2=%08x\n",val);
	val = readl(host->base + LCDC_DVICTRL3);
    pr_err("LCDC_DVICTRL3=%08x\n",val);
	val = readl(host->base + LCDC_DVICTRL4);
    pr_err("LCDC_DVICTRL4=%08x\n",val);
	val = readl(host->base + LCDC_V4_DATA);
    pr_err("LCDC_V4_DATA=%08x\n",val);
	val = readl(host->base + LCDC_V4_DEBUG0);
    pr_err("LCDC_V4_DEBUG0=%08x\n",val);
}
/*
 * Decode the two most importent registers in the CPU LCD controler
 */
void decode_setup(struct mxsfb_info *host)
{
	unsigned int val;
	val = readl(host->base + LCDC_CTRL);
    pr_err("%s LCDC_CTRL=%x\n",__func__,val);
    pr_err("CTRL_SFTRST=%x\n",(val & CTRL_SFTRST)!=0);
//#define CTRL_SFTRST			(1 << 31)
    pr_err("CTRL_CLKGATE=%x\n",(val & CTRL_CLKGATE)!=0);
//#define CTRL_CLKGATE			(1 << 30)
    pr_err("CTRL_YCBCR422_INPUT=%x\n",(val & CTRL_YCBCR422_INPUT)!=0);
//#define CTRL_YCBCR422_INPUT	(1 << 29)
    pr_err("CTRL_READ_WRITEB=%x\n",(val & CTRL_READ_WRITEB)!=0);
//#define CTRL_READ_WRITEB		(1 << 28)
    pr_err("CTRL_WAIT_FOR_VSYNC_EDGE=%x\n",(val & CTRL_WAIT_FOR_VSYNC_EDGE)!=0);
//#define CTRL_WAIT_FOR_VSYNC_EDGE	(1 << 27)
    pr_err("CTRL_DATA_SHIFT_DIR=%x\n",(val & CTRL_DATA_SHIFT_DIR)!=0);
//#define CTRL_DATA_SHIFT_DIR	(1 << 26)
    pr_err("CTRL_SHIFT_NUM_BITS=%x\n",(val & CTRL_SHIFT_NUM_BITS_MASK)>>CTRL_SHIFT_NUM_BITS_OFFSET);
//#define CTRL_SHIFT_NUM_BITS_MASK	(0x1f << 21)
//#define CTRL_SHIFT_NUM_BITS_OFFSET	21
    pr_err("CTRL_DVI_MODE=%x\n",(val & CTRL_DVI_MODE)!=0);
//#define CTRL_DVI_MODE					(1 << 20)
    pr_err("CTRL_BYPASS_COUNT=%x\n",(val & CTRL_BYPASS_COUNT)!=0);
//#define CTRL_BYPASS_COUNT		(1 << 19)
    pr_err("CTRL_VSYNC_MODE=%x\n",(val & CTRL_VSYNC_MODE)!=0);
//#define CTRL_VSYNC_MODE			(1 << 18)
    pr_err("CTRL_DOTCLK_MODE=%x\n",(val & CTRL_DOTCLK_MODE)!=0);
//#define CTRL_DOTCLK_MODE		(1 << 17)
    pr_err("CTRL_DATA_SELECT=%x\n",(val & CTRL_DATA_SELECT)!=0);
//#define CTRL_DATA_SELECT		(1 << 16)
    pr_err("CTRL_INPUT_DATA_SWIZZLE=%x\n",(val & CTRL_INPUT_DATA_SWIZZLE_MASK)>>CTRL_INPUT_DATA_SWIZZLE_OFFSET);
//#define CTRL_INPUT_DATA_SWIZZLE_MASK			(0x3 << 14)
//#define CTRL_INPUT_DATA_SWIZZLE_OFFSET			14
    pr_err("TRL_CSC_DATA_SWIZZLE=%x\n",(val & CTRL_CSC_DATA_SWIZZLE_MASK)>>CTRL_CSC_DATA_SWIZZLE_OFFSET);
//#define CTRL_CSC_DATA_SWIZZLE_MASK			(0x3 << 12)
//#define CTRL_CSC_DATA_SWIZZLE_OFFSET			12
    pr_err("CTRL_LCD_DATABUS_WIDTH=%x\n",CTRL_GET_BUS_WIDTH(val));
//#define CTRL_LCD_DATABUS_WIDTH_MASK			(0x3 << 10)
//#define CTRL_LCD_DATABUS_WIDTH_OFFSET			10
//#define CTRL_LCD_DATABUS_WIDTH_16BIT			(0 << 10)
//#define CTRL_LCD_DATABUS_WIDTH_8BIT			(1 << 10)
//#define CTRL_LCD_DATABUS_WIDTH_18BIT			(2 << 10)
//#define CTRL_LCD_DATABUS_WIDTH_24BIT			(3 << 10)
//#define CTRL_SET_BUS_WIDTH(x)		(((x) & 0x3) << 10)
//#define CTRL_GET_BUS_WIDTH(x)		(((x) >> 10) & 0x3)
    pr_err("CTRL_WORD_LENGTH=%x\n",CTRL_GET_WORD_LENGTH(val));
//#define CTRL_WORD_LENGTH_MASK				(0x3 << 8)
//#define CTRL_WORD_LENGTH_OFFSET				8
//#define CTRL_WORD_LENGTH_16BIT				(0 << 8)
//#define CTRL_WORD_LENGTH_8BIT				(1 << 8)
//#define CTRL_WORD_LENGTH_18BIT				(2 << 8)
//#define CTRL_WORD_LENGTH_24BIT				(3 << 8)
//#define CTRL_SET_WORD_LENGTH(x)		(((x) & 0x3) << 8)
//#define CTRL_GET_WORD_LENGTH(x)		(((x) >> 8) & 0x3)
    pr_err("CTRL_RGB_TO_YCBCR422_CSC=%x\n",(val & CTRL_RGB_TO_YCBCR422_CSC)!=0);
//#define CTRL_RGB_TO_YCBCR422_CSC				(1 << 7)
//#define CTRL_MASTER			(1 << 5)
    pr_err("CTRL_MASTER=%x\n",(val & CTRL_MASTER)!=0);
//#define CTRL_DATA_FORMAT_16_BIT				(1 << 3)
    pr_err("CTRL_DATA_FORMAT_16_BIT=%x\n",(val & CTRL_DATA_FORMAT_16_BIT)!=0);
//#define CTRL_DATA_FORMAT_18_BIT				(1 << 2)
    pr_err("CTRL_DATA_FORMAT_18_BIT=%x\n",(val & CTRL_DATA_FORMAT_18_BIT)!=0);
//#define CTRL_DATA_FORMAT_24_BIT				(1 << 1)
    pr_err("CTRL_DATA_FORMAT_24_BIT=%x\n",(val & CTRL_DATA_FORMAT_24_BIT)!=0);
//#define CTRL_RUN			(1 << 0)
    pr_err("CTRL_RUN=%x\n",(val & CTRL_RUN)!=0);

	val = readl(host->base + LCDC_CTRL1);
    pr_err("%s LCDC_CTRL1=%x\n",__func__,val);
//#define CTRL1_COMBINE_MPU_WR_STRB				(1 << 27)
    pr_err("CTRL1_COMBINE_MPU_WR_STRB=%x\n",(val & CTRL1_COMBINE_MPU_WR_STRB)!=0);
//#define CTRL1_BM_ERROR_IRQ_EN				(1 << 26)
    pr_err("CTRL1_BM_ERROR_IRQ_EN=%x\n",(val & CTRL1_BM_ERROR_IRQ_EN)!=0);
//#define CTRL1_BM_ERROR_IRQ				(1 << 25)
    pr_err("CTRL1_BM_ERROR_IRQ=%x\n",(val & CTRL1_BM_ERROR_IRQ)!=0);
//#define CTRL1_RECOVERY_ON_UNDERFLOW		(1 << 24)
    pr_err("CTRL1_RECOVERY_ON_UNDERFLOW=%x\n",(val & CTRL1_RECOVERY_ON_UNDERFLOW)!=0);
//#define CTRL1_INTERLACE_FIELDS				(1 << 23)
    pr_err("CTRL1_INTERLACE_FIELDS=%x\n",(val & CTRL1_INTERLACE_FIELDS)!=0);
//#define CTRL1_START_INTERLACE_FROM_SECOND_FIELD		(1 << 22)
    pr_err("CTRL1_START_INTERLACE_FROM_SECOND_FIELD=%x\n",(val & CTRL1_START_INTERLACE_FROM_SECOND_FIELD)!=0);
//#define CTRL1_FIFO_CLEAR				(1 << 21)
    pr_err("CTRL1_FIFO_CLEAR=%x\n",(val & CTRL1_FIFO_CLEAR)!=0);
//#define CTRL1_IRQ_ON_ALTERNATE_FIELDS			(1 << 20)
    pr_err("CTRL1_IRQ_ON_ALTERNATE_FIELDS=%x\n",(val & CTRL1_IRQ_ON_ALTERNATE_FIELDS)!=0);
//#define CTRL1_BYTE_PACKING_FORMAT_MASK			(0xf << 16)
//#define CTRL1_BYTE_PACKING_FORMAT_OFFSET			16
//#define CTRL1_SET_BYTE_PACKAGING(x)		(((x) & 0xf) << 16)
//#define CTRL1_GET_BYTE_PACKAGING(x)		(((x) >> 16) & 0xf)
     pr_err("CTRL1_BYTE_PACKING_FORMAT=%x\n",CTRL1_GET_BYTE_PACKAGING(val));
//#define CTRL1_OVERFLOW_IRQ_EN			(1 << 15)
    pr_err("CTRL1_OVERFLOW_IRQ_EN=%x\n",(val & CTRL1_OVERFLOW_IRQ_EN)!=0);
//#define CTRL1_UNDERFLOW_IRQ_EN			(1 << 14)
    pr_err("CTRL1_UNDERFLOW_IRQ_EN=%x\n",(val & CTRL1_UNDERFLOW_IRQ_EN)!=0);
//#define CTRL1_CUR_FRAME_DONE_IRQ_EN		(1 << 13)
    pr_err("CTRL1_CUR_FRAME_DONE_IRQ_EN=%x\n",(val & CTRL1_CUR_FRAME_DONE_IRQ_EN)!=0);
//#define CTRL1_VSYNC_EDGE_IRQ_EN			(1 << 12)
    pr_err("CTRL1_VSYNC_EDGE_IRQ_EN=%x\n",(val & CTRL1_VSYNC_EDGE_IRQ_EN)!=0);
//#define CTRL1_OVERFLOW_IRQ				(1 << 11)
    pr_err("CTRL1_OVERFLOW_IRQ=%x\n",(val & CTRL1_OVERFLOW_IRQ)!=0);
//#define CTRL1_UNDERFLOW_IRQ				(1 << 10)
    pr_err("CTRL1_UNDERFLOW_IRQ=%x\n",(val & CTRL1_UNDERFLOW_IRQ)!=0);
//#define CTRL1_CUR_FRAME_DONE_IRQ		(1 << 9)
    pr_err("CTRL1_CUR_FRAME_DONE_IRQ=%x\n",(val & CTRL1_CUR_FRAME_DONE_IRQ)!=0);
//#define CTRL1_VSYNC_EDGE_IRQ			(1 << 8)
    pr_err("CTRL1_VSYNC_EDGE_IRQ=%x\n",(val & CTRL1_VSYNC_EDGE_IRQ)!=0);
//#define CTRL1_BUSY_ENABLE					(1 << 2)
    pr_err("CTRL1_BUSY_ENABLE=%x\n",(val & CTRL1_BUSY_ENABLE)!=0);
//#define CTRL1_MODE86					(1 << 1)
    pr_err("CTRL1_MODE86=%x\n",(val & CTRL1_MODE86)!=0);
//#define CTRL1_RESET					(1 << 0)
    pr_err("CTRL1_RESET=%x\n",(val & CTRL1_RESET)!=0);
}

/*
Register-Content Interface Mode
mode   MPU_DATA, Set index, (read value)/(write data) of length
	   MPU_CMD,  Set index, rw = MPU_WRITE
	             Read status, rw = MPU_READ
rw     MPU_WRITE,MPU_READ
index  value to write to index register
data data to be written to register pointed by indexregister
length number of bytes in data

Returns -1 on error
         0 on MPU_WRITE
		 read value 8-16 bit unsigned  on MPU_READ
*/
int mxsfb_mpu_access_rcim(
	struct mxsfb_info *host,
	int mode,
	int rw,
	unsigned int index,
	int data,
	unsigned int length)
{
	unsigned int tmp, wordlen, ret = 0,return_value=0;
	unsigned int remaining_bytes=length;

	if (mxsfb_mpu_wait_for_ready(host) != 0)
	{
	    pr_err("mxsfb_mpu_wait_for_ready failed, in mxsfb_mpu_access_rcim\n");
		return -1;
	}
	writel(CTRL_MASTER,	                   host->base + LCDC_CTRL  + REG_CLR);
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK, host->base + LCDC_CTRL1 + REG_CLR);

	tmp = readl(host->base + LCDC_CTRL);
	wordlen = CTRL_GET_WORD_LENGTH(tmp);
	writel(CTRL_WORD_LENGTH_MASK,        host->base + LCDC_CTRL  + REG_CLR);

	writel(CTRL_YCBCR422_INPUT | CTRL_INPUT_DATA_SWIZZLE_MASK,
			                             host->base + LCDC_CTRL + REG_CLR);

	// Set command interface width
    writel(CTRL_LCD_DATABUS_WIDTH_8BIT,	 host->base + LCDC_CTRL + REG_SET);
	writel((0x1 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
	                                     host->base + LCDC_CTRL1 + REG_SET);
	writel(CTRL_WORD_LENGTH_8BIT,        host->base + LCDC_CTRL  + REG_SET);
	writel(CTRL_DATA_FORMAT_24_BIT,      host->base + LCDC_CTRL  + REG_CLR);

	tmp = (1 << TRANSFER_COUNT_V_COUNT_OFFSET) |
	      (1 << TRANSFER_COUNT_H_COUNT_OFFSET);
	writel(tmp, host->base + host->devdata->transfer_count); //tranfer 1 byte
	while (remaining_bytes--)
	{
		/* set RS (Register select) low*/
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 0);
		else
		    if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
				writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL_DATA_SELECT, host->base + LCDC_CTRL + REG_CLR);
		
		if((rw == MPU_READ) && (mode == MPU_CMD))
		{//Read status, not described what kind of status
			writel(CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_MASK,
			               	    	 host->base + LCDC_V4_CTRL2 + REG_CLR);
			writel((0x1 << CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_OFFSET),
			                         host->base + LCDC_V4_CTRL2 + REG_SET);
			writel(CTRL_READ_WRITEB, host->base + LCDC_CTRL + REG_SET);
			writel(CTRL_RUN,         host->base + LCDC_CTRL + REG_SET);
		}
		else
		{// send index
			writel(CTRL_READ_WRITEB,   host->base + LCDC_CTRL + REG_CLR);
			writel(CTRL_RUN,           host->base + LCDC_CTRL + REG_SET);
			writel(index,              host->base + host->devdata->data);
		}
		//wait for LCD controller to finish
		tmp = readl(host->base + LCDC_CTRL);
		while(tmp & CTRL_RUN)
		{
            ret = readl(host->base + host->devdata->data); //dummy read
			tmp = readl(host->base + LCDC_CTRL);
		}
        if((rw == MPU_READ) && (mode == MPU_CMD))
            return_value = readl(host->base + host->devdata->data); //Get the status
	    /* set RS (Register select) high*/
		if (host->mpu_lcd_sigs->lcd_rs_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio, 1);
		else
		    if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
				writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_DATA_SELECT, host->base + LCDC_CTRL + REG_SET);

		if(mode == MPU_DATA)
		{
			if(rw == MPU_READ)
			{
				writel(CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_MASK,
				                     	 host->base + LCDC_V4_CTRL2 + REG_CLR);
				writel((0x1 << CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_OFFSET),
				                         host->base + LCDC_V4_CTRL2 + REG_SET);
				writel(CTRL_READ_WRITEB,
				                         host->base + LCDC_CTRL + REG_SET);
				writel(CTRL_RUN,
				                         host->base + LCDC_CTRL + REG_SET);
			}
			else
			{// send data
	//		writel(CTRL_READ_WRITEB,   host->base + LCDC_CTRL + REG_CLR);
				writel(CTRL_RUN,         host->base + LCDC_CTRL + REG_SET);
				writel((data>>(remaining_bytes<<3)) & 0xFF,
				                         host->base + host->devdata->data);
			}
			//wait for LCD controller to finish
			tmp = readl(host->base + LCDC_CTRL);
			while(tmp & CTRL_RUN)
			{
                ret = readl(host->base + host->devdata->data); //dummy read
                tmp = readl(host->base + LCDC_CTRL);
			}
            if(rw == MPU_READ)
            { 
                ret = readl(host->base + host->devdata->data); //Get the data
			    return_value = (return_value << 8) | (ret & 0xFF);
            }
			index++;
		}
	}

	/* Configure the output bus width */
	writel(CTRL_LCD_DATABUS_WIDTH_MASK,	     host->base + LCDC_CTRL + REG_CLR);
	switch (host->mpu_lcd_sigs->interface_width) {
	case 8:
		writel(CTRL_LCD_DATABUS_WIDTH_8BIT,	 host->base + LCDC_CTRL + REG_SET);
		break;
	case 16:
		writel(CTRL_LCD_DATABUS_WIDTH_16BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	case 18:
		writel(CTRL_LCD_DATABUS_WIDTH_18BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	case 24:
	default:
		writel(CTRL_LCD_DATABUS_WIDTH_24BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	}
	writel(CTRL_MASTER,                      host->base + LCDC_CTRL  + REG_SET);

	writel(CTRL_WORD_LENGTH_MASK,			 host->base + LCDC_CTRL  + REG_CLR);
	  // reestablish xx bits valid output data
	writel((wordlen << CTRL_WORD_LENGTH_OFFSET),
	                                         host->base + LCDC_CTRL  + REG_SET);
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK,
	                             		     host->base + LCDC_CTRL1 + REG_CLR);
      // 32 input data but only 24 bits valid
	writel((0x7 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
	                                         host->base + LCDC_CTRL1 + REG_SET);
	writel(CTRL_MASTER,                      host->base + LCDC_CTRL  + REG_SET);

	return (int)return_value;
}

int mxsfb_mpu_refresh_panel(struct fb_info *info)
{
    struct mxsfb_info *host=info->par;
#if 1
    static int i=0;
    unsigned int tmp;
    struct timespec ts;
    static time_t last_check_time=0;
#if 0
    static time_t last_fail=0;
#endif
    tmp = *((unsigned int *)host->fb_info->screen_base);
    getnstimeofday(&ts);
#if 1
    //test for "white" flash, don't show that frame
    if ((tmp & 0x00FFFFFF) == 0x00BEBEBE)
    {
//        printk("%d.%09d pixel 0 = %08x\n",ts.tv_sec,ts.tv_nsec,tmp);
#if 0
        //show the "white" frame, and hold it for 10 seconds
      	if (host->enabled)
        {
            last_fail=ts.tv_sec;
            if (mxsfb_mpu_wait_for_ready(host) != 0)
            {
                pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
                return -ETIME;
            }
            dmac_flush_range((void *)host->fb_info->screen_base, 
            (void *)host->fb_info->screen_base + host->fb_info->fix.smem_len);
            writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
            writel(CTRL_RUN,    host->base + LCDC_CTRL + REG_SET);
        }
        return 0;
    }
    if (ts.tv_sec < last_fail+10)
        return 0;
#else
        return 0;
    }
#endif
#endif
    i++;
#endif
	if (host->enabled)
	{
		if (mxsfb_mpu_wait_for_ready(host) != 0)
		{
			pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
			return -ETIME;
		}
#if 1
        //one time pr. second, test if display controller is alive, 
        //the reading and setup after, is normaly what is needed to recover the controller
        //if it has been knocked out by ESD
        //we have never seen the power cycle in action
		if ((!host->inhibit_updates) && (last_check_time != ts.tv_sec))
        {
            last_check_time=ts.tv_sec;
            if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ, 0x0C, 0x00, 2)))
            {
                pr_err("1 display alive test failed\n");
            }
            if (tmp!=0x13F)
            {
                pr_err("display alive test i=%d tmp=%x not 0x13F\n",i,tmp);
                if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ, 0x0C, 0x00, 2)))
                {
                    pr_err("2 display alive test failed\n");
                }
            }
            if (-1 == mxsfb_mpu_access_rcim(host,MPU_CMD,MPU_WRITE,0x22, 0x00, 1))
            {
                pr_err("3 display alive test failed\n");
            }
            mxsfb_mpu_setup_refresh_data(host);
//            pr_err("display alive test i=%d tmp=%x\n",i,tmp);
            if (tmp != 0x13F)
            {
                pr_err("mxsfb_mpu_refresh_panel power-cycle display i=%d\n",i);
                gpiod_set_value(host->mpu_lcd_sigs->lcd_power_gpio, 0);
                msleep(100);
                gpiod_set_value(host->mpu_lcd_sigs->lcd_power_gpio, 1);
                msleep(100);
                host->mpu_lcd_functions->mpu_lcd_setup(host);
            }
        }
#endif
 		dmac_flush_range((void *)host->fb_info->screen_base, 
           (void *)host->fb_info->screen_base + host->fb_info->fix.smem_len);

		if (!host->inhibit_updates) {
			writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
			writel(CTRL_RUN,    host->base + LCDC_CTRL + REG_SET);
		}
	}
	return 0;
}

void mxsfb_mpu_setup_refresh_data(struct mxsfb_info *host)
{
	unsigned int tmp;

	if (mxsfb_mpu_wait_for_ready(host) != 0)
	{
	    pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
		return;
	}

	tmp = readl(host->base + host->devdata->transfer_count);
	tmp &= ~(TRANSFER_COUNT_H_COUNT_MASK |
		 TRANSFER_COUNT_V_COUNT_MASK);
    if (host->var.xres != 0)
		tmp |= (host->var.xres << TRANSFER_COUNT_H_COUNT_OFFSET) |
			   (host->var.yres << TRANSFER_COUNT_V_COUNT_OFFSET);
	else
		tmp |= (host->fb_info->var.xres << TRANSFER_COUNT_H_COUNT_OFFSET) |
			   (host->fb_info->var.yres << TRANSFER_COUNT_V_COUNT_OFFSET);
	writel(tmp, host->base + host->devdata->transfer_count);

	writel(CTRL_READ_WRITEB,  host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_BYPASS_COUNT, host->base + LCDC_CTRL + REG_CLR);

	if (host->mpu_lcd_sigs->panel_bpp == 16) {
		writel(CTRL_YCBCR422_INPUT |
				(1 << CTRL_INPUT_DATA_SWIZZLE_OFFSET),
				host->base + LCDC_CTRL + REG_SET);
	}
}

static void mxsfb_mpu_setup_interface(struct mxsfb_info *host)
{
  	writel(CTRL_RUN,    host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);

	/* Setup the bus protocol */
	if (host->mpu_lcd_sigs->bus_mode == MPU_BUS_8080)
		writel(CTRL1_MODE86, host->base + LCDC_CTRL1 + REG_CLR);
	else
		writel(CTRL1_MODE86, host->base + LCDC_CTRL1 + REG_SET);

	writel(CTRL1_BUSY_ENABLE, host->base + LCDC_CTRL1 + REG_CLR);
#ifndef LCD_RESET
    if (!FirstReset)
#endif
    {
		/* Take display out of reset */
		if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 1);
		else
			writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_SET);
		msleep(10);
    }
	/* VSYNC is an input by default */
	writel(VDCTRL0_VSYNC_OEB, host->base + LCDC_VDCTRL0 + REG_SET);
#ifndef LCD_RESET
    if (!FirstReset)
#endif
    {
		/*
		 * Make sure we do a high-to-low transition to reset the panel.
		 * First make it low for 10 msec, hi for 120 msec
		 */
		if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 0);
        else
			writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_CLR);	/* low */

		msleep(10);

		if (host->mpu_lcd_sigs->lcd_reset_is_gpio)
			gpiod_set_value(host->mpu_lcd_sigs->lcd_reset_gpio, 1);
        else
			writel(CTRL1_RESET, host->base + LCDC_CTRL1 + REG_SET);	/* high */

		msleep(120);
    }

	writel(CTRL_DATA_SHIFT_DIR,          host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL_SHIFT_NUM_BITS_MASK,     host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL2_OUTSTANDING_REQS_MASK,  host->base + LCDC_V4_CTRL2 + REG_CLR);
	writel(CTRL2_OUTSTANDING_REQS_REQ_8, host->base + LCDC_V4_CTRL2 + REG_SET);

	/* Recover on underflow */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW,  host->base + LCDC_CTRL1 + REG_SET);

	/* Configure the input pixel format */
	writel(CTRL_YCBCR422_INPUT |
		   CTRL_WORD_LENGTH_MASK |
		   CTRL_INPUT_DATA_SWIZZLE_MASK |
		   CTRL_DATA_FORMAT_16_BIT |
		   CTRL_DATA_FORMAT_18_BIT |
		   CTRL_DATA_FORMAT_24_BIT,
		     host->base + LCDC_CTRL + REG_CLR);
	writel(CTRL1_BYTE_PACKING_FORMAT_MASK, host->base + LCDC_CTRL1 + REG_CLR);
	switch (host->mpu_lcd_sigs->panel_bpp) {
	case 16:
		writel((0xF << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
		                           host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_WORD_LENGTH_16BIT |(0 << CTRL_INPUT_DATA_SWIZZLE_OFFSET), 
		                           host->base + LCDC_CTRL  + REG_SET);
		break;
	case 18:
	case 24:
	default:
		writel((0x7 << CTRL1_BYTE_PACKING_FORMAT_OFFSET),
		                           host->base + LCDC_CTRL1 + REG_SET);
		writel(CTRL_WORD_LENGTH_24BIT|(0 << CTRL_INPUT_DATA_SWIZZLE_OFFSET),
		                           host->base + LCDC_CTRL  + REG_SET);
		break;
	}

	/* Configure the output bus width */
	writel(CTRL_LCD_DATABUS_WIDTH_MASK,	     host->base + LCDC_CTRL + REG_CLR);
	switch (host->mpu_lcd_sigs->interface_width) {
	case 8:
		writel(CTRL_LCD_DATABUS_WIDTH_8BIT,	 host->base + LCDC_CTRL + REG_SET);
		break;
	case 16:
		writel(CTRL_LCD_DATABUS_WIDTH_16BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	case 18:
		writel(CTRL_LCD_DATABUS_WIDTH_18BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	case 24:
	default:
		writel(CTRL_LCD_DATABUS_WIDTH_24BIT, host->base + LCDC_CTRL + REG_SET);
		break;
	}

	/* Configure the MPU timing */
	writel((1 << TIMING_CMD_HOLD_OFFSET)  | (1 << TIMING_CMD_SETUP_OFFSET) |
		   (1 << TIMING_DATA_HOLD_OFFSET) | (1 << TIMING_DATA_SETUP_OFFSET),
			 host->base + LCDC_TIMING);

	msleep(10);
}

static irqreturn_t mxsfb_irq_handler(int irq, void *dev_id)
{
	struct mxsfb_info *host = dev_id;
	u32 ctrl1, enable, status, acked_status;
 
	ctrl1 = readl(host->base + LCDC_CTRL1);
	enable = (ctrl1 & CTRL1_IRQ_ENABLE_MASK) >> CTRL1_IRQ_ENABLE_SHIFT;
	status = (ctrl1 & CTRL1_IRQ_STATUS_MASK) >> CTRL1_IRQ_STATUS_SHIFT;
	acked_status = (enable & status) << CTRL1_IRQ_STATUS_SHIFT;

	if ((acked_status & CTRL1_VSYNC_EDGE_IRQ) && host->wait4vsync) {
		writel(CTRL1_VSYNC_EDGE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_VSYNC_EDGE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		host->wait4vsync = 0;
		complete(&host->vsync_complete);
	}

	if (acked_status & CTRL1_CUR_FRAME_DONE_IRQ) {
		writel(CTRL1_CUR_FRAME_DONE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		complete(&host->flip_complete);
	}

	if (acked_status & CTRL1_UNDERFLOW_IRQ)
		writel(CTRL1_UNDERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	if (acked_status & CTRL1_OVERFLOW_IRQ)
		writel(CTRL1_OVERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	return IRQ_HANDLED;
}

static int mxsfb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	const struct fb_bitfield *rgb = NULL;
 
	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;

	if (var->xres_virtual > var->xres) {
		dev_dbg(fb_info->device, "stride not supported\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;


	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 16) )
		var->bits_per_pixel = 32;

	switch (var->bits_per_pixel) {
	case 16:
		/* always expect RGB 565 */
		rgb = def_rgb565;
		break;
	case 18:
		/* 24 bit to 18 bit mapping */
		rgb = def_rgb666;
		break;
	case 32:
		if (host->is_mpu_lcd) {
			rgb = def_rgb888;
			break;
		}

		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			pr_err("Unsupported LCD bus width mapping\n");
			return -EINVAL;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			rgb = def_rgb666;
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(var, def_rgb666))
			{
				/* 24 bit to 18 bit mapping */
				rgb = def_rgb666;
			}
			else
				rgb = def_rgb888;
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			rgb = def_rgb888;
			break;
		}
		break;
	default:
		pr_err("Unsupported colour depth: %u\n", var->bits_per_pixel);
		return -EINVAL;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = rgb[RED];
	var->green  = rgb[GREEN];
	var->blue   = rgb[BLUE];
	var->transp = rgb[TRANSP];

	return 0;
}

static void mxsfb_enable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 reg;
	int ret;
 
	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	if (host->dispdrv && host->dispdrv->drv->setup) {
		ret = host->dispdrv->drv->setup(host->dispdrv, fb_info);
		if (ret < 0) {
			dev_err(&host->pdev->dev, "failed to setup"
				"dispdrv:%s\n", host->dispdrv->drv->name);
			return;
		}
		host->sync = fb_info->var.sync;
	}

	if (host->reg_lcd) {
		ret = regulator_enable(host->reg_lcd);
		if (ret) {
			dev_err(&host->pdev->dev,
				"lcd regulator enable failed:	%d\n", ret);
			return;
		}
	}

	/* the pixel clock should be disabled before
	 * trying to set its clock rate successfully.
	 */
	clk_disable_pix(host);
	ret = clk_set_rate(host->clk_pix, PICOS2KHZ(fb_info->var.pixclock) * 1000U);
	if (ret) {
		dev_err(&host->pdev->dev,
			"lcd pixel rate set failed: %d\n", ret);

		if (host->reg_lcd) {
			ret = regulator_disable(host->reg_lcd);
			if (ret)
				dev_err(&host->pdev->dev,
					"lcd regulator disable failed: %d\n",
					ret);
		}
		return;
	}
	clk_enable_pix(host);

	if (!host->is_mpu_lcd) {
		writel(CTRL2_OUTSTANDING_REQS_REQ_16,
			host->base + LCDC_V4_CTRL2 + REG_SET);

		/* if it was disabled, re-enable the mode again */
		writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_SET);

		/* enable the SYNC signals first, then the DMA engine */
		reg = readl(host->base + LCDC_VDCTRL4);
		reg |= VDCTRL4_SYNC_SIGNALS_ON;
		writel(reg, host->base + LCDC_VDCTRL4);

		writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
		writel(CTRL_RUN, host->base + LCDC_CTRL + REG_SET);

		/* Recovery on underflow */
		writel(CTRL1_RECOVERY_ON_UNDERFLOW, host->base + LCDC_CTRL1 + REG_SET);
	}

	host->enabled = 1;

	if (host->dispdrv && host->dispdrv->drv->enable) {
		ret = host->dispdrv->drv->enable(host->dispdrv, fb_info);
		if (ret < 0)
			dev_err(&host->pdev->dev, "failed to enable "
				"dispdrv:%s\n", host->dispdrv->drv->name);
	}
}

static void mxsfb_disable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	unsigned loop;
	u32 reg;
	int ret;

 	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	if (host->dispdrv && host->dispdrv->drv->disable)
		host->dispdrv->drv->disable(host->dispdrv, fb_info);

	/*
	 * Even if we disable the controller here, it will still continue
	 * until its FIFOs are running out of data
	 */
	writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_CLR);

	loop = 1000;
	while (loop) {
		reg = readl(host->base + LCDC_CTRL);
		if (!(reg & CTRL_RUN))
			break;
		loop--;
	}

	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);

	reg = readl(host->base + LCDC_VDCTRL4);
	writel(reg & ~VDCTRL4_SYNC_SIGNALS_ON, host->base + LCDC_VDCTRL4);

	host->enabled = 0;

	if (host->reg_lcd) {
		ret = regulator_disable(host->reg_lcd);
		if (ret)
			dev_err(&host->pdev->dev,
				"lcd regulator disable failed: %d\n", ret);
	}
}

/**
   This function compare the fb parameter see whether it was different
   parameter for hardware, if it was different parameter, the hardware
   will reinitialize. All will compared except x/y offset.
 */
static bool mxsfb_par_equal(struct fb_info *fbi, struct mxsfb_info *host)
{
	/* Here we set the xoffset, yoffset to zero, and compare two
	 * var see have different or not. */
	struct fb_var_screeninfo oldvar = host->var;
	struct fb_var_screeninfo newvar = fbi->var;
  
	if ((fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW &&
	    fbi->var.activate & FB_ACTIVATE_FORCE)
		return false;

	oldvar.xoffset = newvar.xoffset = 0;
	oldvar.yoffset = newvar.yoffset = 0;

	return memcmp(&oldvar, &newvar, sizeof(struct fb_var_screeninfo)) == 0;
}

static int mxsfb_set_par(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 ctrl, vdctrl0, vdctrl4;
	int line_size, fb_size;
	int reenable = 0;
	static u32 equal_bypass = 0;

	if (host->inhibit_updates)
		return 0;

	if (likely(equal_bypass > 1)) {
		/* If parameter no change, don't reconfigure. */
		if (mxsfb_par_equal(fb_info, host))
			return 0;
	} else
		equal_bypass++;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	/* If fb is in blank mode, it is
	 * unnecessary to really set par here.
	 * It can be delayed when unblank fb
	 */
	if (host->cur_blank != FB_BLANK_UNBLANK)
		return 0;

	line_size =  fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.line_length = line_size;
	fb_size = fb_info->var.yres_virtual * line_size;

	if (fb_size > fb_info->fix.smem_len) {
		dev_err(&host->pdev->dev, "exceeds the fb buffer size limit!\n");
		return -ENOMEM;
	}

	/*
	 * It seems, you can't re-program the controller if it is still running.
	 * This may lead into shifted pictures (FIFO issue?).
	 * So, first stop the controller and drain its FIFOs
	*/
	if (host->enabled && (!host->is_mpu_lcd)) {
		reenable = 1;
		mxsfb_disable_controller(fb_info);
	}

	if (host->is_mpu_lcd) {
		if (host->enabled) {
			mxsfb_mpu_setup_interface(host);
			writel(fb_info->fix.smem_start +
					fb_info->fix.line_length * fb_info->var.yoffset,
					host->base + host->devdata->cur_buf);
			writel(fb_info->fix.smem_start +
					fb_info->fix.line_length * fb_info->var.yoffset,
					host->base + host->devdata->next_buf);
#ifdef LCD_RESET
			if (host->mpu_lcd_functions->mpu_lcd_setup(host))
				return -1;
#else
    		if (!FirstReset)
			{
				if (host->mpu_lcd_functions->mpu_lcd_setup(host))
					return -1;
			}
            else
			{
				// GRAM Write Data, dummy set, to be sure of correct register setup
				if (-1 == mxsfb_mpu_access_rcim(host,MPU_CMD,MPU_WRITE,0x22, 0x00, 1))
				{
					pr_err("dummy startup_sequence failed\n");
				}
			}
#endif
			mxsfb_mpu_setup_refresh_data(host);
#ifndef LCD_RESET
            if (!FirstReset)
#endif
			{
				mxsfb_mpu_refresh_panel(fb_info);
			}
		}
	} else {
		/* clear the FIFOs */
		writel(CTRL1_FIFO_CLEAR, host->base + LCDC_CTRL1 + REG_SET);

		ctrl = CTRL_BYPASS_COUNT | CTRL_MASTER |
			CTRL_SET_BUS_WIDTH(host->ld_intf_width);

		switch (fb_info->var.bits_per_pixel) {
		case 16:
			dev_dbg(&host->pdev->dev, "Setting up RGB565 mode\n");
			ctrl |= CTRL_SET_WORD_LENGTH(0);
			writel(CTRL1_SET_BYTE_PACKAGING(0xf), host->base + LCDC_CTRL1);
			break;
		case 32:
			dev_dbg(&host->pdev->dev, "Setting up RGB888/666 mode\n");
			ctrl |= CTRL_SET_WORD_LENGTH(3);
			switch (host->ld_intf_width) {
			case STMLCDIF_8BIT:
				dev_dbg(&host->pdev->dev,
						"Unsupported LCD bus width mapping\n");
				return -EINVAL;
			case STMLCDIF_16BIT:
				/* 24 bit to 18 bit mapping */
				ctrl |= CTRL_DATA_FORMAT_24_BIT; /* ignore the upper 2 bits in
						                          *  each colour component
						                          */
				break;
			case STMLCDIF_18BIT:
				if (pixfmt_is_equal(&fb_info->var, def_rgb666))
					/* 24 bit to 18 bit mapping */
					ctrl |= CTRL_DATA_FORMAT_24_BIT; /* ignore the upper 2 bits in
							                          *  each colour component
							                          */
				break;
			case STMLCDIF_24BIT:
				/* real 24 bit */
				break;
			}
			/* do not use packed pixels = one pixel per word instead */
			writel(CTRL1_SET_BYTE_PACKAGING(0x7), host->base + LCDC_CTRL1);
			break;
		default:
			dev_dbg(&host->pdev->dev, "Unhandled color depth of %u\n",
					fb_info->var.bits_per_pixel);
			return -EINVAL;
		}

		writel(ctrl, host->base + LCDC_CTRL);

		writel(TRANSFER_COUNT_SET_VCOUNT(fb_info->var.yres) |
				TRANSFER_COUNT_SET_HCOUNT(fb_info->var.xres),
				host->base + host->devdata->transfer_count);

		vdctrl0 = VDCTRL0_ENABLE_PRESENT |	/* always in DOTCLOCK mode */
			VDCTRL0_VSYNC_PERIOD_UNIT |
			VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
			VDCTRL0_SET_VSYNC_PULSE_WIDTH(fb_info->var.vsync_len);
		/* use the saved sync to avoid wrong sync information */
		if (host->sync & FB_SYNC_HOR_HIGH_ACT)
			vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
		if (host->sync & FB_SYNC_VERT_HIGH_ACT)
			vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
		if (!(host->sync & FB_SYNC_OE_LOW_ACT))
			vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
		if (host->sync & FB_SYNC_CLK_LAT_FALL)
			vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

		writel(vdctrl0, host->base + LCDC_VDCTRL0);

		/* frame length in lines */
		writel(fb_info->var.upper_margin + fb_info->var.vsync_len +
			fb_info->var.lower_margin + fb_info->var.yres,
			host->base + LCDC_VDCTRL1);

		/* line length in units of clocks or pixels */
		writel(set_hsync_pulse_width(host, fb_info->var.hsync_len) |
			VDCTRL2_SET_HSYNC_PERIOD(fb_info->var.left_margin +
			fb_info->var.hsync_len + fb_info->var.right_margin +
			fb_info->var.xres),
			host->base + LCDC_VDCTRL2);

		writel(SET_HOR_WAIT_CNT(fb_info->var.left_margin +
			fb_info->var.hsync_len) |
			SET_VERT_WAIT_CNT(fb_info->var.upper_margin +
				fb_info->var.vsync_len),
			host->base + LCDC_VDCTRL3);

		vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(fb_info->var.xres);
		if (mxsfb_is_v4(host))
			vdctrl4 |= VDCTRL4_SET_DOTCLK_DLY(host->dotclk_delay);
		writel(vdctrl4, host->base + LCDC_VDCTRL4);

		writel(fb_info->fix.smem_start +
				fb_info->fix.line_length * fb_info->var.yoffset,
				host->base + host->devdata->next_buf);
	}

	if (reenable && (!host->is_mpu_lcd))
		mxsfb_enable_controller(fb_info);

	/* Clear activate as not Reconfiguring framebuffer again */
	if ((fb_info->var.activate & FB_ACTIVATE_FORCE) &&
		(fb_info->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		fb_info->var.activate = FB_ACTIVATE_NOW;

	host->var = fb_info->var;
	return 0;
}

static int mxsfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int transp, struct fb_info *fb_info)
{
	unsigned int tmp;
	int ret = -EINVAL;
 
	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fb_info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (fb_info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
 		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fb_info->pseudo_palette;
 			tmp  = chan_to_field(red, &fb_info->var.red);
			tmp |= chan_to_field(green, &fb_info->var.green);
			tmp |= chan_to_field(blue, &fb_info->var.blue);

			pal[regno] = tmp;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxsfb_wait_for_vsync(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	int ret = 0;
 
	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(fb_info->device, "can't wait for VSYNC when fb is blank\n");
		return -EINVAL;
	}

	if (host->is_mpu_lcd) {
		if (mxsfb_mpu_wait_for_ready(host) != 0)
		{
			pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
			return -ETIME;
		}
	} else {
		init_completion(&host->vsync_complete);

		host->wait4vsync = 1;
		writel(CTRL1_VSYNC_EDGE_IRQ_EN,
			host->base + LCDC_CTRL1 + REG_SET);
		ret = wait_for_completion_interruptible_timeout(
					&host->vsync_complete, 1 * HZ);
		if (ret == 0) {
			dev_err(fb_info->device,
				"mxs wait for vsync timeout\n");
			host->wait4vsync = 0;
			ret = -ETIME;
		} else if (ret > 0) {
			ret = 0;
		}
	}
	return ret;
}

static int dump_regs(struct fb_info *info)
{
    int i=0;
	unsigned int tmp;
    struct mxsfb_info *host=info->par;

    if (mxsfb_mpu_wait_for_ready(host) != 0)
    {
        pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
		return -ETIME;
    }
    for (i=0x1;i<0x2e;i++)
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %d\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    for (i=0x35;i<0x55;i++)
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %02x\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    for (i=0x64;i<0x68;i++)
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %02x\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    i=0x70;
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %02x\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    i=0x72;
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %02x\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    for (i=0x90;i<0x96;i++)
    {
        if (-1 ==(tmp = mxsfb_mpu_access_rcim(host,MPU_DATA, MPU_READ,i, 0x00, 1)))
        {
            pr_err("1 dump_regs failed reading register %02x\n",i);
        }
        else
            pr_err("display reg[%02x]=%02x\n",i,tmp);
    }

    if (-1 == mxsfb_mpu_access_rcim(host,MPU_CMD,MPU_WRITE,0x22, 0x00, 1))
    {
        pr_err("dump_regs write failed\n");
    }
 
    mxsfb_mpu_setup_refresh_data(host);
    return 0;

}

static int mxsfb_ioctl(struct fb_info *fb_info, unsigned int cmd, unsigned long arg)
{
	struct mxsfb_info *host = fb_info->par;
	int ret = -EINVAL;
 
	switch (cmd) {
	case MXCFB_WAIT_FOR_VSYNC:
		ret = mxsfb_wait_for_vsync(fb_info);
		break;
	case MXCFB_MPU_REFRESH_PANEL:
		ret = mxsfb_mpu_refresh_panel(fb_info);
		break;
    case MXCFB_MPU_DUMP_REGS:
        host->inhibit_updates = 1;
        msleep(20);
		ret = dump_regs(fb_info);
        host->inhibit_updates = 0;
        break;
	default:
		break;
	}
	return ret;
}

static int mxsfb_blank(int blank, struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;

	if (host->inhibit_updates)
		return 0;

	host->cur_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
  		if (host->enabled) {
			if (host->is_mpu_lcd)
				host->mpu_lcd_functions->mpu_lcd_poweroff(host);
			mxsfb_disable_controller(fb_info);
			pm_runtime_put_sync_suspend(&host->pdev->dev);
		}

		clk_disable_disp_axi(host);
		clk_disable_axi(host);
		clk_disable_pix(host);
		break;

	case FB_BLANK_UNBLANK:
 		fb_info->var.activate = (fb_info->var.activate & ~FB_ACTIVATE_MASK) |
				FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;

		clk_enable_pix(host);
		clk_enable_axi(host);
		clk_enable_disp_axi(host);

		if (!host->enabled) {
			pm_runtime_get_sync(&host->pdev->dev);

			writel(0, host->base + LCDC_CTRL);
			if (host->is_mpu_lcd) {
				mxsfb_enable_controller(fb_info);
				mxsfb_set_par(host->fb_info);
			} else {
				mxsfb_set_par(host->fb_info);
				mxsfb_enable_controller(fb_info);
			}
		}
		break;
	}
	return 0;
}

static int mxsfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb_info)
{
	int ret = 0;
	struct mxsfb_info *host = fb_info->par;
	unsigned int offset;

	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_dbg(fb_info->device, "can't do pan display when fb is blank\n");
		return -EINVAL;
	}

	if (var->xoffset > 0) {
		dev_dbg(fb_info->device, "x panning not supported\n");
		dev_info(fb_info->device, "LCD x panning not supported\n");
		return -EINVAL;
	}

	if ((var->yoffset + var->yres > var->yres_virtual)) {
		dev_err(fb_info->device, "y panning exceeds\n");
		return -EINVAL;
	}

	if (host->is_mpu_lcd) {
		if (mxsfb_mpu_wait_for_ready(host) != 0)
		{
			pr_err("mxsfb_mpu_wait_for_ready failed, line==%d\n",__LINE__);
			return -ETIMEDOUT;
		}

		offset = fb_info->fix.line_length * var->yoffset;

		writel(fb_info->fix.smem_start + offset, host->base + host->devdata->next_buf);
		writel(fb_info->fix.smem_start + offset, host->base + host->devdata->cur_buf);

		mxsfb_mpu_refresh_panel(fb_info);
	} else {
		init_completion(&host->flip_complete);

		offset = fb_info->fix.line_length * var->yoffset;

		/* update on next VSYNC */
		writel(fb_info->fix.smem_start + offset,
				host->base + host->devdata->next_buf);

		writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
			host->base + LCDC_CTRL1 + REG_SET);

		ret = wait_for_completion_timeout(&host->flip_complete, HZ / 2);
		if (!ret) {
			dev_err(fb_info->device,
				"mxs wait for pan flip timeout\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int mxsfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

 	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static struct fb_ops mxsfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxsfb_check_var,
	.fb_set_par = mxsfb_set_par,
	.fb_setcolreg = mxsfb_setcolreg,
	.fb_ioctl = mxsfb_ioctl,
	.fb_blank = mxsfb_blank,
	.fb_pan_display = mxsfb_pan_display,
	.fb_mmap = mxsfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int mxsfb_restore_mode(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	unsigned line_count;
	unsigned period;
	unsigned long pa, fbsize;
	int bits_per_pixel, ofs;
	u32 transfer_count, vdctrl0, vdctrl2, vdctrl3, vdctrl4, ctrl;
	struct fb_videomode vmode;

	clk_enable_axi(host);
	clk_enable_disp_axi(host);

	/* Enable pixel clock earlier since in 7D
	 * the lcdif registers should be accessed
	 * when the pixel clock is enabled, otherwise
	 * the bus will be hang.
	 */
	clk_enable_pix(host);

	if (! host->is_mpu_lcd) {
		/* Only restore the mode when the controller is running */
		ctrl = readl(host->base + LCDC_CTRL);
		if (!(ctrl & CTRL_RUN))
			return -EINVAL;

		memset(&vmode, 0, sizeof(vmode));

		vdctrl0 = readl(host->base + LCDC_VDCTRL0);
		vdctrl2 = readl(host->base + LCDC_VDCTRL2);
		vdctrl3 = readl(host->base + LCDC_VDCTRL3);
		vdctrl4 = readl(host->base + LCDC_VDCTRL4);

		transfer_count = readl(host->base + host->devdata->transfer_count);

		vmode.xres = TRANSFER_COUNT_GET_HCOUNT(transfer_count);
		vmode.yres = TRANSFER_COUNT_GET_VCOUNT(transfer_count);

		switch (CTRL_GET_WORD_LENGTH(ctrl)) {
		case 0:
			bits_per_pixel = 16;
			break;
		case 3:
			bits_per_pixel = 32;
			break;
		case 1:
		default:
			return -EINVAL;
		}

		fb_info->var.bits_per_pixel = bits_per_pixel;

		vmode.pixclock = KHZ2PICOS(clk_get_rate(host->clk_pix) / 1000U);
		vmode.hsync_len = get_hsync_pulse_width(host, vdctrl2);
		vmode.left_margin = GET_HOR_WAIT_CNT(vdctrl3) - vmode.hsync_len;
		vmode.right_margin = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2) - vmode.hsync_len -
			vmode.left_margin - vmode.xres;
		vmode.vsync_len = VDCTRL0_GET_VSYNC_PULSE_WIDTH(vdctrl0);
		period = readl(host->base + LCDC_VDCTRL1);
		vmode.upper_margin = GET_VERT_WAIT_CNT(vdctrl3) - vmode.vsync_len;
		vmode.lower_margin = period - vmode.vsync_len - vmode.upper_margin - vmode.yres;

		vmode.vmode = FB_VMODE_NONINTERLACED;

		vmode.sync = 0;
		if (vdctrl0 & VDCTRL0_HSYNC_ACT_HIGH)
			vmode.sync |= FB_SYNC_HOR_HIGH_ACT;
		if (vdctrl0 & VDCTRL0_VSYNC_ACT_HIGH)
			vmode.sync |= FB_SYNC_VERT_HIGH_ACT;

		pr_debug("Reconstructed video mode:\n");
		pr_debug("%dx%d, hsync: %u left: %u, right: %u, vsync: %u, upper: %u, lower: %u\n",
				vmode.xres, vmode.yres,
				vmode.hsync_len, vmode.left_margin, vmode.right_margin,
				vmode.vsync_len, vmode.upper_margin, vmode.lower_margin);
		pr_debug("pixclk: %ldkHz\n", PICOS2KHZ(vmode.pixclock));

		fb_add_videomode(&vmode, &fb_info->modelist);

		host->ld_intf_width = CTRL_GET_BUS_WIDTH(ctrl);
		host->dotclk_delay = VDCTRL4_GET_DOTCLK_DLY(vdctrl4);

		fb_info->fix.line_length = vmode.xres * (bits_per_pixel >> 3);

		pa = readl(host->base + host->devdata->cur_buf);
		fbsize = fb_info->fix.line_length * vmode.yres;
		if (pa < fb_info->fix.smem_start)
			return -EINVAL;
		if (pa + fbsize > fb_info->fix.smem_start + fb_info->fix.smem_len)
			return -EINVAL;
		ofs = pa - fb_info->fix.smem_start;
		if (ofs) {
			memmove(fb_info->screen_base, fb_info->screen_base + ofs, fbsize);
			writel(fb_info->fix.smem_start, host->base + host->devdata->next_buf);
		}

		line_count = fb_info->fix.smem_len / fb_info->fix.line_length;
		fb_info->fix.ypanstep = 1;
		fb_info->fix.ywrapstep = 1;

		host->enabled = 1;
	}
	return 0;
}

#ifdef CONFIG_OF
static int mxsfb_init_fbinfo_dt(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct device *dev = &host->pdev->dev;
	struct device_node *np = host->pdev->dev.of_node;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings = NULL;
	const char *disp_dev;
	u32 width;
	int i;
	const char *lcd_panel;
	int ret = 0;

	host->id = of_alias_get_id(np, "lcdif");

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(dev, "failed to find display phandle\n");
		return -ENOENT;
	}

	host->is_mpu_lcd = of_property_read_bool(display_np, "mpu-mode");
	if (host->is_mpu_lcd) {
		struct fb_videomode *mpu_lcd_modedb;
		struct fb_videomode fb_vm;
		int size;

		ret = of_property_read_string(display_np, "lcd_panel", &lcd_panel);
		if (ret) {
			dev_err(dev, "failed to read of property lcd_panel\n");
			goto put_display_node;
		}

		for (i = 0; i < ARRAY_SIZE(mpu_lcd_db); i++) {
			if (!strcmp(lcd_panel, mpu_lcd_db[i].lcd_panel)) {
				host->mpu_lcd_functions =
					&mpu_lcd_db[i].lcd_callback;
				break;
			}
		}
		if (i == ARRAY_SIZE(mpu_lcd_db)) {
			dev_err(dev, "failed to find supported lcd panel.\n");
			ret = -EINVAL;
			goto put_display_node;
		}
		host->mpu_lcd_functions->get_mpu_lcd_videomode(&mpu_lcd_modedb, &size,
						&host->mpu_lcd_sigs);

		memcpy(&fb_vm, mpu_lcd_modedb, sizeof(struct fb_videomode));
		var->bits_per_pixel = host->mpu_lcd_sigs->panel_bpp;

		switch (host->mpu_lcd_sigs->interface_width) {
		case 8:
			host->ld_intf_width = STMLCDIF_8BIT;
			break;
		case 16:
			host->ld_intf_width = STMLCDIF_16BIT;
			break;
		case 18:
			host->ld_intf_width = STMLCDIF_18BIT;
			break;
		case 24:
			host->ld_intf_width = STMLCDIF_24BIT;
			break;
		default:
			dev_err(dev, "invalid interface width value\n");
			ret = -EINVAL;
			goto put_display_node;
		}

		/* lcd power gpio pin */
		host->mpu_lcd_sigs->lcd_power_gpio=devm_gpiod_get(dev,"lcd_power",GPIOD_OUT_HIGH);
		if (IS_ERR(host->mpu_lcd_sigs->lcd_power_gpio))
		{
    		dev_info(dev, "did not find lcd power gpio pin.\n");
		}
        else
        {
            gpiod_set_value(host->mpu_lcd_sigs->lcd_power_gpio,1);

        }

		/* lcd reset gpio pin */
		host->mpu_lcd_sigs->lcd_reset_gpio=devm_gpiod_get(dev,"lcd_reset",GPIOD_OUT_HIGH);
		if (IS_ERR(host->mpu_lcd_sigs->lcd_reset_gpio))
		{
			host->mpu_lcd_sigs->lcd_reset_is_gpio = 0;
		}
		else
		{
			host->mpu_lcd_sigs->lcd_reset_is_gpio = 1;
			dev_info(dev, "find lcd reset gpio pin.\n");
			gpiod_set_value(host->mpu_lcd_sigs->lcd_reset_gpio,1);
		}


		/* lcd rs gpio pin */
		
		host->mpu_lcd_sigs->lcd_rs_gpio=devm_gpiod_get(dev,"lcd_rs",GPIOD_OUT_HIGH);
		if (IS_ERR(host->mpu_lcd_sigs->lcd_rs_gpio))
		{
			host->mpu_lcd_sigs->lcd_rs_is_gpio = 0;
	        host->mpu_lcd_sigs->lcd_rs_is_on_reset = 
                    of_property_read_bool(np, "lcd_rs_is_on_reset");
            if (host->mpu_lcd_sigs->lcd_rs_is_on_reset)
				dev_info(dev, "Got lcd rs pin on LCD_RESET.\n");
            else
				dev_info(dev, "did not find lcd rs gpio pin.\n");
		}
		else
		{
			host->mpu_lcd_sigs->lcd_rs_is_gpio = 1;
			dev_info(dev, "Got lcd rs gpio pin.\n");
			gpiod_set_value(host->mpu_lcd_sigs->lcd_rs_gpio,1);
		}

		fb_add_videomode(&fb_vm, &fb_info->modelist);
		goto put_display_node;
	} else {
		ret = of_property_read_u32(display_np, "bus-width", &width);
		if (ret < 0) {
			dev_err(dev, "failed to get property bus-width\n");
			goto put_display_node;
		}

		switch (width) {
		case 8:
			host->ld_intf_width = STMLCDIF_8BIT;
			break;
		case 16:
			host->ld_intf_width = STMLCDIF_16BIT;
			break;
		case 18:
			host->ld_intf_width = STMLCDIF_18BIT;
			break;
		case 24:
			host->ld_intf_width = STMLCDIF_24BIT;
			break;
		default:
			dev_err(dev, "invalid bus-width value\n");
			ret = -EINVAL;
			goto put_display_node;
		}

		ret = of_property_read_u32(display_np, "bits-per-pixel",
					   &var->bits_per_pixel);
		if (ret < 0) {
			dev_err(dev, "failed to get property bits-per-pixel\n");
			goto put_display_node;
		}

		ret = of_property_read_string(np, "disp-dev", &disp_dev);
		if (!ret) {
			memcpy(host->disp_dev, disp_dev, strlen(disp_dev));
			/* Timing is from encoder driver */
			goto put_display_node;
		}

		timings = of_get_display_timings(display_np);
		if (!timings) {
			dev_err(dev, "failed to get display timings\n");
			ret = -ENOENT;
			goto put_display_node;
		}

		timings_np = of_find_node_by_name(display_np,
						  "display-timings");
		if (!timings_np) {
			dev_err(dev, "failed to find display-timings node\n");
			ret = -ENOENT;
			goto put_display_node;
		}

		for (i = 0; i < of_get_child_count(timings_np); i++) {
			struct videomode vm;
			struct fb_videomode fb_vm;

			ret = videomode_from_timings(timings, &vm, i);
			if (ret < 0)
				goto put_timings_node;
			ret = fb_videomode_from_videomode(&vm, &fb_vm);
			if (ret < 0)
				goto put_timings_node;

			if (!(vm.flags & DISPLAY_FLAGS_DE_HIGH))
				fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
			if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
				fb_vm.sync |= FB_SYNC_CLK_LAT_FALL;
			fb_add_videomode(&fb_vm, &fb_info->modelist);
		}
	}

put_timings_node:
	of_node_put(timings_np);
put_display_node:
	if (timings)
		kfree(timings);
	of_node_put(display_np);
	return ret;
}
#endif

static int mxsfb_init_fbinfo(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct fb_modelist *modelist;
	int ret;

	fb_info->fbops = &mxsfb_ops;
	fb_info->flags = FBINFO_FLAG_DEFAULT | FBINFO_READS_FAST;
	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR,
	fb_info->fix.accel = FB_ACCEL_NONE;

	ret = mxsfb_init_fbinfo_dt(host);
	if (ret)
		return ret;

	if (host->id < 0)
		sprintf(fb_info->fix.id, "mxs-lcdif");
	else
		sprintf(fb_info->fix.id, "mxs-lcdif%d", host->id);

	if (!list_empty(&fb_info->modelist)) {
		/* first video mode in the modelist as default video mode  */
		modelist = list_first_entry(&fb_info->modelist,
				struct fb_modelist, list);
		fb_videomode_to_var(var, &modelist->mode);
	}
	/* save the sync value getting from dtb */
	host->sync = fb_info->var.sync;

	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->accel_flags = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	/* init the color fields */
	mxsfb_check_var(var, fb_info);

	fb_info->fix.line_length = fb_info->var.xres *
	                          (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.smem_len = SZ_512K;//SZ_32M;

	/* Memory allocation for framebuffer */
	if (mxsfb_map_videomem(fb_info) < 0)
		return -ENOMEM;

	if (mxsfb_restore_mode(host))
		memset((char *)fb_info->screen_base, 0, fb_info->fix.smem_len);

	return 0;
}

static void mxsfb_dispdrv_init(struct platform_device *pdev,
			      struct fb_info *fbi)
{
	struct mxsfb_info *host = fbi->par;
	struct mxc_dispdrv_setting setting;
	struct device *dev = &pdev->dev;
	char disp_dev[32];

	memset(&setting, 0x0, sizeof(setting));
	setting.fbi = fbi;
	memcpy(disp_dev, host->disp_dev, strlen(host->disp_dev));
	disp_dev[strlen(host->disp_dev)] = '\0';
 
	host->dispdrv = mxc_dispdrv_gethandle(disp_dev, &setting);
	if (IS_ERR(host->dispdrv)) {
		host->dispdrv = NULL;
//		dev_info(dev, "failed to find mxc display driver %s\n", disp_dev);
	} else {
		dev_info(dev, "registered mxc display driver %s\n", disp_dev);
	}
}

static void mxsfb_free_videomem(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
 
	mxsfb_unmap_videomem(fb_info);
}

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_map_videomem(struct fb_info *fbi)
{
 	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA | GFP_KERNEL);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0x00, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_unmap_videomem(struct fb_info *fbi)
{
	dma_free_writecombine(fbi->device, fbi->fix.smem_len, fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

static struct platform_device_id mxsfb_devtype[] = {
	{
		.name = "imx23-fb",
		.driver_data = MXSFB_V3,
	}, {
		.name = "imx28-fb",
		.driver_data = MXSFB_V4,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mxsfb_devtype);

static const struct of_device_id mxsfb_dt_ids[] = {
	{ .compatible = "fsl,imx23-lcdif", .data = &mxsfb_devtype[0], },
	{ .compatible = "fsl,imx28-lcdif", .data = &mxsfb_devtype[1], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxsfb_dt_ids);

/* this is called back from the deferred io workqueue */
static void mxsfb_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
    mxsfb_mpu_refresh_panel(info);
}

static struct fb_deferred_io mxsfb_defio = {
	.delay		= HZ/50,
	.deferred_io	= mxsfb_deferred_io,
};


static int mxsfb_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxsfb_dt_ids, &pdev->dev);
	struct resource *res;
	struct mxsfb_info *host;
	struct fb_info *fb_info;
	struct pinctrl *pinctrl;
	int irq = platform_get_irq(pdev, 0);
	int gpio, ret;

	if (of_id)
		pdev->id_entry = of_id->data;

	gpio = of_get_named_gpio(pdev->dev.of_node, "enable-gpio", 0);
	if (gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, gpio,
		                          GPIOF_OUT_INIT_LOW, "lcd_pwr_en");
		if (ret) {
			dev_err(&pdev->dev, "faild to request gpio %d, ret = %d\n", gpio, ret);
			return ret;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get memory IO resource\n");
		return -ENODEV;
	}

	host = devm_kzalloc(&pdev->dev, sizeof(struct mxsfb_info), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "Failed to allocate IO resource\n");
		return -ENOMEM;
	}

	fb_info = framebuffer_alloc(sizeof(struct fb_info), &pdev->dev);
	if (!fb_info) {
		dev_err(&pdev->dev, "Failed to allocate fbdev\n");
		devm_kfree(&pdev->dev, host);
		return -ENOMEM;
	}
	host->fb_info = fb_info;
	fb_info->par = host;

	ret = devm_request_irq(&pdev->dev, irq, mxsfb_irq_handler, 0,
	                                   dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",irq, ret);
		ret = -ENODEV;
		goto fb_release;
	}

	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = PTR_ERR(host->base);
		goto fb_release;
	}

	host->pdev = pdev;
	platform_set_drvdata(pdev, host);

	host->devdata = &mxsfb_devdata[pdev->id_entry->driver_data];

	host->clk_pix = devm_clk_get(&host->pdev->dev, "pix");
	if (IS_ERR(host->clk_pix)) {
		host->clk_pix = NULL;
		ret = PTR_ERR(host->clk_pix);
		goto fb_release;
	}

	host->clk_axi = devm_clk_get(&host->pdev->dev, "axi");
	if (IS_ERR(host->clk_axi)) {
		host->clk_axi = NULL;
		ret = PTR_ERR(host->clk_axi);
		goto fb_release;
	}

	host->clk_disp_axi = devm_clk_get(&host->pdev->dev, "disp_axi");
	if (IS_ERR(host->clk_disp_axi)) {
		host->clk_disp_axi = NULL;
		ret = PTR_ERR(host->clk_disp_axi);
		goto fb_release;
	}

	host->reg_lcd = devm_regulator_get(&pdev->dev, "lcd");
	if (IS_ERR(host->reg_lcd))
		host->reg_lcd = NULL;

	fb_info->pseudo_palette = devm_kzalloc(&pdev->dev, sizeof(u32) * 16,
					       GFP_KERNEL);
	if (!fb_info->pseudo_palette) {
		ret = -ENOMEM;
		goto fb_release;
	}

	INIT_LIST_HEAD(&fb_info->modelist);

	pm_runtime_enable(&host->pdev->dev);

	ret = mxsfb_init_fbinfo(host);
	if (ret != 0)
		goto fb_pm_runtime_disable;

	mxsfb_dispdrv_init(pdev, fb_info);

	if (!host->dispdrv) {
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			ret = PTR_ERR(pinctrl);
			goto fb_pm_runtime_disable;
		}
	}

	if (!host->enabled) {
		writel(0, host->base + LCDC_CTRL);
		
		if (host->is_mpu_lcd) {
			mxsfb_enable_controller(fb_info);
			ret=mxsfb_set_par(host->fb_info);
			if (ret != 0) {
				dev_err(&pdev->dev, "mxsfb_set_par Failed\n");
				goto fb_destroy;
			}
		} else {
			mxsfb_set_par(host->fb_info);
			mxsfb_enable_controller(fb_info);
		}
		pm_runtime_get_sync(&host->pdev->dev);
	}
	fb_info->fbdefio = &mxsfb_defio;
	fb_deferred_io_init(fb_info);

	ret = register_framebuffer(fb_info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer\n");
		goto fb_destroy;
	}

	console_lock();
	ret = fb_blank(fb_info, FB_BLANK_UNBLANK);
	console_unlock();
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to unblank framebuffer\n");
		goto fb_unregister;
	}

#ifdef CONFIG_LOGO
	fb_prepare_logo(fb_info, 0);
	fb_show_logo(fb_info, 0);
	mxsfb_mpu_refresh_panel(fb_info);
#endif
    FirstReset=0;

    ret = sysfs_create_group(&pdev->dev.kobj, &mxsfb_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to create sysfs group\n");
		goto fb_unregister;
	}

	dev_info(&pdev->dev, "initialized\n");

    //ReadMem();
   // decode_setup(host);
	return 0;

fb_unregister:
	unregister_framebuffer(fb_info);
fb_destroy:
	if (host->enabled)
		clk_disable_unprepare(host->clk_pix);
	fb_destroy_modelist(&fb_info->modelist);
fb_pm_runtime_disable:
	pm_runtime_disable(&host->pdev->dev);
	devm_kfree(&pdev->dev, fb_info->pseudo_palette);
fb_release:
	framebuffer_release(fb_info);
	devm_kfree(&pdev->dev, host);

	return ret;
}

static int mxsfb_remove(struct platform_device *pdev)
{
	struct mxsfb_info *host = platform_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;

    sysfs_remove_group(&pdev->dev.kobj, &mxsfb_attr_group);

	if (host->enabled)
		mxsfb_disable_controller(fb_info);

	pm_runtime_disable(&host->pdev->dev);
	fb_deferred_io_cleanup(fb_info);
	unregister_framebuffer(fb_info);
	mxsfb_free_videomem(host);

	platform_set_drvdata(pdev, NULL);

	devm_kfree(&pdev->dev, fb_info->pseudo_palette);
	framebuffer_release(fb_info);
	devm_kfree(&pdev->dev, host);
	return 0;
}

static void mxsfb_shutdown(struct platform_device *pdev)
{
	struct mxsfb_info *host = platform_get_drvdata(pdev);
	/*
	 * Force stop the LCD controller as keeping it running during reboot
	 * might interfere with the BootROM's boot mode pads sampling.
	 */
	if (host->cur_blank == FB_BLANK_UNBLANK) {
		writel(CTRL_RUN, host->base + LCDC_CTRL + REG_CLR);
		writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);
		if (host->is_mpu_lcd)
			host->mpu_lcd_functions->mpu_lcd_poweroff(host);
	}
}

#ifdef CONFIG_PM
static int mxsfb_runtime_suspend(struct device *dev)
{
	release_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "mxsfb busfreq high release.\n");

	return 0;
}

static int mxsfb_runtime_resume(struct device *dev)
{
	request_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "mxsfb busfreq high request.\n");

	return 0;
}

static int mxsfb_suspend(struct device *pdev)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;
	int saved_blank;

	console_lock();
	fb_set_suspend(fb_info, 1);
	saved_blank = host->cur_blank;
	mxsfb_blank(FB_BLANK_POWERDOWN, fb_info);
	host->restore_blank = saved_blank;
	console_unlock();

	pinctrl_pm_select_sleep_state(pdev);

	return 0;
}

static int mxsfb_resume(struct device *pdev)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;

	//mxsfb_resume(pdev);

	console_lock();
	mxsfb_blank(host->restore_blank, fb_info);
	fb_set_suspend(fb_info, 0);
	console_unlock();

	return 0;
}
#else
#define	mxsfb_runtime_suspend	NULL
#define	mxsfb_runtime_resume	NULL

#define	mxsfb_suspend	NULL
#define	mxsfb_resume	NULL
#endif

static const struct dev_pm_ops mxsfb_pm_ops = {
	SET_RUNTIME_PM_OPS(mxsfb_runtime_suspend, mxsfb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(mxsfb_suspend, mxsfb_resume)
};

static struct platform_driver mxsfb_driver = {
	.probe = mxsfb_probe,
	.remove = mxsfb_remove,
	.shutdown = mxsfb_shutdown,
	.id_table = mxsfb_devtype,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mxsfb_dt_ids,
		   .pm = &mxsfb_pm_ops,
	},
};

module_platform_driver(mxsfb_driver);

MODULE_DESCRIPTION("Freescale mxs framebuffer driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
