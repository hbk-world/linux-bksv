/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __MXSFB_H__
#define __MXSFB_H__

#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>

#define REG_SET	4
#define REG_CLR	8

#define LCDC_CTRL			0x00
#define LCDC_CTRL1			0x10
#define LCDC_V4_CTRL2			0x20
#define LCDC_V3_TRANSFER_COUNT		0x20
#define LCDC_V4_TRANSFER_COUNT		0x30
#define LCDC_V4_CUR_BUF			0x40
#define LCDC_V4_NEXT_BUF		0x50
#define LCDC_V3_CUR_BUF			0x30
#define LCDC_V3_NEXT_BUF		0x40
#define LCDC_TIMING			0x60
#define LCDC_VDCTRL0			0x70
#define LCDC_VDCTRL1			0x80
#define LCDC_VDCTRL2			0x90
#define LCDC_VDCTRL3			0xa0
#define LCDC_VDCTRL4			0xb0
#define LCDC_DVICTRL0			0xc0
#define LCDC_DVICTRL1			0xd0
#define LCDC_DVICTRL2			0xe0
#define LCDC_DVICTRL3			0xf0
#define LCDC_DVICTRL4			0x100
#define LCDC_V4_DATA			0x180
#define LCDC_V3_DATA			0x1b0
#define LCDC_V4_DEBUG0			0x1d0
#define LCDC_V3_DEBUG0			0x1f0

#define CTRL_SFTRST			(1 << 31)
#define CTRL_CLKGATE			(1 << 30)
#define CTRL_YCBCR422_INPUT	(1 << 29)
#define CTRL_READ_WRITEB		(1 << 28)
#define CTRL_WAIT_FOR_VSYNC_EDGE	(1 << 27)
#define CTRL_DATA_SHIFT_DIR	(1 << 26)
#define CTRL_SHIFT_NUM_BITS_MASK	(0x1f << 21)
#define CTRL_SHIFT_NUM_BITS_OFFSET	21
#define CTRL_DVI_MODE					(1 << 20)
#define CTRL_BYPASS_COUNT		(1 << 19)
#define CTRL_VSYNC_MODE			(1 << 18)
#define CTRL_DOTCLK_MODE		(1 << 17)
#define CTRL_DATA_SELECT		(1 << 16)
#define CTRL_INPUT_DATA_SWIZZLE_MASK			(0x3 << 14)
#define CTRL_INPUT_DATA_SWIZZLE_OFFSET			14
#define CTRL_CSC_DATA_SWIZZLE_MASK			(0x3 << 12)
#define CTRL_CSC_DATA_SWIZZLE_OFFSET			12
#define CTRL_LCD_DATABUS_WIDTH_MASK			(0x3 << 10)
#define CTRL_LCD_DATABUS_WIDTH_OFFSET			10
#define CTRL_LCD_DATABUS_WIDTH_16BIT			(0 << 10)
#define CTRL_LCD_DATABUS_WIDTH_8BIT			(1 << 10)
#define CTRL_LCD_DATABUS_WIDTH_18BIT			(2 << 10)
#define CTRL_LCD_DATABUS_WIDTH_24BIT			(3 << 10)
#define CTRL_SET_BUS_WIDTH(x)		(((x) & 0x3) << 10)
#define CTRL_GET_BUS_WIDTH(x)		(((x) >> 10) & 0x3)
#define CTRL_WORD_LENGTH_MASK				(0x3 << 8)
#define CTRL_WORD_LENGTH_OFFSET				8
#define CTRL_WORD_LENGTH_16BIT				(0 << 8)
#define CTRL_WORD_LENGTH_8BIT				(1 << 8)
#define CTRL_WORD_LENGTH_18BIT				(2 << 8)
#define CTRL_WORD_LENGTH_24BIT				(3 << 8)
#define CTRL_SET_WORD_LENGTH(x)		(((x) & 0x3) << 8)
#define CTRL_GET_WORD_LENGTH(x)		(((x) >> 8) & 0x3)
#define CTRL_RGB_TO_YCBCR422_CSC				(1 << 7)
#define CTRL_MASTER			(1 << 5)
#define CTRL_DATA_FORMAT_16_BIT				(1 << 3)
#define CTRL_DATA_FORMAT_18_BIT				(1 << 2)
#define CTRL_DATA_FORMAT_24_BIT				(1 << 1)
#define CTRL_RUN			(1 << 0)

#define CTRL1_COMBINE_MPU_WR_STRB				(1 << 27)
#define CTRL1_BM_ERROR_IRQ_EN				(1 << 26)
#define CTRL1_BM_ERROR_IRQ				(1 << 25)
#define CTRL1_RECOVERY_ON_UNDERFLOW		(1 << 24)
#define CTRL1_INTERLACE_FIELDS				(1 << 23)
#define CTRL1_START_INTERLACE_FROM_SECOND_FIELD		(1 << 22)
#define CTRL1_FIFO_CLEAR				(1 << 21)
#define CTRL1_IRQ_ON_ALTERNATE_FIELDS			(1 << 20)
#define CTRL1_BYTE_PACKING_FORMAT_MASK			(0xf << 16)
#define CTRL1_BYTE_PACKING_FORMAT_OFFSET			16
#define CTRL1_SET_BYTE_PACKAGING(x)		(((x) & 0xf) << 16)
#define CTRL1_GET_BYTE_PACKAGING(x)		(((x) >> 16) & 0xf)
#define CTRL1_OVERFLOW_IRQ_EN			(1 << 15)
#define CTRL1_UNDERFLOW_IRQ_EN			(1 << 14)
#define CTRL1_CUR_FRAME_DONE_IRQ_EN		(1 << 13)
#define CTRL1_VSYNC_EDGE_IRQ_EN			(1 << 12)
#define CTRL1_OVERFLOW_IRQ				(1 << 11)
#define CTRL1_UNDERFLOW_IRQ				(1 << 10)
#define CTRL1_CUR_FRAME_DONE_IRQ		(1 << 9)
#define CTRL1_VSYNC_EDGE_IRQ			(1 << 8)
#define CTRL1_BUSY_ENABLE					(1 << 2)
#define CTRL1_MODE86					(1 << 1)
#define CTRL1_RESET					(1 << 0)
#define CTRL1_IRQ_ENABLE_MASK			(CTRL1_OVERFLOW_IRQ_EN | \
						 CTRL1_UNDERFLOW_IRQ_EN | \
						 CTRL1_CUR_FRAME_DONE_IRQ_EN | \
						 CTRL1_VSYNC_EDGE_IRQ_EN)
#define CTRL1_IRQ_ENABLE_SHIFT			12
#define CTRL1_IRQ_STATUS_MASK			(CTRL1_OVERFLOW_IRQ | \
						 CTRL1_UNDERFLOW_IRQ | \
						 CTRL1_CUR_FRAME_DONE_IRQ | \
						 CTRL1_VSYNC_EDGE_IRQ)
#define CTRL1_IRQ_STATUS_SHIFT			8

#define CTRL2_OUTSTANDING_REQS_MASK			(0x7 << 21)
#define CTRL2_OUTSTANDING_REQS_OFFSET			21
#define CTRL2_OUTSTANDING_REQS_REQ_1			(0x0 << 21)
#define CTRL2_OUTSTANDING_REQS_REQ_2			(0x1 << 21)
#define CTRL2_OUTSTANDING_REQS_REQ_4			(0x2 << 21)
#define CTRL2_OUTSTANDING_REQS_REQ_8			(0x3 << 21)
#define CTRL2_OUTSTANDING_REQS_REQ_16			(0x4 << 21)
#define CTRL2_BURST_LEN_8					(1 << 20)
#define CTRL2_ODD_LINE_PATTERN_MASK			(0x7 << 16)
#define CTRL2_ODD_LINE_PATTERN_OFFSET			16
#define CTRL2_ODD_LINE_PATTERN_RGB			(0x0 << 16)
#define CTRL2_ODD_LINE_PATTERN_RBG			(0x1 << 16)
#define CTRL2_ODD_LINE_PATTERN_GBR			(0x2 << 16)
#define CTRL2_ODD_LINE_PATTERN_GRB			(0x3 << 16)
#define CTRL2_ODD_LINE_PATTERN_BRG			(0x4 << 16)
#define CTRL2_ODD_LINE_PATTERN_BGR			(0x5 << 16)
#define CTRL2_EVEN_LINE_PATTERN_MASK			(0x7 << 12)
#define CTRL2_EVEN_LINE_PATTERN_OFFSET			12
#define CTRL2_EVEN_LINE_PATTERN_RGB			(0x0 << 12)
#define CTRL2_EVEN_LINE_PATTERN_RBG			(0x1 << 12)
#define CTRL2_EVEN_LINE_PATTERN_GBR			(0x2 << 12)
#define CTRL2_EVEN_LINE_PATTERN_GRB			(0x3 << 12)
#define CTRL2_EVEN_LINE_PATTERN_BRG			(0x4 << 12)
#define CTRL2_EVEN_LINE_PATTERN_BGR			(0x5 << 12)
#define CTRL2_READ_PACK_DIR				(1 << 10)
#define CTRL2_READ_MODE_OUTPUT_IN_RGB_FORMAT		(1 << 9)
#define CTRL2_READ_MODE_6_BIT_INPUT			(1 << 8)
#define CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_MASK		(0x7 << 4)
#define CTRL2_READ_MODE_NUM_PACKED_SUBWORDS_OFFSET	4
#define CTRL2_INITIAL_DUMMY_READ_MASK			(0x7 << 1)
#define CTRL2_INITIAL_DUMMY_READ_OFFSET			1

#define TRANSFER_COUNT_V_COUNT_MASK			(0xffff << 16)
#define TRANSFER_COUNT_V_COUNT_OFFSET			16
#define TRANSFER_COUNT_H_COUNT_MASK			(0xffff << 0)
#define TRANSFER_COUNT_H_COUNT_OFFSET			0
#define TRANSFER_COUNT_SET_VCOUNT(x)	(((x) & 0xffff) << 16)
#define TRANSFER_COUNT_GET_VCOUNT(x)	(((x) >> 16) & 0xffff)
#define TRANSFER_COUNT_SET_HCOUNT(x)	((x) & 0xffff)
#define TRANSFER_COUNT_GET_HCOUNT(x)	((x) & 0xffff)

#define CUR_BUF_ADDR_MASK					0xffffffff
#define CUR_BUF_ADDR_OFFSET				0

#define NEXT_BUF_ADDR_MASK				0xffffffff
#define NEXT_BUF_ADDR_OFFSET				0

#define TIMING_CMD_HOLD_MASK				(0xff << 24)
#define TIMING_CMD_HOLD_OFFSET				24
#define TIMING_CMD_SETUP_MASK				(0xff << 16)
#define TIMING_CMD_SETUP_OFFSET				16
#define TIMING_DATA_HOLD_MASK				(0xff << 8)
#define TIMING_DATA_HOLD_OFFSET				8
#define TIMING_DATA_SETUP_MASK				(0xff << 0)
#define TIMING_DATA_SETUP_OFFSET				0

#define VDCTRL0_VSYNC_OEB				(1 << 29)
#define VDCTRL0_ENABLE_PRESENT		(1 << 28)
#define VDCTRL0_VSYNC_ACT_HIGH		(1 << 27)
#define VDCTRL0_HSYNC_ACT_HIGH		(1 << 26)
#define VDCTRL0_DOTCLK_ACT_FALLING	(1 << 25)
#define VDCTRL0_ENABLE_ACT_HIGH		(1 << 24)
#define VDCTRL0_VSYNC_PERIOD_UNIT	(1 << 21)
#define VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	(1 << 20)
#define VDCTRL0_HALF_LINE		(1 << 19)
#define VDCTRL0_HALF_LINE_MODE		(1 << 18)
#define VDCTRL0_SET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)
#define VDCTRL0_GET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)

#define VDCTRL2_SET_HSYNC_PERIOD(x)	((x) & 0x3ffff)
#define VDCTRL2_GET_HSYNC_PERIOD(x)	((x) & 0x3ffff)

#define VDCTRL3_MUX_SYNC_SIGNALS	(1 << 29)
#define VDCTRL3_VSYNC_ONLY		(1 << 28)
#define SET_HOR_WAIT_CNT(x)		(((x) & 0xfff) << 16)
#define GET_HOR_WAIT_CNT(x)		(((x) >> 16) & 0xfff)
#define SET_VERT_WAIT_CNT(x)		((x) & 0xffff)
#define GET_VERT_WAIT_CNT(x)		((x) & 0xffff)

#define VDCTRL4_SET_DOTCLK_DLY(x)	(((x) & 0x7) << 29) /* v4 only */
#define VDCTRL4_GET_DOTCLK_DLY(x)	(((x) >> 29) & 0x7) /* v4 only */
#define VDCTRL4_SYNC_SIGNALS_ON		(1 << 18)
#define SET_DOTCLK_H_VALID_DATA_CNT(x)	((x) & 0x3ffff)

#define DEBUG0_HSYNC			(1 < 26)
#define DEBUG0_VSYNC			(1 < 25)

#define MIN_XRES			120
#define MIN_YRES			120

#define RED 0
#define GREEN 1
#define BLUE 2
#define TRANSP 3

#define STMLCDIF_8BIT  1 /** pixel data bus to the display is of 8 bit width */
#define STMLCDIF_16BIT 0 /** pixel data bus to the display is of 16 bit width */
#define STMLCDIF_18BIT 2 /** pixel data bus to the display is of 18 bit width */
#define STMLCDIF_24BIT 3 /** pixel data bus to the display is of 24 bit width */

#define FB_SYNC_OE_LOW_ACT		0x80000000
#define FB_SYNC_CLK_LAT_FALL	0x40000000

enum mxsfb_devtype {
	MXSFB_V3,
	MXSFB_V4,
};

enum {
	MPU_DATA,
	MPU_CMD,
};

enum {
	MPU_READ,
	MPU_WRITE,
};

enum {
	MPU_BUS_8080,
	MPU_BUS_6800,
};

struct mpu_lcd_config {
	u32 bus_mode;
	u32 interface_width;
	u32 panel_bpp;
	bool lcd_reset_is_gpio;
	struct gpio_desc *lcd_power_gpio;
	struct gpio_desc *lcd_reset_gpio;
	bool lcd_rs_is_gpio;
	bool lcd_rs_is_on_reset;
	struct gpio_desc *lcd_rs_gpio;
};

/* CPU dependent register offsets */
struct mxsfb_devdata {
	unsigned transfer_count;
	unsigned data;
	unsigned cur_buf;
	unsigned next_buf;
	unsigned debug0;
	unsigned hs_wdth_mask;
	unsigned hs_wdth_shift;
	unsigned ipversion;
};

struct mxsfb_info;

struct mpu_lcd_callback {
	/* callback for lcd panel operation */
	void (*get_mpu_lcd_videomode)(struct fb_videomode **, int *,
			struct mpu_lcd_config **);
	int  (*mpu_lcd_setup)(struct mxsfb_info *);
	int  (*mpu_lcd_poweroff)(struct mxsfb_info *);
};

struct mpu_match_lcd {
	char *lcd_panel;
	struct mpu_lcd_callback lcd_callback;
};

struct mxsfb_info {
	struct fb_info *fb_info;
	struct platform_device *pdev;
	struct clk *clk_pix;
	struct clk *clk_axi;
	struct clk *clk_disp_axi;
	bool clk_pix_enabled;
	bool clk_axi_enabled;
	bool clk_disp_axi_enabled;
	void __iomem *base;	/* registers */
	u32 sync;		/* record display timing polarities */
	unsigned allocated_size;
	int enabled;
	unsigned int ld_intf_width;
	unsigned int dotclk_delay;
	const struct mxsfb_devdata *devdata;
	struct regulator *reg_lcd;
	bool wait4vsync;
	struct completion vsync_complete;
	struct completion flip_complete;
	int cur_blank;
	int restore_blank;
	char disp_dev[32];
	struct mxc_dispdrv_handle *dispdrv;
	int id;
	struct fb_var_screeninfo var;
	bool is_mpu_lcd;
	struct mpu_lcd_config * mpu_lcd_sigs;
	struct mpu_lcd_callback * mpu_lcd_functions;
	unsigned long inhibit_updates;
};

unsigned int mxsfb_mpu_access(struct mxsfb_info *host, int mode, int rw, int data);
int mxsfb_mpu_access_rcim(struct mxsfb_info *host, int mode, int rw, 
                 unsigned int index, int data, unsigned int length);
void mxsfb_mpu_setup_refresh_data(struct mxsfb_info *host);
int mxsfb_mpu_refresh_panel(struct fb_info *info);
int mxsfb_mpu_wait_for_ready(struct mxsfb_info *host);

#ifdef CONFIG_FB_MXS_ST7789S_QVGA
void mpu_st7789s_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mpu_lcd_config **data);
int mpu_st7789s_lcd_setup(struct mxsfb_info * mxsfb);
int mpu_st7789s_lcd_poweroff(struct mxsfb_info * mxsfb);
#endif

#ifdef CONFIG_FB_MXS_ST7735R_SQUARE
void mpu_st7735r_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mpu_lcd_config **data);
int mpu_st7735r_lcd_setup(struct mxsfb_info * mxsfb);
int mpu_st7735r_lcd_poweroff(struct mxsfb_info * mxsfb);
#endif

#ifdef CONFIG_FB_MXS_COM24H2P39ULC
void mpu_com24h2p39_get_lcd_videomode(struct fb_videomode **mode, int *size, struct mpu_lcd_config **data);
int  mpu_com24h2p39_lcd_setup(struct mxsfb_info * mxsfb);
int  mpu_com24h2p39_lcd_poweroff(struct mxsfb_info * mxsfb);
#endif

#endif
