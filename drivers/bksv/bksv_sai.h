/*
 * Copyright 2012-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FSL_SAI_H
#define __FSL_SAI_H

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <asm/uaccess.h>
#include <linux/dmaengine.h>

enum imx7d_pads {
	MX7D_PAD_RESERVE0 = 0,
	MX7D_PAD_RESERVE1 = 1,
	MX7D_PAD_RESERVE2 = 2,
	MX7D_PAD_RESERVE3 = 3,
	MX7D_PAD_RESERVE4 = 4,
	MX7D_PAD_GPIO1_IO08 = 5,
	MX7D_PAD_GPIO1_IO09 = 6,
	MX7D_PAD_GPIO1_IO10 = 7,
	MX7D_PAD_GPIO1_IO11 = 8,
	MX7D_PAD_GPIO1_IO12 = 9,
	MX7D_PAD_GPIO1_IO13 = 10,
	MX7D_PAD_GPIO1_IO14 = 11,
	MX7D_PAD_GPIO1_IO15 = 12,
	MX7D_PAD_EPDC_DATA00 = 13,
	MX7D_PAD_EPDC_DATA01 = 14,
	MX7D_PAD_EPDC_DATA02 = 15,
	MX7D_PAD_EPDC_DATA03 = 16,
	MX7D_PAD_EPDC_DATA04 = 17,
	MX7D_PAD_EPDC_DATA05 = 18,
	MX7D_PAD_EPDC_DATA06 = 19,
	MX7D_PAD_EPDC_DATA07 = 20,
	MX7D_PAD_EPDC_DATA08 = 21,
	MX7D_PAD_EPDC_DATA09 = 22,
	MX7D_PAD_EPDC_DATA10 = 23,
	MX7D_PAD_EPDC_DATA11 = 24,
	MX7D_PAD_EPDC_DATA12 = 25,
	MX7D_PAD_EPDC_DATA13 = 26,
	MX7D_PAD_EPDC_DATA14 = 27,
	MX7D_PAD_EPDC_DATA15 = 28,
	MX7D_PAD_EPDC_SDCLK = 29,
	MX7D_PAD_EPDC_SDLE = 30,
	MX7D_PAD_EPDC_SDOE = 31,
	MX7D_PAD_EPDC_SDSHR = 32,
	MX7D_PAD_EPDC_SDCE0 = 33,
	MX7D_PAD_EPDC_SDCE1 = 34,
	MX7D_PAD_EPDC_SDCE2 = 35,
	MX7D_PAD_EPDC_SDCE3 = 36,
	MX7D_PAD_EPDC_GDCLK = 37,
	MX7D_PAD_EPDC_GDOE = 38,
	MX7D_PAD_EPDC_GDRL = 39,
	MX7D_PAD_EPDC_GDSP = 40,
	MX7D_PAD_EPDC_BDR0 = 41,
	MX7D_PAD_EPDC_BDR1 = 42,
	MX7D_PAD_EPDC_PWR_COM = 43,
	MX7D_PAD_EPDC_PWR_STAT = 44,
	MX7D_PAD_LCD_CLK = 45,
	MX7D_PAD_LCD_ENABLE = 46,
	MX7D_PAD_LCD_HSYNC = 47,
	MX7D_PAD_LCD_VSYNC = 48,
	MX7D_PAD_LCD_RESET = 49,
	MX7D_PAD_LCD_DATA00 = 50,
	MX7D_PAD_LCD_DATA01 = 51,
	MX7D_PAD_LCD_DATA02 = 52,
	MX7D_PAD_LCD_DATA03 = 53,
	MX7D_PAD_LCD_DATA04 = 54,
	MX7D_PAD_LCD_DATA05 = 55,
	MX7D_PAD_LCD_DATA06 = 56,
	MX7D_PAD_LCD_DATA07 = 57,
	MX7D_PAD_LCD_DATA08 = 58,
	MX7D_PAD_LCD_DATA09 = 59,
	MX7D_PAD_LCD_DATA10 = 60,
	MX7D_PAD_LCD_DATA11 = 61,
	MX7D_PAD_LCD_DATA12 = 62,
	MX7D_PAD_LCD_DATA13 = 63,
	MX7D_PAD_LCD_DATA14 = 64,
	MX7D_PAD_LCD_DATA15 = 65,
	MX7D_PAD_LCD_DATA16 = 66,
	MX7D_PAD_LCD_DATA17 = 67,
	MX7D_PAD_LCD_DATA18 = 68,
	MX7D_PAD_LCD_DATA19 = 69,
	MX7D_PAD_LCD_DATA20 = 70,
	MX7D_PAD_LCD_DATA21 = 71,
	MX7D_PAD_LCD_DATA22 = 72,
	MX7D_PAD_LCD_DATA23 = 73,
	MX7D_PAD_UART1_RX_DATA = 74,
	MX7D_PAD_UART1_TX_DATA = 75,
	MX7D_PAD_UART2_RX_DATA = 76,
	MX7D_PAD_UART2_TX_DATA = 77,
	MX7D_PAD_UART3_RX_DATA = 78,
	MX7D_PAD_UART3_TX_DATA = 79,
	MX7D_PAD_UART3_RTS_B = 80,
	MX7D_PAD_UART3_CTS_B = 81,
	MX7D_PAD_I2C1_SCL = 82,
	MX7D_PAD_I2C1_SDA = 83,
	MX7D_PAD_I2C2_SCL = 84,
	MX7D_PAD_I2C2_SDA = 85,
	MX7D_PAD_I2C3_SCL = 86,
	MX7D_PAD_I2C3_SDA = 87,
	MX7D_PAD_I2C4_SCL = 88,
	MX7D_PAD_I2C4_SDA = 89,
	MX7D_PAD_ECSPI1_SCLK = 90,
	MX7D_PAD_ECSPI1_MOSI = 91,
	MX7D_PAD_ECSPI1_MISO = 92,
	MX7D_PAD_ECSPI1_SS0 = 93,
	MX7D_PAD_ECSPI2_SCLK = 94,
	MX7D_PAD_ECSPI2_MOSI = 95,
	MX7D_PAD_ECSPI2_MISO = 96,
	MX7D_PAD_ECSPI2_SS0 = 97,
	MX7D_PAD_SD1_CD_B = 98,
	MX7D_PAD_SD1_WP = 99,
	MX7D_PAD_SD1_RESET_B = 100,
	MX7D_PAD_SD1_CLK = 101,
	MX7D_PAD_SD1_CMD = 102,
	MX7D_PAD_SD1_DATA0 = 103,
	MX7D_PAD_SD1_DATA1 = 104,
	MX7D_PAD_SD1_DATA2 = 105,
	MX7D_PAD_SD1_DATA3 = 106,
	MX7D_PAD_SD2_CD_B = 107,
	MX7D_PAD_SD2_WP = 108,
	MX7D_PAD_SD2_RESET_B = 109,
	MX7D_PAD_SD2_CLK = 110,
	MX7D_PAD_SD2_CMD = 111,
	MX7D_PAD_SD2_DATA0 = 112,
	MX7D_PAD_SD2_DATA1 = 113,
	MX7D_PAD_SD2_DATA2 = 114,
	MX7D_PAD_SD2_DATA3 = 115,
	MX7D_PAD_SD3_CLK = 116,
	MX7D_PAD_SD3_CMD = 117,
	MX7D_PAD_SD3_DATA0 = 118,
	MX7D_PAD_SD3_DATA1 = 119,
	MX7D_PAD_SD3_DATA2 = 120,
	MX7D_PAD_SD3_DATA3 = 121,
	MX7D_PAD_SD3_DATA4 = 122,
	MX7D_PAD_SD3_DATA5 = 123,
	MX7D_PAD_SD3_DATA6 = 124,
	MX7D_PAD_SD3_DATA7 = 125,
	MX7D_PAD_SD3_STROBE = 126,
	MX7D_PAD_SD3_RESET_B = 127,
	MX7D_PAD_SAI1_RX_DATA = 128,
	MX7D_PAD_SAI1_TX_BCLK = 129,
	MX7D_PAD_SAI1_TX_SYNC = 130,
	MX7D_PAD_SAI1_TX_DATA = 131,
	MX7D_PAD_SAI1_RX_SYNC = 132,
	MX7D_PAD_SAI1_RX_BCLK = 133,
	MX7D_PAD_SAI1_MCLK = 134,
	MX7D_PAD_SAI2_TX_SYNC = 135,
	MX7D_PAD_SAI2_TX_BCLK = 136,
	MX7D_PAD_SAI2_RX_DATA = 137,
	MX7D_PAD_SAI2_TX_DATA = 138,
	MX7D_PAD_ENET1_RGMII_RD0 = 139,
	MX7D_PAD_ENET1_RGMII_RD1 = 140,
	MX7D_PAD_ENET1_RGMII_RD2 = 141,
	MX7D_PAD_ENET1_RGMII_RD3 = 142,
	MX7D_PAD_ENET1_RGMII_RX_CTL = 143,
	MX7D_PAD_ENET1_RGMII_RXC = 144,
	MX7D_PAD_ENET1_RGMII_TD0 = 145,
	MX7D_PAD_ENET1_RGMII_TD1 = 146,
	MX7D_PAD_ENET1_RGMII_TD2 = 147,
	MX7D_PAD_ENET1_RGMII_TD3 = 148,
	MX7D_PAD_ENET1_RGMII_TX_CTL = 149,
	MX7D_PAD_ENET1_RGMII_TXC = 150,
	MX7D_PAD_ENET1_TX_CLK = 151,
	MX7D_PAD_ENET1_RX_CLK = 152,
	MX7D_PAD_ENET1_CRS = 153,
	MX7D_PAD_ENET1_COL = 154,
};

#define FSL_SAI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

/* SAI Register Map Register */
#define FSL_SAI_TCSR	0x00 /* SAI Transmit Control */
#define FSL_SAI_TCR1	0x04 /* SAI Transmit Configuration 1 */
#define FSL_SAI_TCR2	0x08 /* SAI Transmit Configuration 2 */
#define FSL_SAI_TCR3	0x0c /* SAI Transmit Configuration 3 */
#define FSL_SAI_TCR4	0x10 /* SAI Transmit Configuration 4 */
#define FSL_SAI_TCR5	0x14 /* SAI Transmit Configuration 5 */
#define FSL_SAI_TDR		0x20 /* SAI Transmit Data */
#define FSL_SAI_TFR		0x40 /* SAI Transmit FIFO */
#define FSL_SAI_TMR		0x60 /* SAI Transmit Mask */
#define FSL_SAI_RCSR	0x80 /* SAI Receive Control */
#define FSL_SAI_RCR1	0x84 /* SAI Receive Configuration 1 */
#define FSL_SAI_RCR2	0x88 /* SAI Receive Configuration 2 */
#define FSL_SAI_RCR3	0x8c /* SAI Receive Configuration 3 */
#define FSL_SAI_RCR4	0x90 /* SAI Receive Configuration 4 */
#define FSL_SAI_RCR5	0x94 /* SAI Receive Configuration 5 */
#define FSL_SAI_RDR	0xa0 /* SAI Receive Data */
#define FSL_SAI_RFR	0xc0 /* SAI Receive FIFO */
#define FSL_SAI_RMR	0xe0 /* SAI Receive Mask */

#define FSL_SAI_xCSR(tx)	(tx ? FSL_SAI_TCSR : FSL_SAI_RCSR)
#define FSL_SAI_xCR1(tx)	(tx ? FSL_SAI_TCR1 : FSL_SAI_RCR1)
#define FSL_SAI_xCR2(tx)	(tx ? FSL_SAI_TCR2 : FSL_SAI_RCR2)
#define FSL_SAI_xCR3(tx)	(tx ? FSL_SAI_TCR3 : FSL_SAI_RCR3)
#define FSL_SAI_xCR4(tx)	(tx ? FSL_SAI_TCR4 : FSL_SAI_RCR4)
#define FSL_SAI_xCR5(tx)	(tx ? FSL_SAI_TCR5 : FSL_SAI_RCR5)
#define FSL_SAI_xDR(tx)		(tx ? FSL_SAI_TDR : FSL_SAI_RDR)
#define FSL_SAI_xFR(tx)		(tx ? FSL_SAI_TFR : FSL_SAI_RFR)
#define FSL_SAI_xMR(tx)		(tx ? FSL_SAI_TMR : FSL_SAI_RMR)

/* SAI Transmit/Recieve Control Register */
#define FSL_SAI_CSR_TERE	BIT(31)
#define FSL_SAI_CSR_BCE 	BIT(28)
#define FSL_SAI_CSR_FR		BIT(25)
#define FSL_SAI_CSR_SR		BIT(24)
#define FSL_SAI_CSR_xF_SHIFT	16
#define FSL_SAI_CSR_xF_W_SHIFT	18
#define FSL_SAI_CSR_xF_MASK	(0x1f << FSL_SAI_CSR_xF_SHIFT)
#define FSL_SAI_CSR_xF_W_MASK	(0x7 << FSL_SAI_CSR_xF_W_SHIFT)
#define FSL_SAI_CSR_WSF		BIT(20)
#define FSL_SAI_CSR_SEF		BIT(19)
#define FSL_SAI_CSR_FEF		BIT(18)
#define FSL_SAI_CSR_FWF		BIT(17)
#define FSL_SAI_CSR_FRF		BIT(16)
#define FSL_SAI_CSR_xIE_SHIFT	8
#define FSL_SAI_CSR_xIE_MASK	(0x1f << FSL_SAI_CSR_xIE_SHIFT)
#define FSL_SAI_CSR_WSIE	BIT(12)
#define FSL_SAI_CSR_SEIE	BIT(11)
#define FSL_SAI_CSR_FEIE	BIT(10)
#define FSL_SAI_CSR_FWIE	BIT(9)
#define FSL_SAI_CSR_FRIE	BIT(8)
#define FSL_SAI_CSR_FRDE	BIT(0)

/* SAI Transmit and Recieve Configuration 1 Register */
#define FSL_SAI_CR1_RFW_MASK	0x1f

/* SAI Transmit and Recieve Configuration 2 Register */
#define FSL_SAI_CR2_SYNC	BIT(30)
#define FSL_SAI_CR2_MSEL_MASK	(0x3 << 26)
#define FSL_SAI_CR2_MSEL_BUS	0
#define FSL_SAI_CR2_MSEL_MCLK1	BIT(26)
#define FSL_SAI_CR2_MSEL_MCLK2	BIT(27)
#define FSL_SAI_CR2_MSEL_MCLK3	(BIT(26) | BIT(27))
#define FSL_SAI_CR2_MSEL(ID)	((ID) << 26)
#define FSL_SAI_CR2_BCP		BIT(25)
#define FSL_SAI_CR2_BCD_MSTR	BIT(24)
#define FSL_SAI_CR2_DIV_MASK	0xff

/* SAI Transmit and Recieve Configuration 3 Register */
#define FSL_SAI_CR3_TRCE	BIT(16)
#define FSL_SAI_CR3_WDFL(x)	(x)
#define FSL_SAI_CR3_WDFL_MASK	0x1f

/* SAI Transmit and Recieve Configuration 4 Register */
#define FSL_SAI_CR4_FRSZ(x)	(((x) - 1) << 16)
#define FSL_SAI_CR4_FRSZ_MASK	(0x1f << 16)
#define FSL_SAI_CR4_SYWD(x)	(((x) - 1) << 8)
#define FSL_SAI_CR4_SYWD_MASK	(0x1f << 8)
#define FSL_SAI_CR4_MF		BIT(4)
#define FSL_SAI_CR4_FSE		BIT(3)
#define FSL_SAI_CR4_FSP		BIT(1)
#define FSL_SAI_CR4_FSD_MSTR	BIT(0)

/* SAI Transmit and Recieve Configuration 5 Register */
#define FSL_SAI_CR5_WNW(x)	(((x) - 1) << 24)
#define FSL_SAI_CR5_WNW_MASK	(0x1f << 24)
#define FSL_SAI_CR5_W0W(x)	(((x) - 1) << 16)
#define FSL_SAI_CR5_W0W_MASK	(0x1f << 16)
#define FSL_SAI_CR5_FBT(x)	((x) << 8)
#define FSL_SAI_CR5_FBT_MASK	(0x1f << 8)

/* SAI type */
#define FSL_SAI_DMA		BIT(0)
#define FSL_SAI_USE_AC97	BIT(1)
#define FSL_SAI_NET		BIT(2)
#define FSL_SAI_TRA_SYN		BIT(3)
#define FSL_SAI_REC_SYN		BIT(4)
#define FSL_SAI_USE_I2S_SLAVE	BIT(5)

#define FSL_FMT_TRANSMITTER	0
#define FSL_FMT_RECEIVER	1

/* SAI clock sources */
#define FSL_SAI_CLK_BUS		0
#define FSL_SAI_CLK_MAST1	1
#define FSL_SAI_CLK_MAST2	2
#define FSL_SAI_CLK_MAST3	3

#define FSL_SAI_MCLK_MAX	4

/* SAI data transfer numbers per DMA request */
#define FSL_SAI_MAXBURST_TX 6
#define FSL_SAI_MAXBURST_RX 6

#define IMX_RXBD_NUM 20
#define RX_BUF_SIZE	(PAGE_SIZE)
#define IMX_TXBD_NUM 16
#define TX_BUF_SIZE	(PAGE_SIZE)

#define BKDAC_SET_AUDIO_ENABLE  _IOW('P', 0x01, __u32)
#define BKDAC_GET_AUDIO_ENABLE  _IOR('P', 0x02, __u32)

enum sai_imx_devtype {
	IMX6SX_SAI,
	IMX7D_SAI,
};
/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};


struct fsl_sai;

struct sai_imx_devtype_data {
	/*
	void (*intctrl)(struct fsl_sai *, int);
	int (*config)(struct fsl_sai *, struct spi_imx_config *);
	void (*trigger)(struct fsl_sai *);
	int (*rx_available)(struct fsl_sai *);
	void (*reset)(struct fsl_sai *);*/
	enum sai_imx_devtype devtype;
};
struct imx_dma_bufinfo {
	bool filled;
	unsigned int rx_bytes;
};

struct imx_dma_buf {
	unsigned int		periods;
	unsigned int		period_len;
	unsigned int		buf_len;

	void			   *buf;
	dma_addr_t		    dmaaddr;
	unsigned int		cur_idx;
	unsigned int		last_completed_idx;
	bool                underrun;
	dma_cookie_t		cookie;
	struct imx_dma_bufinfo	buf_info[IMX_RXBD_NUM];
};

typedef struct 
{
    ktime_t wall_clock;
    unsigned int gpt_cnt;
    unsigned int gpt_icr1;
    unsigned int gpt_ocr1;
    unsigned int gps_present;
    int first_valid_sample;
    struct timespec ts;
    struct timespec ts1;
} adc_ioctl_starttime_t;

typedef struct 
{
	unsigned int samplefreq; //default 65536
} adc_ioctl_options_t;

#define ADC_OPTIONS   _IOWR(0xB5,0, adc_ioctl_options_t)
#define ADC_START     _IO(  0xB5,1)                          //Start using best of GPS and system time
#define ADC_STARTTIME _IOWR(0xB5,2, adc_ioctl_starttime_t)
#define ADC_STOP      _IO(  0xB5,3)
#define ADC_START_SYS _IO(  0xB5,4)                          //Start using system time
#define ADC_START_GPS _IO(  0xB5,5)                          //Start using GPS time

struct fsl_sai {
	struct platform_device *pdev;
	struct regmap *regmap;
    struct device *device;
	struct device		dev;
	struct cdev cdev;
	struct resource *res;
	struct clk *bus_clk;
	struct clk *mclk_clk[FSL_SAI_MCLK_MAX];
	struct clk *amclk;

    int re_sync_running;
    adc_ioctl_starttime_t  adc_start;
    adc_ioctl_options_t    adc_options;

    ktime_t start_clock;
    unsigned int start_count;   
    int start_result; //0 == fail, 1 == system_time, 2 == gps_time

	bool is_open;
	bool is_started;
	
	bool active; //driver not removed
    bool is_slave_mode;
	bool is_lsb_first;
	bool sai_on_imx;
    bool psu_is_on;

	unsigned int mclk_id[2];
	unsigned int mclk_streams;

	unsigned int slots;
	unsigned int slot_width;
	unsigned int gpio1[IMX_RXBD_NUM];
	
	struct mxc_gpio_hwdata *gpio1_addr;
	uint32_t volatile *iomux;
    void __iomem volatile *gpt;    //General Purpose Timer
	struct gpio_desc *clock_enable_gpio;
//	struct gpio_desc *adc_start_gpio;
	struct gpio_desc *ovl_high_gpio;
	struct gpio_desc *ovl_low_gpio;
	struct gpio_desc *ana_int_gpio;
	struct gpio_desc *gen_disable_gpio;
	struct gpio_desc *sai_mclk;
	struct gpio_desc *sai_bclk;
	struct gpio_desc *sai_sync;
	struct gpio_desc *sai_data;
	struct gpio_desc *led0;
	struct dma_slave_config rx_config;
	/* DMA fields */
	unsigned int		 dma_is_inited:1;
	unsigned int		 dma_is_enabled:1;
	unsigned int		 dma_is_runing:1;
	struct completion    dma_completion;
	struct imx_dma_buf   dma_buf;
	struct dma_chan		*dma_chan;
	wait_queue_head_t    dma_wait;
};

#define TX 1
#define RX 0

#endif /* __FSL_SAI_H */
