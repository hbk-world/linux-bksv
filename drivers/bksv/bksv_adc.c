/*
 * Freescale ALSA SoC Digital Audio Interface (SAI) driver.
 *
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 *
 * This program is free software, you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or(at your
 * option) any later version.
 *
 */

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/regulator/consumer.h>
#include "bksv_sai.h"
typedef struct
{
    int valid;
    uint32_t ppstime;
    int length;
    uint32_t cnt1;
    uint32_t cnt2;
    unsigned char message[4*1024];
} gps_pps_data_t;

extern gps_pps_data_t gps_pps_data;
       gps_pps_data_t local_gps_pps_data;
extern int time_server_useGPS;
DEFINE_SPINLOCK(gps_pps_lock);

/*
 * There are 2 versions of the timer hardware on Freescale MXC hardware.
 * Version 1: MX1/MXL, MX21, MX27.
 * Version 2: MX25, MX31, MX35, MX37, MX51
 */

/* defines common for all i.MX */
#define GPT_CR 		0x00
#define GPT_PR 		0x04
#define GPT_SR   	0x08
#define GPT_IR		0x0c
#define GPT_OCR1 	0x10
#define GPT_OCR2 	0x14
#define GPT_OCR3 	0x18
#define GPT_ICR1 	0x1C
#define GPT_ICR2 	0x20
#define GPT_CNT	 	0x24

#define GPT_CR_EN 		    (1 <<  0) /* Enable module */
#define GPT_CR_WAITEN 		(1 <<  3) /* Wait enable mode */
#define GPT_CR_CLK_IPG		(1 <<  6)
#define GPT_CR_CLK_PER		(2 <<  6)
#define GPT_CR_CLK_OSC_DIV8	(5 <<  6)
#define GPT_CR_FRR	 	    (1 <<  9)
#define GPT_CR_EN_24M		(1 << 10)
#define GPT_CR_IM1_SH	    16
#define GPT_CR_IM2_SH	    18
#define GPT_CR_IM1_MASK	    (3 << 16)
#define GPT_CR_IM2_MASK     (3 << 18)
#define GPT_CR_OM1_SH	    20
#define GPT_CR_OM2_SH	    23
#define GPT_CR_OM3_SH	    26
#define GPT_CR_OM1_MASK	    (7 << 20)
#define GPT_CR_OM2_MASK     (7 << 23)
#define GPT_CR_OM3_MASK	    (7 << 26)
#define GPT_CR_FO1		    (1 << 29)
#define GPT_CR_FO2		    (1 << 30)
#define GPT_CR_FO3		    (1 << 31)

#define GPT_PR_PRE24M_SH	12
#define GPT_PR_PRE24M_MASK	(0XF << 12)

#define GPT_SR_OF1		(1 << 0)
#define GPT_SR_OF2		(1 << 1)
#define GPT_SR_OF3		(1 << 2)
#define GPT_SR_IF1		(1 << 3)
#define GPT_SR_IF2		(1 << 4)
#define GPT_SR_ROV		(1 << 5)

#define GPT_IR_OF1IE	(1 << 0)
#define GPT_IR_OF2IE	(1 << 1)
#define GPT_IR_OF3IE	(1 << 2)
#define GPT_IR_IF1IE	(1 << 3)
#define GPT_IR_IF2IE	(1 << 4)
#define GPT_IR_ROVIE	(1 << 5)

#define V2_TIMER_RATE_OSC_DIV8	3000000

#define FSL_SAI_FLAGS (FSL_SAI_CSR_SEIE |\
                       FSL_SAI_CSR_FEIE |\
                       FSL_SAI_CSR_FWIE )

#define DRIVER_NAME        "bkadc"
#define DEVICE_NAME_FORMAT "bkadc%d"

struct class *bk_sys_class     = NULL;
static dev_t adc_devt = 0;
static bool bkadc_platform_driver_done = 0;
static struct regulator *ana_regulator = NULL;
static int fsl_sai_runtime_resume(struct device *dev, unsigned int cmd);
static int fsl_sai_runtime_suspend(struct device *dev);
static long get_starttime(struct fsl_sai *sai, adc_ioctl_starttime_t *starttime);

static irqreturn_t fsl_sai_isr(int irq, void *devid)
{
	struct fsl_sai *sai = (struct fsl_sai *)devid;
	struct device *dev = &sai->pdev->dev;
	u32 flags, xcsr, mask;
	bool irq_none = true;

	/*
	 * Both IRQ status bits and IRQ mask bits are in the xCSR but
	 * different shifts. And we here create a mask only for those
	 * IRQs that we activated.
	 */
	mask = (FSL_SAI_FLAGS >> FSL_SAI_CSR_xIE_SHIFT) << FSL_SAI_CSR_xF_SHIFT;

	/* Tx IRQ */
	regmap_read(sai->regmap, FSL_SAI_TCSR, &xcsr);
	flags = xcsr & mask;

	if (flags)
		irq_none = false;
	else
		goto irq_rx;

    if (flags & FSL_SAI_CSR_WSF)
    {
        dev_dbg(dev, "isr: Start of Tx word detected\n");
    }

    if (flags & FSL_SAI_CSR_SEF)
    {
        dev_dbg(dev, "isr: Tx Frame sync error detected\n");
    }

    if (flags & FSL_SAI_CSR_FEF) {
        dev_dbg(dev, "isr: Transmit underrun detected\n");
        /* FIFO reset for safety */
        xcsr |= FSL_SAI_CSR_FR;
    }

    if (flags & FSL_SAI_CSR_FWF)
    {
        dev_dbg(dev, "isr: Enabled transmit FIFO is empty\n");
    }

    if (flags & FSL_SAI_CSR_FRF)
    {
        dev_dbg(dev, "isr: Transmit FIFO watermark has been reached\n");
    }

	flags &= FSL_SAI_CSR_xF_W_MASK;
	xcsr &= ~FSL_SAI_CSR_xF_MASK;

	if (flags)
		regmap_write(sai->regmap, FSL_SAI_TCSR, flags | xcsr);

irq_rx:

	/* Rx IRQ */
	regmap_read(sai->regmap, FSL_SAI_RCSR, &xcsr);
	flags = xcsr & mask;

	if (flags)
		irq_none = false;
	else
		goto out;

    if (flags & FSL_SAI_CSR_WSF)
    {
		dev_dbg(dev, "isr: Start of Rx word detected\n");
    }

    if (flags & FSL_SAI_CSR_SEF)
    {
		dev_dbg(dev, "isr: Rx Frame sync error detected\n");
    }

	if (flags & FSL_SAI_CSR_FEF) {
		dev_dbg(dev, "isr: Receive overflow detected\n");
		/* FIFO reset for safety */
		xcsr |= FSL_SAI_CSR_FR;
	}

	if (flags & FSL_SAI_CSR_FWF)
    {
		dev_dbg(dev, "isr: Enabled receive FIFO is full\n");
    }

	if (flags & FSL_SAI_CSR_FRF)
    {
		dev_dbg(dev, "isr: Receive FIFO watermark has been reached\n");
    }

	flags &= FSL_SAI_CSR_xF_W_MASK;
	xcsr &= ~FSL_SAI_CSR_xF_MASK;

	if (flags)
		regmap_write(sai->regmap, FSL_SAI_RCSR, flags | xcsr);

out:
	if (irq_none)
		return IRQ_NONE;
	else
		return IRQ_HANDLED;
}


static int fsl_sai_set_bclk(struct fsl_sai *sai, bool tx, u32 freq)
{
	unsigned long clk_rate;
	u32 savediv = 0, ratio, savesub = freq;
	u32 id;
	int ret = 0;
//    printk("%s freq=%d\n",__func__,freq);

	/* Don't apply to slave mode */
	if (sai->is_slave_mode)
		return 0;

	for (id = 0; id < FSL_SAI_MCLK_MAX; id++) {
		clk_rate = clk_get_rate(sai->mclk_clk[id]);
		if (!clk_rate)
			continue;
		ratio = clk_rate / freq;
		ret = clk_rate - ratio * freq;
		/*
		 * Drop the source that can not be
		 * divided into the required rate.
		 */
		if (ret != 0 && clk_rate / ret < 1000)
			continue;
		if (ratio % 2 == 0 && ratio >= 2 && ratio <= 512)
			ratio /= 2;
		else
			continue;
		if (ret < savesub) {
			savediv = ratio;
			sai->mclk_id[tx] = id;
			savesub = ret;
		}
		if (ret == 0)
			break;
	}
	if (savediv == 0) {
		dev_err(&sai->pdev->dev,"failed to derive required %cx rate: %d\n",tx ? 'T' : 'R', freq);
		return -EINVAL;
	}
	/*
	 * 1) For Asynchronous mode, we must set RCR2 register for capture, and
	 *    set TCR2 register for playback.
	 * 2) For Tx sync with Rx clock, we must set RCR2 register for playback
	 *    and capture.
	 * 3) For Rx sync with Tx clock, we must set TCR2 register for playback
	 *    and capture.
	 * 4) For Tx and Rx are both Synchronous with another SAI, we just
	 *    ignore it.
	 */
    if (tx)
    {
        regmap_update_bits(sai->regmap, FSL_SAI_TCR2,
                                        FSL_SAI_CR2_MSEL_MASK,
                                        FSL_SAI_CR2_MSEL(1));
        regmap_update_bits(sai->regmap, FSL_SAI_TCR2,
                                        FSL_SAI_CR2_DIV_MASK, savediv - 1);
    }
    else
    {
        regmap_update_bits(sai->regmap, FSL_SAI_RCR2,
                                        FSL_SAI_CR2_MSEL_MASK,
                                        FSL_SAI_CR2_MSEL(1));
        regmap_update_bits(sai->regmap, FSL_SAI_RCR2,
                                        FSL_SAI_CR2_DIV_MASK, savediv - 1);
    }
	return 0;
}


static struct reg_default fsl_sai_reg_defaults[] = {
	{FSL_SAI_TCR1, 0},
	{FSL_SAI_TCR2, 0},
	{FSL_SAI_TCR3, 0},
	{FSL_SAI_TCR4, 0},
	{FSL_SAI_TCR5, 0},
	{FSL_SAI_TDR,  0},
	{FSL_SAI_TMR,  0},
	{FSL_SAI_RCR1, 0},
	{FSL_SAI_RCR2, 0},
	{FSL_SAI_RCR3, 0},
	{FSL_SAI_RCR4, 0},
	{FSL_SAI_RCR5, 0},
	{FSL_SAI_RMR,  0},
};

static bool fsl_sai_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_SAI_TCSR:
	case FSL_SAI_TCR1:
	case FSL_SAI_TCR2:
	case FSL_SAI_TCR3:
	case FSL_SAI_TCR4:
	case FSL_SAI_TCR5:
	case FSL_SAI_TFR:
	case FSL_SAI_TMR:
	case FSL_SAI_RCSR:
	case FSL_SAI_RCR1:
	case FSL_SAI_RCR2:
	case FSL_SAI_RCR3:
	case FSL_SAI_RCR4:
	case FSL_SAI_RCR5:
	case FSL_SAI_RDR:
	case FSL_SAI_RFR:
	case FSL_SAI_RMR:
		return true;
	default:
		return false;
	}
}

static bool fsl_sai_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_SAI_TCSR:
	case FSL_SAI_RCSR:
	case FSL_SAI_TFR:
	case FSL_SAI_RFR:
	case FSL_SAI_RDR:
		return true;
	default:
		return false;
	}
}

static bool fsl_sai_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_SAI_TCSR:
	case FSL_SAI_TCR1:
	case FSL_SAI_TCR2:
	case FSL_SAI_TCR3:
	case FSL_SAI_TCR4:
	case FSL_SAI_TCR5:
	case FSL_SAI_TDR:
	case FSL_SAI_TMR:
	case FSL_SAI_RCSR:
	case FSL_SAI_RCR1:
	case FSL_SAI_RCR2:
	case FSL_SAI_RCR3:
	case FSL_SAI_RCR4:
	case FSL_SAI_RCR5:
	case FSL_SAI_RMR:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_sai_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = FSL_SAI_RMR,
	.reg_defaults = fsl_sai_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_sai_reg_defaults),
	.readable_reg = fsl_sai_readable_reg,
	.volatile_reg = fsl_sai_volatile_reg,
	.writeable_reg = fsl_sai_writeable_reg,
	.cache_type = REGCACHE_FLAT,
};

#define ENTRY(reg) {reg,#reg}
typedef struct
{
    int reg;
    char name[15];
} sai_reg_t;
sai_reg_t sai_regs[]=
{
	ENTRY( FSL_SAI_TCSR),
	ENTRY( FSL_SAI_TCR1),
	ENTRY( FSL_SAI_TCR2),
	ENTRY( FSL_SAI_TCR3),
	ENTRY( FSL_SAI_TCR4),
	ENTRY( FSL_SAI_TCR5),
	ENTRY( FSL_SAI_TFR),
	ENTRY( FSL_SAI_TMR),
	ENTRY( FSL_SAI_RCSR),
	ENTRY( FSL_SAI_RCR1),
	ENTRY( FSL_SAI_RCR2),
	ENTRY( FSL_SAI_RCR3),
	ENTRY( FSL_SAI_RCR4),
	ENTRY( FSL_SAI_RCR5),
	ENTRY( FSL_SAI_RDR),
	ENTRY( FSL_SAI_RFR),
	ENTRY( FSL_SAI_RMR)
};
#define _countof(a) \
( \
    sizeof(a) / sizeof(a[0]) \
)

void dump_sai_regs(struct fsl_sai *sai)
{
    int i=0;
    unsigned int val;
    for (i=0;i<_countof(sai_regs);i++)
    {
       	regmap_read(sai->regmap, sai_regs[i].reg, &val);
        printk("%x=%s=%08x\n",sai_regs[i].reg,sai_regs[i].name,val);
    }
    printk("%x=%08x\n",MX7D_PAD_SAI1_MCLK,   sai->iomux[MX7D_PAD_SAI1_MCLK]);
    printk("%x=%08x\n",MX7D_PAD_SAI1_RX_BCLK,sai->iomux[MX7D_PAD_SAI1_RX_BCLK]);
    printk("%x=%08x\n",MX7D_PAD_SAI1_RX_SYNC,sai->iomux[MX7D_PAD_SAI1_RX_SYNC]);

}

int scan_for_first_block(struct fsl_sai *sai)
{
    int startbuf;
    int bufno=0;
    int i,j;
    int *buf;
    int start_index=1024-(85*2);
    int non_zero_found=1;
    int zero_found=0;
    int zero_count=0;
    int zero84_found=0;

//    printk("scan_for_first_block %d\n",sai->dma_buf.cur_idx);
 	startbuf=sai->dma_buf.cur_idx-1;
    if (startbuf<0)
        startbuf= IMX_RXBD_NUM-1;
    bufno=startbuf;
    for (j=0;j<2;j++)
    {//Look at last of previus buf and current buf, to find 84*2 zeros
        buf=&((int*)sai->dma_buf.buf)[(RX_BUF_SIZE/sizeof(int))*bufno];
        for (i=start_index;i<1024;i=i+2)
        {
            if ((buf[i]==0) && (buf[i+1]==0))
            {
                if (non_zero_found==1)
                {
                    if ( (zero_found==0))
                    {
                        zero_count = 1;
                        zero_found = 1;
                    }
                    else
                    {
                        zero_count++;
                    }
                }
            }
            else
            {
                if (zero_found==1)
                {
                    if (zero_count > 84)
                        printk(KERN_WARNING "sync: %d zeros found\n", zero_count);
                    if (zero_count == 84)
                    {
                        zero84_found = 1;
                        sai->adc_start.first_valid_sample=(i>>1);
                        break;
                    }
                    zero_found=0;
                }
                non_zero_found=1;
            }
        }
        if (zero84_found)
        {
            break;
        }
        start_index = 0;
        bufno++;
        bufno = bufno % IMX_RXBD_NUM;
    }
    if (zero84_found)
    {
        printk("sync84 found buf=%d first_valid_sample=%d\n",bufno,sai->adc_start.first_valid_sample);
    }
    return zero84_found;
}

static void dma_rx_callback(void *data)
{
	struct fsl_sai *sai = data;
    unsigned int gpio1_val,gpio1_isr;
    struct timespec t;
    int sync_found=0;
	unsigned int cnt,icr1;
    static int icr_seen=0;
    static int last_fault_icr=0;
    static int last_fault_valid=0;
    int diff;
    int gps=0;
    
    ktime_get_ts(&t);
    gpio1_val=sai->gpio1_addr->psr_reg;
    gpio1_isr=sai->gpio1_addr->isr_reg;    //bit8=OVL_HIGH_N, bit9=OVL_LOW_N, bit 10=ANA_INT
    gpio1_isr &= (3<<8); 
    if (gpio1_isr)
    {
        sai->gpio1_addr->isr_reg=gpio1_isr; //remove interrupt
    }
    if (sai->re_sync_running)
    {
        if ((sync_found=scan_for_first_block(sai))==1)
        {
            sai->re_sync_running=0;
        }
    }
    cnt  = __raw_readl(sai->gpt + GPT_CNT);
    icr1 = __raw_readl(sai->gpt + GPT_ICR1);
    if ((icr1 != 0) || icr_seen)
    {
        if (icr1 != 0)
            icr_seen = 1;
        if (cnt >= icr1)
            diff = cnt - icr1;
        else
            diff = U32_MAX-(icr1 - cnt);
        
        if (diff <(16777216*2 + (16777216/2))) // seen pps-pulse in the last 2.5 sec
        {
            if ((last_fault_valid == 0) || (last_fault_valid && (icr1 != last_fault_icr)))
            {
                gps = 1;
                last_fault_valid = 0;
            }
            else
                gps = 0;
        }
        else
        {
            gps=0;
            last_fault_icr = icr1;
            last_fault_valid = 1;
        }
    }

//           6   5         4       3         2         1       0
    //store gps,sync_found,isr_low,isr_high,windscreen,ovl_low,ovl_high   all active high        
    sai->gpio1[sai->dma_buf.cur_idx]=(gps << 6) | ((sync_found & 1) << 5) | (gpio1_isr>>5) | 
                                     (((gpio1_val >> 8)& 0x7)^0x3);

	sai->dma_buf.cur_idx++;
	sai->dma_buf.cur_idx %= IMX_RXBD_NUM;
#if 0
    if (((gpio1_val >>10)&1) != guf)
    {
        guf=(gpio1_val >>10)&1;
        printk("ADC guf = %d\n",guf);
    }
#endif

	if (sai->dma_buf.cur_idx == sai->dma_buf.last_completed_idx)
    {
        unsigned int cr;
		dev_err(&sai->pdev->dev, "overwrite! buf=%d\n",sai->dma_buf.cur_idx);
//        printk("dma_rx_callback: t=%llu.%09llu\n",  (unsigned long long)t.tv_sec,(unsigned long long)t.tv_nsec);

        cr = __raw_readl(sai->gpt + GPT_CR);
        cr &= ~GPT_CR_OM1_MASK;
        cr |= 2 << GPT_CR_OM1_SH;
	    __raw_writel(cr, sai->gpt + GPT_CR);
        cr |= GPT_CR_FO1;
	    __raw_writel(cr, sai->gpt + GPT_CR); //stop sampling
        
    //  ReadMem();

        //Stop SAI interface
        regmap_update_bits(sai->regmap, FSL_SAI_RCSR,
                    FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_xIE_MASK, 0);
        dmaengine_terminate_all(sai->dma_chan);
        sai->dma_is_runing = 0;
    }
    complete(&sai->dma_completion);
}

static int start_rx_dma(struct fsl_sai *sai)
{
	struct dma_chan	*chan = sai->dma_chan;
	struct dma_async_tx_descriptor *desc;
    
    memset(sai->dma_buf.buf,1,IMX_RXBD_NUM * RX_BUF_SIZE); //set all != 0
    sai->dma_buf.periods = IMX_RXBD_NUM;
	sai->dma_buf.period_len = RX_BUF_SIZE;
	sai->dma_buf.buf_len = IMX_RXBD_NUM * RX_BUF_SIZE;
	sai->dma_buf.cur_idx = 0;
	sai->dma_buf.last_completed_idx = -1;
	desc = dmaengine_prep_dma_cyclic(chan, sai->dma_buf.dmaaddr,
		sai->dma_buf.buf_len, sai->dma_buf.period_len,
		DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(&sai->pdev->dev, "Prepare for the RX slave dma failed!\n");
		return -EINVAL;
	}

	desc->callback = dma_rx_callback;
	desc->callback_param = sai;

//	dev_dbg(&sai->pdev->dev, "RX: prepare for the DMA.\n");
	sai->dma_buf.cookie = dmaengine_submit(desc);
	dma_async_issue_pending(chan);

	sai->dma_is_runing = 1;
	return 0;
}

ssize_t bksv_adc_fop_read(struct file *file,
    char __user *buffer,
    size_t length,
    loff_t *ppos)
{
    struct fsl_sai* sai = file->private_data;
    int buf[IMX_RXBD_NUM+1],res;
    unsigned int cur_buf, count;
    struct timespec t;
//    static int clock_off=false;
    
    if (sai == NULL)
    {
        printk("%s sai is NULL\n",__func__);
        return  -EFAULT;
    }
    
    if (length < (IMX_RXBD_NUM+1)*sizeof(int))
    {
        dev_err(&sai->pdev->dev, "%s length wrong\n",__func__);
        return  -EINVAL;
    }

    if (buffer == NULL)
    {
        dev_err(&sai->pdev->dev, "%s buffer is NULL\n",__func__);
        return  -EFAULT;
    }

    count = 0;
    while (try_wait_for_completion(&sai->dma_completion))
    {
        //count "allready done completions" from the callback function
        count++;
    }; 
    if (sai->is_open == 0)
    {
        dev_err(&sai->pdev->dev, "%s driver not open any more\n",__func__);
        return  -EFAULT;
    }
    ktime_get_ts(&t);
#if 0
    if ((clock_off==false) && (t.tv_sec >= 60))
    {
        sai->iomux[MX7D_PAD_SAI1_MCLK] &= ~0x7;
        sai->iomux[MX7D_PAD_SAI1_MCLK] |=  0x5; //make GPIO
        gpiod_direction_input(sai->sai_mclk);
        
        sai->iomux[MX7D_PAD_SAI1_RX_BCLK] &= ~0x7;
        sai->iomux[MX7D_PAD_SAI1_RX_BCLK] |=  0x5; //make GPIO
        gpiod_direction_input(sai->sai_bclk);
    
        sai->iomux[MX7D_PAD_SAI1_RX_SYNC] &= ~0x7;
        sai->iomux[MX7D_PAD_SAI1_RX_SYNC] |=  0x5; //make GPIO
        gpiod_direction_input(sai->sai_sync);
        clock_off=true;
        printk("stop sample clock %d\n",t.tv_sec);
    }
#endif
    if (count == 0)
    {
        int time_to_timeout = wait_for_completion_timeout(&sai->dma_completion, HZ);
        if(time_to_timeout == 0)
        {
            dev_err(&sai->pdev->dev, "ADC timeout!\n");
//            dump_sai_regs(sai);
            return 0;
        }
        count++;
    }
    if (sai->is_open == 0)
        return  -EFAULT;
    cur_buf = sai->dma_buf.last_completed_idx + count + 1;
    cur_buf %= IMX_RXBD_NUM;

    if ((sai->dma_buf.cur_idx == sai->dma_buf.last_completed_idx) || (sai->dma_is_runing == 0))
    {
        if (sai->dma_is_runing == 0)
            dev_err(&sai->pdev->dev, "DMA stopped due to overwrite!\n");
        else
            dev_err(&sai->pdev->dev, "overwrite!\n");
        buf[0] = -1;
    }
    else
    {
        buf[0]=cur_buf;
#if 0
        if (clock_off==false)
        {
            dev_err(&sai->pdev->dev, "reg dump\n");
            dump_sai_regs(sai);
            clock_off=true;
        }
#endif
    }

    memcpy(&buf[1],sai->gpio1,IMX_RXBD_NUM*sizeof(int));
//    printk("%llu.%09llu r cb=%d cur=%d count=%d\n",(unsigned long long)t.tv_sec,(unsigned long long)t.tv_nsec,cur_buf,sai->dma_buf.cur_idx,count);
    res=arm_copy_to_user(buffer,buf,(IMX_RXBD_NUM+1)*sizeof(int));
    if (res < 0)
    {
        dev_err(&sai->pdev->dev, "arm_copy_to_user failed res=%d\n",res);
        return res;
    }
    else
        if (res == 0)
            return (IMX_RXBD_NUM+1)*sizeof(int); //succes
        else
        {
            dev_err(&sai->pdev->dev, "arm_copy_to_user failed res=%d\n",res);
            return 0;
        }
}

ssize_t bksv_adc_fop_write(struct file *file, const char __user *buffer,
    size_t length, loff_t *ppos)
{
    struct fsl_sai* sai = file->private_data;
    int buf[10];
    if ((length <= 10) && (arm_copy_from_user(buf,buffer,length) == 0))
    {
        sai->dma_buf.last_completed_idx = buf[0];        //user program has finished using buffer[buf[0]]
        sai->dma_buf.last_completed_idx %= IMX_RXBD_NUM;
    }
    return length;
}
int ubx_checksum(unsigned char *buf,int len)
{
    int n;
    unsigned char ck_a = (unsigned char)0;
    unsigned char ck_b = (unsigned char)0;
    for (n = 2; n < (len - 2); n++)
    {
//        printk("0x%02x ",buf[n]);
        ck_a += buf[n];
        ck_b += ck_a;
    }
//        printk("\n");
    if ((ck_a == buf[len - 2]) && (ck_b == buf[len - 1]))
        return 0;
    else
    {
        printk("UBX checksum 0x%02hhx%02hhx over length %d,"
            " expecting 0x%02hhx%02hhx (type 0x%02hhx%02hhx)\n",
            ck_a,
            ck_b,
            len-2-2,
            buf[len - 2],
            buf[len - 1],
            buf[2], buf[3]);
    }
    return -1;
}

int findubx(unsigned char * buf, unsigned char class, unsigned char id,unsigned int length,unsigned char **pos)
{
    int i=0;
    int found=0;
    *pos=NULL;
    do
    {
        short int *len =(short int *)&buf[i+4];

        if ((buf[i+0] != 0xb5) || (buf[i+1] != 0x62))
            break;
        if ((buf[i+2] == class) && (buf[i+3] == id))
        {
            found = 1;
            *pos = &buf[i];
            break;
        }
        
        i +=  2 + 1 + 1 + 2 + *len + 2;
        if (i>=length)
            break;
    } while (found == 0);
    return found;
}
/*******************************************************************************************/

#define GPS_EPOCH	315964800	/* 6 Jan 1981 00:00:00 UTC */

/* time constant */
#define SECS_PER_DAY	(60*60*24)		/* seconds per day */
#define SECS_PER_WEEK	(7*SECS_PER_DAY)	/* seconds per week */
typedef double timestamp_t;	/* Unix time in seconds with fractional part */

typedef struct
{
    unsigned int iTOW;
    int fTOW;
    short week;
    char leapS;
    unsigned char valid;
    unsigned int tAcc;
} nav_timegps_t;

#define USE_IOCTL_NAMES
#ifdef USE_IOCTL_NAMES
typedef struct 
{
    unsigned int cmd;
    char name[15];
} ioctl_names_t;

ioctl_names_t ioctl_names[]={
    {ADC_OPTIONS,"ADC_OPTIONS"},
    {ADC_START,"ADC_START"},            //Start using best of GPS and system time
    {ADC_STARTTIME,"ADC_STARTTIME"},
    {ADC_STOP,"ADC_STOP"},
    {ADC_START_SYS, "ADC_START_SYS"},   //Start using system time
    {ADC_START_GPS, "ADC_START_GPS"},   //Start using GPS time
    {0,"Unknown ioctl"}
};

char* find_ioctl_name(unsigned int cmd)
{
    int i=0;
    while (ioctl_names[i].cmd!=0)
    {
        if (ioctl_names[i].cmd == cmd)
            break;
        i++;
    }
    return ioctl_names[i].name;
}
#else
#define find_ioctl_name(cmd) ""
#endif
/*******************************************************************************************/
long bksv_adc_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct fsl_sai* sai = file->private_data;
    int res;

    printk("%s cmd=%x %s\n",__func__,cmd,find_ioctl_name(cmd));
    switch (cmd)
    {
        case ADC_OPTIONS:
        {
            adc_ioctl_options_t    *adc_options=(adc_ioctl_options_t*)arg;
            adc_ioctl_options_t old_opts;
            old_opts=sai->adc_options;
            if (sai->adc_options.samplefreq)
            {
                if (arm_copy_from_user(&sai->adc_options, adc_options, sizeof(adc_ioctl_options_t))!=0)
                    dev_err(&sai->pdev->dev, "arm_copy_from_user in ioctl=0x%x failed\n",cmd);
            }
            res=arm_copy_to_user(adc_options,&old_opts,sizeof(adc_ioctl_options_t));
            if (res < 0)
            {
                dev_err(&sai->pdev->dev, "bksv_adc_fop_ioctl: arm_copy_to_user failed res=%d\n",res);
                return res;
            }
            return 0;
        }
        break;

        case ADC_START:
        case ADC_START_SYS:
        case ADC_START_GPS:
        {
            long ret = fsl_sai_runtime_resume((struct device *)sai->pdev, cmd);
            if (ret)
                dev_err(&sai->pdev->dev, "fsl_sai_runtime_resume failed, cmd=0x%x ret=0x%x\n",cmd,ret);
            return ret;
        }
        break;

        case ADC_STARTTIME:
        {
            adc_ioctl_starttime_t *starttime=(adc_ioctl_starttime_t*)arg;
            return get_starttime(sai,starttime);
        }
        break;

        case ADC_STOP:
        {
            long ret = fsl_sai_runtime_suspend((struct device *)sai->pdev);
            if (ret)
                dev_err(&sai->pdev->dev, "fsl_sai_runtime_suspend failed, cmd=0x%x ret=0x%x\n",cmd,ret);
            return ret;
        }
        break;

        default:
            dev_err(&sai->pdev->dev, "unknown ioctl, cmd=0x%x\n",cmd);
        return -ENOTTY;
    }
    return 0;
}

int bksv_adc_fop_open(struct inode *inode, struct file *file)
{
    struct fsl_sai* sai;
    int err;
    int status = 0;
//    printk("%s\n",__func__);
    sai = container_of(inode->i_cdev, struct fsl_sai, cdev);
    file->private_data = sai;
	err = regulator_enable(ana_regulator); //inc. 100 ms delay
	if (err < 0)
		dev_err(&sai->pdev->dev,"%s: regulator enable, err=%d", __func__,err);
    else
        sai->psu_is_on = true;
    sai->is_open = 1;
    return status;
}

int bksv_adc_fop_close(struct inode *inode, struct file *file)
{
    struct fsl_sai* sai = file->private_data;
//    printk("%s\n",__func__);
    if (sai->psu_is_on)
    {
        int err;
        err = regulator_disable(ana_regulator);
        if (err < 0)
            dev_err(&sai->pdev->dev,"%s: regulator disable, err=%d", __func__ , err);
        else
            sai->psu_is_on = false;
    }

    sai->is_open = 0;
    return 0;
}

static int bksv_sai_mmap(struct file *info, struct vm_area_struct *vma)
{
    int ret;
    struct fsl_sai* sai = info->private_data;
    long length = vma->vm_end - vma->vm_start;
//    printk("%s\n",__func__);
    
    /* check length - do not allow larger mappings than the number of
       pages allocated */
    if (length > IMX_RXBD_NUM * PAGE_SIZE)
            return -EIO;
    /* map the whole physically contiguous area in one piece */
    if (vma->vm_pgoff == 0)
    {
        ret = dma_mmap_coherent(NULL, vma, sai->dma_buf.buf,
            sai->dma_buf.dmaaddr, length);
    }
    else
    {
        dev_err(&sai->pdev->dev, "Error: Using remap_pfn_range\n");
        ret = -1;
    }
    if (ret < 0)
    {
        dev_err(&sai->pdev->dev,"mmap_alloc: remap failed (%d)\n", ret);
        return ret;
    }
    return 0;
}

static const struct file_operations bksv_sai_fops = {
	.owner		= THIS_MODULE,
	.read		= bksv_adc_fop_read,
	.write		= bksv_adc_fop_write,
	.poll		= NULL,
	.unlocked_ioctl	= bksv_adc_fop_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= bksv_adc_fop_ioctl,
#endif
	.open		= bksv_adc_fop_open,
	.release	= bksv_adc_fop_close,
    .llseek		= NULL,
    .mmap       = bksv_sai_mmap,
};

static inline unsigned sai_imx_get_fifosize(struct fsl_sai *d)
{
	return  32;
}

static int bksv_adc_dma_init(struct fsl_sai *sai)
{
	struct dma_slave_config slave_config = {};
	struct device *dev = &sai->pdev->dev;
	int ret;
//    printk("%s\n",__func__);

	/* Prepare for RX : */
	sai->dma_chan = dma_request_slave_channel(dev, "rx");
    if (!sai->dma_chan)
    {
		dev_dbg(dev, "cannot get the DMA channel.\n");
		ret = -EINVAL;
		goto err;
	}
    
    slave_config.direction = DMA_DEV_TO_MEM;
    slave_config.device_fc = true;
	slave_config.src_addr = sai->res->start + FSL_SAI_RDR;     //"Read data" register
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.src_maxburst = FSL_SAI_MAXBURST_RX;
	ret = dmaengine_slave_config(sai->dma_chan, &slave_config);
    if (ret)
    {
		dev_err(dev, "error in RX dma configuration.\n");
		goto err;
	}
    
    //allocate dma buffer
    sai->dma_buf.buf = dma_alloc_coherent(NULL, IMX_RXBD_NUM * RX_BUF_SIZE,
	                                     &sai->dma_buf.dmaaddr, GFP_KERNEL);
    if (!sai->dma_buf.buf)
    {
		dev_err(dev, "cannot alloc DMA buffer.\n");
		ret = -ENOMEM;
		goto err;
	}
	sai->dma_is_inited = 1;

	return 0;
err:
	return ret;
}
static ssize_t get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
    int len = 3;
    if (pdev)
    {
     	struct fsl_sai *sai = platform_get_drvdata(pdev);
        if (sai)
        {
            struct timespec t;
            t = ns_to_timespec(ktime_to_ns(sai->start_clock));
            len=sprintf(buf,"%d %d %ld.%09ld", sai->start_count, sai->start_result, t.tv_sec, t.tv_nsec);
        }
        else
             strncpy(buf,"ups",4);
    }
    else
            strncpy(buf,"guf",4);
	return len;
}
static DEVICE_ATTR(source,                   S_IRUSR, get_status,           NULL);

static struct attribute *adc_attributes[] = {
     &dev_attr_source.attr,
     NULL
};
 
static const struct attribute_group adc_attr_group = {
     .attrs = adc_attributes,
};

static int bksv_sai_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_sai *sai;
	void __iomem *base;
	char tmp[8];
	int irq, ret=0, i;
	struct gpio_desc *led1;
 
//    printk("%s\n",__func__);
	sai = devm_kzalloc(&pdev->dev, sizeof(*sai), GFP_KERNEL);
	if (!sai)
		return -ENOMEM;

    sai->start_count = 0;   
    sai->start_result = 0;
    sai->pdev = pdev;
    sai->is_slave_mode = false;
    sai->psu_is_on     = false;
    sai->is_open       = false;
    sai->is_started    = false;
    sai->adc_options.samplefreq = 65536;
    sai->re_sync_running  = 0;
    sai->dev.driver_data = sai;

	if (of_device_is_compatible(pdev->dev.of_node, "bksv,imx7d-sai_adc"))
		sai->sai_on_imx = true;

	sai->is_lsb_first = of_property_read_bool(np, "lsb-first");

    //get addr of SAI registers
    sai->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, sai->res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ana_regulator = devm_regulator_get(&pdev->dev, "ana_on");
	if (ana_regulator == NULL) {
		dev_err(&pdev->dev,"%s regulator is null\n", __func__);
		return -1;
	}

    //map pin mux
    sai->iomux=(uint32_t *)ioremap_nocache(0x30330000,4096);
    if (sai->iomux==NULL)
    {
        printk("request_mem_region iomux failed\n");
		return PTR_ERR((void*)sai->iomux);
    }

    //map gpt
    sai->gpt=(uint32_t *)ioremap_nocache(0x302e0000,4096);
    if (sai->gpt==NULL)
    {
        printk("request_mem_region gpt failed\n");
		return PTR_ERR((void*)sai->gpt);
    }
    
    //map gpio1 
    sai->gpio1_addr=(struct mxc_gpio_hwdata *)ioremap_nocache(0x30200000,4096);
    if (sai->gpio1_addr==NULL)
    {
        dev_err(&sai->pdev->dev,"request_mem_region gpio1 failed\n");
		return PTR_ERR(sai->gpio1_addr);
    }
    sai->gpio1_addr->icr1_reg |= 0x000F0000; //Falling_edge on ovl high and low
    
    sai->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "bus", base, &fsl_sai_regmap_config);

    sai->iomux[MX7D_PAD_SAI1_MCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_MCLK] |=  0x5; //make GPIO
    sai->sai_mclk = devm_gpiod_get(&pdev->dev,"sai1_mclk",GPIOD_IN);
    if (IS_ERR(sai->sai_mclk))
    {
 		dev_err(&pdev->dev, "sai1_mclk not found in devicetree\n");
		return PTR_ERR(sai->sai_mclk);
    }
    
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] |=  0x5; //make GPIO
    sai->sai_bclk = devm_gpiod_get(&pdev->dev,"sai1_bclk",GPIOD_IN);
    if (IS_ERR(sai->sai_bclk))
    {
 		dev_err(&pdev->dev, "sai1_bclk not found in devicetree\n");
		return PTR_ERR(sai->sai_bclk);
    }
    
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] |=  0x5; //make GPIO
    sai->sai_sync = devm_gpiod_get(&pdev->dev,"sai1_sync",GPIOD_IN);
    if (IS_ERR(sai->sai_sync))
    {
 		dev_err(&pdev->dev, "sai1_sync not found in devicetree\n");
		return PTR_ERR(sai->sai_sync);
    }
    
    sai->led0=devm_gpiod_get(&pdev->dev,"led0",GPIOD_OUT_LOW);
    led1=devm_gpiod_get(&pdev->dev,"led1",GPIOD_OUT_HIGH);

    /* overload high gpio pin */
    sai->ovl_high_gpio = devm_gpiod_get(&pdev->dev,"ovl_high",GPIOD_IN);
    if (IS_ERR(sai->ovl_high_gpio))
    {
 		dev_err(&pdev->dev, "ovl_high not found in devicetree\n");
		return PTR_ERR(sai->ovl_high_gpio);
    }
    /* overload low gpio pin */
    sai->ovl_low_gpio = devm_gpiod_get(&pdev->dev,"ovl_low",GPIOD_IN);
    if (IS_ERR(sai->ovl_low_gpio))
    {
 		dev_err(&pdev->dev, "ovl_low not found in devicetree\n");
		return PTR_ERR(sai->ovl_low_gpio);
    }
    /* windscreen detect interrupt pin */
    sai->ana_int_gpio = devm_gpiod_get(&pdev->dev,"ana_int",GPIOD_IN);
    if (IS_ERR(sai->ana_int_gpio))
    {
 		dev_err(&pdev->dev, "ana_int dnot found in devicetree\n");
		return PTR_ERR(sai->ana_int_gpio);
    }
    /* master clock enable gpio pin */
    sai->clock_enable_gpio = devm_gpiod_get(&pdev->dev,"clock_enable",GPIOD_OUT_HIGH);
    if (IS_ERR(sai->clock_enable_gpio))
    {
 		dev_err(&pdev->dev, "clock_enable not found in devicetree\n");
		return PTR_ERR(sai->clock_enable_gpio);
    }

#if 0
    /* adc start gpio pin */
    sai->adc_start_gpio = devm_gpiod_get(&pdev->dev,"adc_start",GPIOD_OUT_LOW);
    if (IS_ERR(sai->adc_start_gpio))
    {
 		dev_err(&pdev->dev, "adc_start not found in devicetree\n");
		return PTR_ERR(sai->adc_start_gpio);
    }
#endif

	/* Compatible with old DTB cases */
	if (IS_ERR(sai->regmap))
		sai->regmap = devm_regmap_init_mmio_clk(&pdev->dev,
				"sai", base, &fsl_sai_regmap_config);
	if (IS_ERR(sai->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(sai->regmap);
	}

	/* No error out for old DTB cases but only mark the clock NULL */
	sai->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(sai->bus_clk)) {
		dev_err(&pdev->dev, "failed to get bus clock: %ld\n",
				PTR_ERR(sai->bus_clk));
		sai->bus_clk = NULL;
	}
	ret = clk_prepare_enable(sai->bus_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable bus clock: %d\n", ret);
		return ret;
	}
	sai->amclk = devm_clk_get(&pdev->dev, "amclk");
	if (IS_ERR(sai->amclk)) {
		if (PTR_ERR(sai->amclk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}

	for (i = 0; i < FSL_SAI_MCLK_MAX; i++) {
		sprintf(tmp, "mclk%d", i);
		sai->mclk_clk[i] = devm_clk_get(&pdev->dev, tmp);
		if (IS_ERR(sai->mclk_clk[i])) {
			dev_err(&pdev->dev, "failed to get mclk%d clock: %ld\n",
					i + 1, PTR_ERR(sai->mclk_clk[i]));
			sai->mclk_clk[i] = NULL;
		}
    }
    	/* Software Reset for both Tx and Rx */
	regmap_write(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR);
	regmap_write(sai->regmap, FSL_SAI_RCSR, FSL_SAI_CSR_SR);
	/* Clear SR bit to finish the reset */
	regmap_write(sai->regmap, FSL_SAI_TCSR, 0);
	regmap_write(sai->regmap, FSL_SAI_RCSR, 0);
	sai->slots = 2;
	sai->slot_width = 32;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for node %s\n", pdev->name);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, fsl_sai_isr, 0, np->name, sai);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %u\n", irq);
		return ret;
	}
    platform_set_drvdata(pdev, sai);
	pm_runtime_enable(&pdev->dev);
  	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
		dev_err(&pdev->dev, "%s: No suitable DMA available\n",__func__);
		return -EINVAL;
	}

    bksv_adc_dma_init(sai);
    init_completion(&sai->dma_completion);
    sai->device = device_create(bk_sys_class, &pdev->dev, MKDEV(MAJOR(adc_devt), 0),
                               (void *)sai, DEVICE_NAME_FORMAT, MINOR(MKDEV(MAJOR(adc_devt), 0)));
    //create char-device
    cdev_init(&sai->cdev, &bksv_sai_fops);
    sai->cdev.owner = THIS_MODULE;
    if (cdev_add(&sai->cdev, MKDEV(MAJOR(adc_devt), 0), 1)) 
    {
        dev_err(&pdev->dev, "unable to add adc c-device\n");
        goto err;
    }
    ret = sysfs_create_group(&pdev->dev.kobj, &adc_attr_group);

    bkadc_platform_driver_done = 1;
    dev_dbg(&pdev->dev,"ADC started\n");
    sai->active = 1;
    printk("%s end\n",__func__);
    return ret;
err:
	if (sai->dma_chan) {
		dma_release_channel(sai->dma_chan);
		sai->dma_chan = NULL;
	}
    return -1;
}

static const struct of_device_id bksv_sai_adc_ids[] = {
	{
      .compatible = "bksv,imx7d-sai_adc",
      .data = &adc_attr_group,
    },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bksv_sai_adc_ids);

#if 0
ssize_t format_timeval(struct timeval *tv, char *buf, size_t sz)
{
  ssize_t written = -1;
  struct tm *gm = gmtime(&tv->tv_sec);

  if (gm)
  {
    written = (ssize_t)strftime(buf, sz, "%Y-%m-%dT%H:%M:%S", gm);
    if ((written > 0) && ((size_t)written < sz))
    {
      int w = snprintf(buf+written, sz-(size_t)written, ".%06dZ", tv->tv_usec);
      written = (w > 0) ? written + w : -1;
    }
  }
  return written;
}
#endif


/*
start sampling is actualy = re-sync filters
sampling starts earlier, when the clock is turned on

*/
int start_sampling(struct fsl_sai *sai, int use_gps)
{
	unsigned int cr,cnt,ocr1,icr1;
//    struct timeval tval;
//    char buf[28];
    unsigned long flags;
    unsigned int oldcpsr=0,tmp_cpsr=0;
    unsigned int offset;
    int64_t diff;
    uint64_t cnt64;
 
    if (use_gps)
    {//start clock in the middle of a second
        cnt  = __raw_readl(sai->gpt + GPT_CNT);
        icr1 = __raw_readl(sai->gpt + GPT_ICR1);
        cnt64  = cnt;
        if (cnt < icr1)
            cnt64 += 0x100000000ll;
        diff = cnt64 - icr1;
        if (diff < (16777216 >> 1))
        {
            int diff32 = (int)diff;
            int ms = (((16777216 >> 1) - diff32)*60)/1000000;
            if (ms>10)
            {
//                printk("%s sleep %d ms\n",__func__,ms);
                msleep(ms);
            }
        }
    }

    sai->re_sync_running=1;
    //stop sampling
    cr = __raw_readl(sai->gpt + GPT_CR);
    cr &= ~GPT_CR_OM1_MASK;
    cr |= 2 << GPT_CR_OM1_SH;
    __raw_writel(cr, sai->gpt + GPT_CR);
    cr |= GPT_CR_FO1;
    __raw_writel(cr, sai->gpt + GPT_CR); //stop sampling now
    //start sampling
    cnt  = __raw_readl(sai->gpt + GPT_CNT);
    ocr1 = cnt - 1;
    __raw_writel(ocr1, sai->gpt + GPT_OCR1); //we have now 4 minutes to calculate a start time.
    cr = __raw_readl(sai->gpt + GPT_CR);
    cr &= ~GPT_CR_OM1_MASK;
    cr |= 3 << GPT_CR_OM1_SH;
    __raw_writel(cr, sai->gpt + GPT_CR); //start sampling at event
    offset=10000;
	spin_lock_irqsave(&gps_pps_lock, flags);
    //disable irq, rember old value
    //http://www.ethernut.de/en/documents/arm-inline-asm.html
	__asm__ __volatile__("mrs %[oldval], cpsr\n\t"
                         "orr %[tmp], %[oldval],  #0xC0\n\t"
                         "msr cpsr_c, %[tmp]\n\t" 
                         : "+X" (sai), [oldval] "=r" (oldcpsr), [tmp] "+r" (tmp_cpsr) 
                         :
                         : "cc");
    oldcpsr &= 0xC0; //only irq and fiq
    
    cnt  = __raw_readl(sai->gpt + GPT_CNT);
    icr1 = __raw_readl(sai->gpt + GPT_ICR1);
    sai->adc_start.wall_clock = ktime_get_real();
    if (icr1 != __raw_readl(sai->gpt + GPT_ICR1))
    {
        icr1  = __raw_readl(sai->gpt + GPT_ICR1);
        sai->adc_start.wall_clock = ktime_get_real();
    }
    //icr1 and wall_clock has now a good relation
    if (use_gps)
    {//a GPS pps has occoured 
        cnt64  = cnt;
        if (cnt < icr1)
            cnt64 += 0x100000000ll;
        diff = cnt64 - icr1;
        ocr1 = icr1;
        do                            //ocr at least 0.5 millisecond in the future
        {
            ocr1 += 16777216;
            diff -= 16777216;
        } while (diff > -10000);
    }
    else
        ocr1 = cnt + offset;
    __raw_writel(ocr1, sai->gpt + GPT_OCR1);
    if (use_gps)
    {
        memcpy(&local_gps_pps_data,&gps_pps_data,gps_pps_data.length+(sizeof(gps_pps_data_t)-(4*1024)));
    }
    //restore old irq value
	__asm__ __volatile__("mrs %[tmp], cpsr\n\t"
                         "bic %[tmp], %[tmp], #0xC0\n\t"
                         "orr %[tmp], %[oldval]\n"
                         "msr cpsr_c, %[tmp]" 
                         : [tmp] "+r" (tmp_cpsr)
                         : "X" (cnt), [oldval] "r" (oldcpsr)
                         : "cc");

 	spin_unlock_irqrestore(&gps_pps_lock, flags);
//    printk("cnt=%x ocr1=%x icr1=%x use_gps=%d\n",cnt,ocr1,icr1,use_gps);
    sai->adc_start.gpt_icr1=icr1;
    sai->adc_start.gpt_cnt =cnt;
    sai->adc_start.gpt_ocr1=ocr1;
    sai->adc_start.gps_present=use_gps;
    return 0;
}

/*******************************************************************************************/
static long get_starttime(struct fsl_sai *sai, adc_ioctl_starttime_t *starttime)
{
    struct timespec t;
    long long offset;
    unsigned char *pos=NULL;
    int res;
    uint64_t ocr64;

    sai->adc_start.ts1 = ns_to_timespec(ktime_to_ns(sai->adc_start.wall_clock));

    if(sai->adc_start.gps_present)
    {
        if (local_gps_pps_data.valid && (findubx(local_gps_pps_data.message, 0x01, 0x20,local_gps_pps_data.length,&pos)))
        {
    //        printk("%02x %02x %02x %02x %02x %02x  %02x %02x\n",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[22],pos[23]);
            if (ubx_checksum(pos,8+16))
            {
                sai->adc_start.gps_present=0;
            }
            else
            {
                unsigned char flags = pos[11+6];
                if ((flags & 0x7) != 0x7)
                {
                    sai->adc_start.gps_present=0;
                }
            }
        }
        else
            sai->adc_start.gps_present=0;
    }
//printk("present=%d dif=%u icr=%u cnt=%u\n",sai->adc_start.gps_present,diff,sai->adc_start.gpt_icr1,sai->adc_start.gpt_cnt);
//t = ns_to_timespec(ktime_to_ns(sai->adc_start.wall_clock));
//printk("sai->adc_start.wall_clock=%ld.%09ld\n",t.tv_sec,t.tv_nsec);
    if (sai->adc_start.gps_present == 0)
    {
        //add 10000 clocks 16.777.216 Hz = 596046.45 ns
        sai->adc_start.wall_clock = ktime_add_ns(sai->adc_start.wall_clock,596046); 
        t = ns_to_timespec(ktime_to_ns(sai->adc_start.wall_clock));
    }
    else
    {
        //find time from GPS message NAV-TIMEGPS (0x01 0x20)
        nav_timegps_t *nav_timegps=(nav_timegps_t*)(pos + 6);
        t.tv_sec  = GPS_EPOCH + (nav_timegps->week * SECS_PER_WEEK);
        t.tv_sec += nav_timegps->iTOW/1000;
        t.tv_nsec = ((nav_timegps->iTOW % 1000) * 1000000) + nav_timegps->fTOW;
        if (t.tv_nsec < 0)
        {
            t.tv_sec--;
            t.tv_nsec += 1000000000;
        }
        t.tv_sec -= nav_timegps->leapS;
//        printk("ubx tgps=%ld.%09ld\n",t.tv_sec,t.tv_nsec);
//printk("t=%d f=%d w=%d l=%d tgps=%ld.%09ld\n",nav_timegps->iTOW,nav_timegps->fTOW,nav_timegps->week, nav_timegps->leapS,t.tv_sec,t.tv_nsec);

        if (local_gps_pps_data.ppstime  != sai->adc_start.gpt_icr1)
        {
            uint32_t pps_icr1 = local_gps_pps_data.ppstime ;
            uint32_t gpt_icr1 = sai->adc_start.gpt_icr1;
            int diff_icr;

            diff_icr = pps_icr1 - gpt_icr1;
//printk("diff ppstime=%u icr1=%u diff_icr=%d\n",local_gps_pps_data.ppstime,sai->adc_start.gpt_icr1,diff_icr);
            if ((diff_icr > (3*16777216)) || (diff_icr < (-3*16777216)))
            {
                diff_icr = gpt_icr1 - pps_icr1;
                if ((diff_icr > (3*16777216)) || (diff_icr < (-3*16777216)))
                {
                    printk("diff_icr too big %d\n",diff_icr);
                }
                else
                {
                    if (diff_icr > 0)
                    {
                        do
                        {
                           t.tv_sec++;
                           diff_icr -= 16777216;
                        }while (diff_icr > 5000);
                    }
                    else
                    {
                        do
                        {
                           t.tv_sec--;
                           diff_icr += 16777216;
                        }while (diff_icr < -5000);
                    }
                }
            }
            else
            {
                if (diff_icr > 0)
                {
                    do
                    {
                        t.tv_sec--;
                        diff_icr -= 16777216;
                    }while (diff_icr > 5000);
                }
                else
                {
                    do
                    {
                        t.tv_sec++;
                        diff_icr += 16777216;
                    }while (diff_icr < -5000);
                }
            }
        }
        //t is now the time for latest pps at the time of setting ocr1
//        printk("tgps=%ld.%09ld\n",t.tv_sec,t.tv_nsec);
        
        //offset from latest pps to start signal at the time of setting ocr1
        ocr64=sai->adc_start.gpt_ocr1;
        if (sai->adc_start.gpt_ocr1 < sai->adc_start.gpt_icr1)
            ocr64 |= 0x100000000;
        offset = ocr64 - sai->adc_start.gpt_icr1; //offset in clocks
/*    
        printk("adc: uart icr =%u\n",local_gps_pps_data.ppstime);
        printk("adc: uart cnt1=%u\n",local_gps_pps_data.cnt1);
        printk("adc: uart cnt2=%u\n",local_gps_pps_data.cnt2);
        printk("adc: icr1     =%u\n",sai->adc_start.gpt_icr1);
        printk("adc: cnt      =%u\n",sai->adc_start.gpt_cnt);
        printk("adc: ocr1     =%u\n",sai->adc_start.gpt_ocr1);
        printk("adc: 1 offset =%lld\n",offset);
*/
        while (offset >= 16777216)
        {
            offset -= 16777216;
            t.tv_sec++;
        }
    if (offset) printk("adc: 2 offset=%lld\n",offset);
        offset = offset * ((1000000000ll<<10)/16777216ll); //now in ps
    if (offset)  printk("adc: 3 offset=%lld\n",offset);
        offset  = offset >> 10;                  //now in ns
    if (offset)  printk("adc: 4 offset=%lld\n",offset);
        if (offset > 1000000000)
        {
            t.tv_sec++;
            offset -= 1000000000;
        }
    if (offset)  printk("adc: 5 offset=%lld\n",offset);
        
        if (offset < 0)
        {
            t.tv_sec--;
            offset += 1000000000;
        }
    if (offset)  printk("adc: 6 offset=%lld\n",offset);
        
        t.tv_nsec += (long)offset;
    }
    printk("start tid=%ld.%09ld systemtime=%ld.%09ld gps_present=%d\n",t.tv_sec,t.tv_nsec,sai->adc_start.ts1.tv_sec,sai->adc_start.ts1.tv_nsec,sai->adc_start.gps_present);
    sai->adc_start.ts = t;
    res=arm_copy_to_user(starttime,&sai->adc_start,sizeof(adc_ioctl_starttime_t));
    if (res < 0)
    {
        dev_err(&sai->pdev->dev, "get_starttime: arm_copy_to_user failed res=%d\n",res);
        return res;
    }
    return 0;
}

//#ifdef CONFIG_PM
static int fsl_sai_runtime_resume(struct device *dev, unsigned int cmd)
{
	struct fsl_sai *sai;
	u32 word_width = 24;
    unsigned char *pos=NULL;
	int ret;
	u32 val,val_cr2 = 0, val_cr4 = 0, val_cr5 = 0;
	u32 slot_width = word_width;
//    printk("%s %s\n",__func__);
    int use_gps=time_server_useGPS && gps_pps_data.valid;

    sai = (struct fsl_sai *)dev_get_drvdata(dev);
    if (!sai)
    {
        sai = (struct fsl_sai *)platform_get_drvdata((struct platform_device *)dev);
    }

//    printk("%s 1\n",__func__);
    if (use_gps && !(findubx(gps_pps_data.message, 0x01, 0x20,gps_pps_data.length,&pos)))
    {
        printk(KERN_WARNING "Gps is active, but no UBX block found\n");
        use_gps = 0;
    }

    if ((cmd == ADC_START_GPS) && (use_gps == 0))
    {
        //Set variables so when SLM reads it knows that we failed
        sai->start_clock = ktime_get_real();
        sai->start_result = 0;
        sai->start_count++;

        printk(KERN_WARNING "GPS sync not possible servo: %d data %d\n",time_server_useGPS,gps_pps_data.valid);
        return -ESPIPE; //bad return code, to tell BK_Analog that we failed 
    }

    if (cmd == ADC_START_SYS)
        use_gps = 0;

    if (sai->is_started == true)  //start/restart needs clock off
	    fsl_sai_runtime_suspend(dev);

    sai->start_clock = ktime_get_real();
    sai->start_result = 1 + use_gps;
    sai->start_count++;

	request_bus_freq(BUS_FREQ_AUDIO);
   //enable m-clock to ADC        
    sai->iomux[MX7D_PAD_SAI1_MCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_MCLK] |=  0x0; //make mclk
    
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] |=  0x0; //make bclk
    
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] |=  0x0; //make sync

    while (try_wait_for_completion(&sai->dma_completion))
        ; //empty the queue from last run

    regmap_read(sai->regmap, FSL_SAI_RCSR, &val);
    if ((val & FSL_SAI_CSR_TERE) != 0)
        dev_err(&sai->pdev->dev, "enable bit allready set\n");
    regmap_read(sai->regmap, FSL_SAI_RFR, &val);
//    dev_warn(&sai->pdev->dev, "before reset FSL_SAI_FR=%x\n",val);

    	/* Software Reset for both Tx and Rx */
	regmap_write(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR);
	regmap_write(sai->regmap, FSL_SAI_RCSR, FSL_SAI_CSR_SR);
	/* Clear SR bit to finish the reset */
	regmap_write(sai->regmap, FSL_SAI_TCSR, 0);
	regmap_write(sai->regmap, FSL_SAI_RCSR, 0);
//    regmap_read(sai->regmap, FSL_SAI_RFR, &val);
    if (!sai->is_slave_mode)
    {
		slot_width = sai->slot_width;
		ret = fsl_sai_set_bclk(sai, false,	sai->slots * slot_width * sai->adc_options.samplefreq);
		if (ret)
        {
            printk("%s 1\n",__func__);
			return ret;
        }

        ret = clk_prepare_enable(sai->mclk_clk[sai->mclk_id[0]]);
        if (ret)
        {
            printk("%s 2\n",__func__);
			return ret;
        }

        ret = clk_prepare_enable(sai->amclk);
        if (ret)
        {
            printk("%s 3\n",__func__);
			return ret;
        }

	}
//    dev_err(&sai->pdev->dev, "after reset FSL_SAI_FR=%x\n",val);
    val_cr2 |= FSL_SAI_CR2_BCD_MSTR | FSL_SAI_CR2_BCP;
	val_cr4 |= FSL_SAI_CR4_FSD_MSTR;
    val_cr4 |= FSL_SAI_CR4_MF;
	val_cr4 |= FSL_SAI_CR4_FRSZ(2);   //2 words in a frame
	val_cr4 |= FSL_SAI_CR4_SYWD(32);  //32 bits in a word
    val_cr5 |= FSL_SAI_CR5_WNW(32);   //32 bit in a word
    val_cr5 |= FSL_SAI_CR5_W0W(32);   //32 bit in first word
    val_cr5 |= FSL_SAI_CR5_FBT(31);
    regmap_update_bits(sai->regmap, FSL_SAI_RCR1, FSL_SAI_CR1_RFW_MASK, FSL_SAI_MAXBURST_RX - 1);
    regmap_update_bits(sai->regmap, FSL_SAI_RCR2, FSL_SAI_CR2_BCP | FSL_SAI_CR2_BCD_MSTR, val_cr2);
    regmap_update_bits(sai->regmap, FSL_SAI_RCR3, FSL_SAI_CR3_TRCE | FSL_SAI_CR3_WDFL_MASK,
                                    FSL_SAI_CR3_TRCE);
    regmap_update_bits(sai->regmap, FSL_SAI_RCR4,
                     FSL_SAI_CR4_MF | FSL_SAI_CR4_FSE |  FSL_SAI_CR4_FSP |
                     FSL_SAI_CR4_FSD_MSTR |FSL_SAI_CR4_FRSZ_MASK |FSL_SAI_CR4_SYWD_MASK,
                     val_cr4);
 	regmap_update_bits(sai->regmap, FSL_SAI_RCR5,
			         FSL_SAI_CR5_WNW_MASK | FSL_SAI_CR5_W0W_MASK | FSL_SAI_CR5_FBT_MASK,
                     val_cr5);

    start_rx_dma(sai);
    gpiod_set_value(sai->led0, 1); 

    regmap_update_bits(sai->regmap, FSL_SAI_RCSR,
                        FSL_SAI_CSR_FRDE, FSL_SAI_CSR_FRDE|FSL_SAI_CSR_FR);

    regmap_update_bits(sai->regmap, FSL_SAI_RCSR, FSL_SAI_CSR_TERE, FSL_SAI_CSR_TERE);
    regmap_update_bits(sai->regmap, FSL_SAI_RCSR, FSL_SAI_CSR_xIE_MASK, FSL_SAI_FLAGS);
    gpiod_set_value(sai->led0, 0); 
    //clocks must be running before start of ADC
    //it takes aprox 1955us to get out of power down
    udelay(2000);
    udelay(2000); //more wait to avoid mixing power up and start
    udelay(2000);
    udelay(2000);
    gpiod_set_value(sai->led0, 1); 
    start_sampling(sai, use_gps);
    gpiod_set_value(sai->led0, 0); 
//    gpiod_set_value(sai->adc_start_gpio, 1); //start sampling
    sai->is_started = true;

	return 0;
}

static int fsl_sai_runtime_suspend(struct device *dev)
{
	unsigned int cr;
    struct fsl_sai *sai;
    unsigned int val,timeout;
    uint32_t cnt,icr1;

    sai = (struct fsl_sai *)dev_get_drvdata(dev);
    if (!sai)
    {
        sai = (struct fsl_sai *)platform_get_drvdata((struct platform_device *)dev);
    }

    if (sai->is_started == false)
	    return 0;

    cnt  = __raw_readl(sai->gpt + GPT_CNT);
    icr1 = __raw_readl(sai->gpt + GPT_ICR1);
//printk("stop cnt=%u icr1=%u\n",cnt,icr1);
    //stop sampling
    cr = __raw_readl(sai->gpt + GPT_CR);
    cr &= ~GPT_CR_OM1_MASK;
    cr |= 2 << GPT_CR_OM1_SH;
    __raw_writel(cr, sai->gpt + GPT_CR);
    cr |= GPT_CR_FO1;
    __raw_writel(cr, sai->gpt + GPT_CR); //stop sampling
//     gpiod_set_value(sai->adc_start_gpio, 0); //stop sampling

   //Stop SAI interface
    regmap_update_bits(sai->regmap, FSL_SAI_RCSR,
                FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_xIE_MASK, 0);
    for (timeout=0; timeout<15; timeout++)
    {//wait for the enable bit to clear, when current frame is finished
       	regmap_read(sai->regmap, FSL_SAI_RCSR, &val);
        if ((val & FSL_SAI_CSR_TERE) == 0)
            break;
        usleep_range(500, 2000);
    }

    if (sai->dma_is_runing)
    {
        dmaengine_terminate_sync(sai->dma_chan);
        sai->dma_is_runing = 0;
    }
    sai->iomux[MX7D_PAD_SAI1_MCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_MCLK] |=  0x5; //make GPIO
    gpiod_direction_input(sai->sai_mclk);
     
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_BCLK] |=  0x5; //make GPIO
    gpiod_direction_input(sai->sai_bclk);
   
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] &= ~0x7;
    sai->iomux[MX7D_PAD_SAI1_RX_SYNC] |=  0x5; //make GPIO
    gpiod_direction_input(sai->sai_sync);

    clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[0]]);
    clk_disable_unprepare(sai->amclk);
	release_bus_freq(BUS_FREQ_AUDIO);
    sai->is_started    = false;
	return 0;
}
//#endif


static int bksv_sai_adc_remove(struct platform_device *pdev)
{
    unsigned int cr;
	struct fsl_sai *sai = platform_get_drvdata(pdev);

//    printk("%s active=%d\n",__func__,sai->active);
    if (sai->active == 0)
    {
        return 0;
    }
    complete(&sai->dma_completion); //to end BK_Analog waiting in a read call

    //stop sampling
    cr = __raw_readl(sai->gpt + GPT_CR);
    cr &= ~GPT_CR_OM1_MASK;
    cr |= 2 << GPT_CR_OM1_SH;
    __raw_writel(cr, sai->gpt + GPT_CR);
    cr |= GPT_CR_FO1;
    __raw_writel(cr, sai->gpt + GPT_CR); //stop sampling
//    gpiod_set_value(sai->adc_start_gpio, 0); //stop sampling

    //Stop SAI interface
    regmap_update_bits(sai->regmap, FSL_SAI_RCSR,
                FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_xIE_MASK, 0);
    if (sai->dma_is_runing)
    {
        dmaengine_terminate_all(sai->dma_chan);
        sai->dma_is_runing = 0;
    }
    if (sai->psu_is_on)
    {
        int err;
        err = regulator_disable(ana_regulator);
        if (err < 0)
            dev_err(&pdev->dev, "%s: regulator disable, err=%d", __func__,err);
        else
            sai->psu_is_on = false;
    }
    sai->active = 0;
    sysfs_remove_group(&sai->pdev->dev.kobj, &adc_attr_group);
	return 0;
}
static void bksv_sai_adc_shutdown(struct platform_device *pdev)
{
    printk("%s\n",__func__);
    bksv_sai_adc_remove(pdev);
}
static struct platform_driver bksv_sai_adc_driver = {
	.probe    = bksv_sai_adc_probe,
	.remove	  = bksv_sai_adc_remove,
	.shutdown = bksv_sai_adc_shutdown,
	.driver = {
        .owner = THIS_MODULE,
		.name = "bksv-sai_adc",
		.of_match_table = bksv_sai_adc_ids,
	},
};

static void __exit sai_adc_module_exit(void)
{
//    printk("%s\n",__func__);
#if 0    
    if (sai->psu_is_on)
    {
        int err;
        err = regulator_disable(ana_regulator);
        if (err < 0)
            printk("%s: regulator disable, err=%d", __func__,err);
        else
            sai->psu_is_on = false;
    }
#endif
    if (bkadc_platform_driver_done ){platform_driver_unregister(&bksv_sai_adc_driver);}
    if (bk_sys_class     != NULL)   {class_destroy(bk_sys_class);bk_sys_class=NULL;}
    if (adc_devt != 0)              {unregister_chrdev_region(adc_devt, 0);}
}


static int __init sai_adc_module_init(void)
{
   int retval = 0;
     
    retval = alloc_chrdev_region(&adc_devt, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number\n", DRIVER_NAME);
        adc_devt = 0;
        goto failed;
    }

    if (bk_sys_class == NULL)
    {
        bk_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
        if (IS_ERR_OR_NULL(bk_sys_class)) {
            printk(KERN_ERR "%s: couldn't create sys class\n", DRIVER_NAME);
            retval = PTR_ERR(bk_sys_class);
            bk_sys_class = NULL;
            goto failed;
        }
    }
    retval = platform_driver_register(&bksv_sai_adc_driver);
    if (retval) {
        printk(KERN_ERR "%s: couldn't register platform driver\n", DRIVER_NAME);
    } 
   return 0;

failed:
   sai_adc_module_exit();
   return retval;
}

module_init(sai_adc_module_init);
module_exit(sai_adc_module_exit);

MODULE_DESCRIPTION("Brüel & Kjær SAI Interface");
MODULE_AUTHOR("Lars Thestrup");
MODULE_ALIAS("platform:bksv-sai");
MODULE_LICENSE("GPL");
