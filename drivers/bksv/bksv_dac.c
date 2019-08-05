/*
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
#include <linux/regulator/consumer.h>

#include "bksv_sai.h"

#define FSL_SAI_FLAGS (FSL_SAI_CSR_SEIE |\
                       FSL_SAI_CSR_FEIE |\
                       FSL_SAI_CSR_FWIE )

#define DRIVER_NAME        "bkdac"
#define DEVICE_NAME_FORMAT "bkdac%d"
extern struct class *bk_sys_class;
static dev_t dac_devt = 0;
static bool bkdac_platform_driver_done = 0;
static struct regulator *ana_regulator = NULL;
static int fsl_sai_runtime_resume(struct device *dev);
static int fsl_sai_runtime_suspend(struct device *dev);

#if 1
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
        xcsr &= ~(FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_FWIE);
        sai->dma_buf.underrun = true;
        complete(&sai->dma_completion);
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
		dev_dbg(dev, "isr: Start of Rx word detected\n");

	if (flags & FSL_SAI_CSR_SEF)
		dev_dbg(dev, "isr: Rx Frame sync error detected\n");

	if (flags & FSL_SAI_CSR_FEF) {
		dev_dbg(dev, "isr: Receive overflow detected\n");
		/* FIFO reset for safety */
		xcsr |= FSL_SAI_CSR_FR;
	}

	if (flags & FSL_SAI_CSR_FWF)
		dev_dbg(dev, "isr: Enabled receive FIFO is full\n");

	if (flags & FSL_SAI_CSR_FRF)
		dev_dbg(dev, "isr: Receive FIFO watermark has been reached\n");

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
#endif

static int fsl_sai_set_bclk(struct fsl_sai *sai, bool tx, u32 freq)
{
//	struct fsl_sai *sai = snd_soc_dai_get_drvdata(dai);
	unsigned long clk_rate;
	u32 savediv = 0, ratio, savesub = freq;
	u32 id;
	int ret = 0;

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
			sai->mclk_id[(tx==0)?0:1] = id;
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

static void dma_tx_callback(void *data)
{
	struct fsl_sai *sai = data;
    uint32_t val;
    unsigned int next;
    
	sai->dma_buf.last_completed_idx++;
	sai->dma_buf.last_completed_idx %= IMX_TXBD_NUM;
    sai->dma_buf.buf_info[sai->dma_buf.last_completed_idx].filled = false;
    next=(sai->dma_buf.last_completed_idx + 1) % IMX_TXBD_NUM;
    
    if (sai->dma_buf.buf_info[next].filled == false)
    {
        if (sai->dma_buf.underrun == false)
        {
            sai->dma_buf.underrun = true;
            dev_err(&sai->pdev->dev, "DAC underrun! buf=%d\n",sai->dma_buf.cur_idx);
            regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_xIE_MASK, 0);
            dmaengine_terminate_all(sai->dma_chan);
            //transmit value to get zero on output
            do
            {
                regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
            }
            while ((val & FSL_SAI_CSR_FRF)==0);
            for (val = 0; val < 32; val++)
                regmap_write(sai->regmap, FSL_SAI_TDR, 0);
            do
            {
                regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
            }
            while ((val & FSL_SAI_CSR_FRF) == 0);
            //remove m-clock to power down DAC        
            sai->iomux[75] &= ~0x7;
            sai->iomux[75] |=  0x5; //make GPIO
            gpiod_set_value(sai->sai_mclk, 1);
            gpiod_set_value(sai->sai_mclk, 0);
            
            regmap_update_bits(sai->regmap, FSL_SAI_TCSR,
                FSL_SAI_CSR_BCE|FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE,0);
            udelay(200);//stop all
            regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR, FSL_SAI_CSR_SR);
            udelay(200);//CSR_SR = Software Reset
            regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR, 0);
            gpiod_set_value(sai->gen_disable_gpio,  1); //mute amplifier
            sai->dma_is_runing = 0;
        }
    }
    
    complete(&sai->dma_completion);
}

static int start_tx_dma(struct fsl_sai *sai)
{
	struct dma_chan	*chan = sai->dma_chan;
	struct dma_async_tx_descriptor *desc;
    
    sai->dma_buf.periods    = IMX_TXBD_NUM;
	sai->dma_buf.period_len = TX_BUF_SIZE;
    sai->dma_buf.buf_len    = IMX_TXBD_NUM * TX_BUF_SIZE;
    sai->dma_buf.underrun   = false;
	desc = dmaengine_prep_dma_cyclic(chan, sai->dma_buf.dmaaddr,
		                      sai->dma_buf.buf_len, sai->dma_buf.period_len,
		                      DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
    if (!desc)
    {
		dev_err(&sai->pdev->dev, "Prepare for the RX slave dma failed!\n");
		return -EINVAL;
	}

	desc->callback = dma_tx_callback;
	desc->callback_param = sai;

	dev_dbg(&sai->pdev->dev, "RX: prepare for the DMA.\n");
	sai->dma_buf.cookie = dmaengine_submit(desc);
	dma_async_issue_pending(chan);
    
	sai->dma_is_runing = 1;
	return 0;
}

/* bksv_dac_fop_read:
   return number of buffers to be filled;
   wait if no non-filled buffers exists.
   return -1 if DAC-underrun.
*/
ssize_t bksv_dac_fop_read(struct file *file,
    char __user *buffer,
    size_t length,
    loff_t *ppos)
{
    struct fsl_sai* sai = file->private_data;
    int buf[100],i;
    unsigned int count;

    while (try_wait_for_completion(&sai->dma_completion))
    {//remove "allready done completions" from the callback function
    }; 
    count = 0;
    for (i = 0; i < IMX_TXBD_NUM; i++)
    {//count empty buffers
        if (sai->dma_buf.buf_info[i].filled==false)
            count++;
    }
    while (count == 0)
    {//wait for next completion
        wait_for_completion(&sai->dma_completion);
        for (i = 0; i < IMX_TXBD_NUM; i++)
        {//count empty buffers
            if (sai->dma_buf.buf_info[i].filled==false)
                count++;
        }
    }

    if ((count == IMX_TXBD_NUM) || sai->dma_buf.underrun)
    {
        dev_err(&sai->pdev->dev, "dac-underrun! buf=%d\n",sai->dma_buf.cur_idx);
        buf[0]=-1;
        sai->dma_buf.underrun = false;
    }
    else
        buf[0]=count;
    if (arm_copy_to_user(buffer,buf,(1)*sizeof(int)) != 0)
     return 0;
    else
     return (1)*sizeof(int);
}

ssize_t bksv_dac_fop_write(struct file *file, const char __user *buffer,
    size_t length, loff_t *ppos)
{
    struct fsl_sai* sai = file->private_data;
    int buf[10],i,next_buf,stop_buf;

    if (copy_from_user(buf,buffer,4) == 0)
    {//buf[0] == last buf filled by user program
        next_buf = buf[0]+1;
        if (buf[0] < sai->dma_buf.cur_idx)
            stop_buf = next_buf+IMX_TXBD_NUM;
        else
            stop_buf = next_buf;
        for (i=sai->dma_buf.cur_idx; i!=stop_buf; i++)
        {
            sai->dma_buf.buf_info[i % IMX_TXBD_NUM].filled = true;
        }
        sai->dma_buf.cur_idx = next_buf % IMX_TXBD_NUM;  //next to be filled
    }

    if (sai->dma_is_runing == 0)
    {
        regmap_update_bits(sai->regmap, FSL_SAI_TCSR,FSL_SAI_CSR_FR, FSL_SAI_CSR_FR);
        start_tx_dma(sai);
        regmap_write(sai->regmap, FSL_SAI_TDR, 0);  //prefill
        regmap_write(sai->regmap, FSL_SAI_TDR, 0);  //prefill
                                    
        regmap_update_bits(sai->regmap, FSL_SAI_TCSR,
                        FSL_SAI_CSR_BCE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_FRDE,
                        FSL_SAI_CSR_BCE|FSL_SAI_CSR_TERE|FSL_SAI_CSR_FRDE);
        regmap_update_bits(sai->regmap, FSL_SAI_TCSR,
                        FSL_SAI_CSR_xIE_MASK, FSL_SAI_FLAGS);
//        ReadMem();
    }
    return length;
}

long bksv_dac_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return 0;
}

int bksv_dac_fop_open(struct inode *inode, struct file *file)
{
    struct fsl_sai* sai;
    int status = 0;
    sai = container_of(inode->i_cdev, struct fsl_sai, cdev);
    file->private_data = sai;
    fsl_sai_runtime_resume((struct device *)sai->pdev);
    sai->is_open = 1;
    return status;
}

int bksv_dac_fop_close(struct inode *inode, struct file *file)
{
    struct fsl_sai* sai = file->private_data;
    printk("%s start\n",__func__);
    fsl_sai_runtime_suspend((struct device *)sai->pdev);
    sai->is_open = 0;
    printk("%s end\n",__func__);
    return 0;
}

static int bksv_sai_mmap(struct file *info, struct vm_area_struct *vma)
{
    int ret;
    struct fsl_sai* sai = info->private_data;
    long length = vma->vm_end - vma->vm_start;
    
    /* check length - do not allow larger mappings than the number of
       pages allocated */
    if (length > IMX_TXBD_NUM * PAGE_SIZE)
            return -EIO;
    /* map the whole physically contiguous area in one piece */
    if (vma->vm_pgoff == 0)
    {
//        printk("Using dma_mmap_coherent\n");
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
	.read		= bksv_dac_fop_read,
	.write		= bksv_dac_fop_write,
	.poll		= NULL,
	.unlocked_ioctl	= bksv_dac_fop_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= bksv_dac_fop_ioctl,
#endif
	.open		= bksv_dac_fop_open,
	.release	= bksv_dac_fop_close,
    .llseek		= NULL,
    .mmap       = bksv_sai_mmap,
};

static inline unsigned sai_imx_get_fifosize(struct fsl_sai *d)
{
	return  32;
}

static int bksv_dac_dma_init(struct fsl_sai *sai)
{
	struct dma_slave_config slave_config = {};
	struct device *dev = &sai->pdev->dev;
	int ret;

	/* Prepare for RX : */
	sai->dma_chan = dma_request_slave_channel(dev, "tx");
    if (!sai->dma_chan)
    {
		dev_dbg(dev, "cannot get the DMA channel.\n");
		ret = -EINVAL;
		goto err;
	}
    
    slave_config.direction = DMA_MEM_TO_DEV;
    slave_config.device_fc = true;
	slave_config.dst_addr = sai->res->start + FSL_SAI_TDR;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.dst_maxburst = FSL_SAI_MAXBURST_TX;//sai_imx_get_fifosize(sai) / 2;
	ret = dmaengine_slave_config(sai->dma_chan, &slave_config);
    if (ret)
    {
		dev_err(dev, "error in RX dma configuration.\n");
		goto err;
	}
    
	sai->dma_buf.buf = dma_alloc_coherent(NULL, IMX_TXBD_NUM * TX_BUF_SIZE,
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

static int bksv_sai_dac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_sai *sai;
	void __iomem *base;
	char tmp[8];
	int irq, ret=0, i;

	sai = devm_kzalloc(&pdev->dev, sizeof(*sai), GFP_KERNEL);
	if (!sai)
		return -ENOMEM;

	sai->pdev = pdev;
    sai->is_slave_mode = false;
    sai->psu_is_on     = false;
    sai->is_open       = false;

	if (of_device_is_compatible(pdev->dev.of_node, "bksv,imx7d-sai_dac"))
		sai->sai_on_imx = true;

	sai->is_lsb_first = of_property_read_bool(np, "lsb-first");

    //get addr of SAI registers
	sai->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, sai->res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ana_regulator = devm_regulator_get(&pdev->dev, "ana_on");
	if (ana_regulator == NULL) {
		dev_err(&pdev->dev, "%s regulator is null\n", __func__);
		return -1;
	}

    //map pin mux
    sai->iomux = (uint32_t *) ioremap_nocache(0x30330000,0x4096);
    if (sai->iomux == NULL)
    {
        dev_err(&pdev->dev, "request_mem_region iomux failed\n");
		return PTR_ERR((void*)sai->iomux);
    }
        
	sai->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "bus", base, &fsl_sai_regmap_config);

    /* generator disable gpio pin */
    sai->gen_disable_gpio = devm_gpiod_get(&pdev->dev,"gen_disable",GPIOD_IN); //pull up on ana board disables gen
    if (IS_ERR(sai->gen_disable_gpio))
    {
 		dev_err(&pdev->dev, "gen_disable not found in devicetree\n");
		return PTR_ERR(sai->gen_disable_gpio);
    }

    sai->iomux[MX7D_PAD_UART1_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART1_TX_DATA] |=  0x5; //make GPIO
    sai->sai_mclk = devm_gpiod_get(&pdev->dev,"sai3_mclk",GPIOD_IN);
    if (IS_ERR(sai->sai_mclk))
    {
 		dev_err(&pdev->dev, "sai3_mclk not found in devicetree\n");
		return PTR_ERR(sai->sai_mclk);
    }
    
    sai->iomux[MX7D_PAD_UART3_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_TX_DATA] |=  0x5; //make GPIO
    sai->sai_bclk = devm_gpiod_get(&pdev->dev,"sai3_bclk",GPIOD_IN);
    if (IS_ERR(sai->sai_bclk))
    {
 		dev_err(&pdev->dev, "sai3_bclk not found in devicetree\n");
		return PTR_ERR(sai->sai_bclk);
    }
    
    sai->iomux[MX7D_PAD_UART3_CTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_CTS_B] |=  0x5; //make GPIO
    sai->sai_sync = devm_gpiod_get(&pdev->dev,"sai3_sync",GPIOD_IN);
    if (IS_ERR(sai->sai_sync))
    {
 		dev_err(&pdev->dev, "sai3_sync not found in devicetree\n");
		return PTR_ERR(sai->sai_sync);
    }
    
    sai->iomux[MX7D_PAD_UART3_RTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_RTS_B] |=  0x5; //make GPIO
    sai->sai_data = devm_gpiod_get(&pdev->dev,"sai3_data",GPIOD_IN);
    if (IS_ERR(sai->sai_data))
    {
 		dev_err(&pdev->dev, "sai3_data not found in devicetree\n");
		return PTR_ERR(sai->sai_data);
    }
    
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
    
    bksv_dac_dma_init(sai);
    init_completion(&sai->dma_completion);

    sai->device = device_create(bk_sys_class, &pdev->dev, MKDEV(MAJOR(dac_devt), 0),
                               (void *)sai, DEVICE_NAME_FORMAT, MINOR(MKDEV(MAJOR(dac_devt), 0)));
                               
    //create char-device
    cdev_init(&sai->cdev, &bksv_sai_fops);
    sai->cdev.owner = THIS_MODULE;
    
    if (cdev_add(&sai->cdev, MKDEV(MAJOR(dac_devt), 0), 1)) 
    {
        dev_err(&pdev->dev, "unable to add dac c-device\n");
        goto err;
    }
    bkdac_platform_driver_done = 1;
    sai->active = 1;
    printk("DAC probed\n");
    
    return 0;
    err:
	if (sai->dma_chan) {
		dma_release_channel(sai->dma_chan);
		sai->dma_chan = NULL;
	}
    return -1;
 }

static const struct of_device_id bksv_sai_dac_ids[] = {
	{ .compatible = "bksv,imx7d-sai_dac", }, 
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bksv_sai_dac_ids);

#ifdef CONFIG_PM
static int fsl_sai_runtime_resume(struct device *dev)
{
	struct fsl_sai *sai;
	u32 val_cr2 = 0, val_cr4 = 0, val_cr5 = 0;
	u32 word_width = 24;//params_width(params);
	u32 slot_width = word_width;
    int err,ret,i;

	request_bus_freq(BUS_FREQ_AUDIO);

    sai = (struct fsl_sai *)dev_get_drvdata(dev);
    if (!sai)
    {
        sai = (struct fsl_sai *)platform_get_drvdata((struct platform_device *)dev);
    }

	err = regulator_enable(ana_regulator);
	if (err < 0)
		dev_err(dev,"%s: regulator enable, err=%d", __func__,err);
    else
        sai->psu_is_on = true;
   //enable m-clock to DAC        
    sai->iomux[MX7D_PAD_UART1_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART1_TX_DATA] |=  0x2; //make sai_mclk

    sai->iomux[MX7D_PAD_UART3_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_TX_DATA] |=  0x2; //make bclk
    gpiod_direction_input(sai->sai_bclk);
    
    sai->iomux[MX7D_PAD_UART3_CTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_CTS_B] |=  0x2; //make sync
    
    sai->iomux[MX7D_PAD_UART3_RTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_RTS_B] |=  0x2; //make data

 	if (!sai->is_slave_mode) {
		slot_width = sai->slot_width;
		ret = fsl_sai_set_bclk(sai, true,	sai->slots * slot_width * 65536/*params_rate(params)*/);

		if (ret)
			return ret;

        ret = clk_prepare_enable(sai->mclk_clk[sai->mclk_id[1]]);
        if (ret)
            return ret;

        ret = clk_prepare_enable(sai->amclk);
        if (ret)
            return ret;
	}

    val_cr2 |= FSL_SAI_CR2_BCD_MSTR;
    val_cr2 |= FSL_SAI_CR2_BCP;       //Bit Clock Polarity
	val_cr4 |= FSL_SAI_CR4_FSD_MSTR;  //Frame Sync Direction
    val_cr4 |= FSL_SAI_CR4_MF;        //MSB First
    val_cr4 |= FSL_SAI_CR4_FSP;       //Frame Sync Polarity, active low
    val_cr4 |= FSL_SAI_CR4_FSE;       //Frame Sync Early
	val_cr4 |= FSL_SAI_CR4_FRSZ(2);   //2 words in a frame
	val_cr4 |= FSL_SAI_CR4_SYWD(32);  //32 bits in a word
    val_cr5 |= FSL_SAI_CR5_WNW(32);   //32 bit in a word
    val_cr5 |= FSL_SAI_CR5_W0W(32);   //32 bit in first word
    val_cr5 |= FSL_SAI_CR5_FBT(31);

    regmap_update_bits(sai->regmap, FSL_SAI_TCR1, FSL_SAI_CR1_RFW_MASK,FSL_SAI_MAXBURST_TX * 2);
    regmap_update_bits(sai->regmap, FSL_SAI_TCR2, FSL_SAI_CR2_BCP | FSL_SAI_CR2_BCD_MSTR, val_cr2);
    regmap_update_bits(sai->regmap, FSL_SAI_TCR3, FSL_SAI_CR3_TRCE | FSL_SAI_CR3_WDFL_MASK,
                                    FSL_SAI_CR3_TRCE);
    regmap_update_bits(sai->regmap, FSL_SAI_TCR4,
                     FSL_SAI_CR4_MF | FSL_SAI_CR4_FSE |  FSL_SAI_CR4_FSP |
                     FSL_SAI_CR4_FSD_MSTR |FSL_SAI_CR4_FRSZ_MASK |FSL_SAI_CR4_SYWD_MASK,
                     val_cr4);
 	regmap_update_bits(sai->regmap, FSL_SAI_TCR5,
			         FSL_SAI_CR5_WNW_MASK | FSL_SAI_CR5_W0W_MASK | FSL_SAI_CR5_FBT_MASK,
                     val_cr5);
    sai->dma_buf.cur_idx=0;
	sai->dma_buf.last_completed_idx = -1;
    for (i = 0; i < IMX_TXBD_NUM; i++)
    {
       sai->dma_buf.buf_info[i].filled = false;
	}
    sai->iomux[MX7D_PAD_UART1_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART1_TX_DATA] |=  0x2; //make sai_mclk

    while (try_wait_for_completion(&sai->dma_completion))
    {}; //empty the queue from last run

    gpiod_direction_output(sai->gen_disable_gpio,0);

	return 0;
}

static int fsl_sai_runtime_suspend(struct device *dev)
{
	struct fsl_sai *sai;
    uint32_t val,timeout,i;
    
    sai = (struct fsl_sai *)dev_get_drvdata(dev);
    if (!sai)
    {
        sai = (struct fsl_sai *)platform_get_drvdata((struct platform_device *)dev);
    }
    gpiod_direction_input(sai->gen_disable_gpio);

    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_xIE_MASK, 0);
    if (sai->dma_is_runing)
    {
        dmaengine_terminate_all(sai->dma_chan);
        //wait for all data to bee send
        do
        {
            regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
        }
        while ((val & FSL_SAI_CSR_FRF) == 0);
    }
    sai->dma_is_runing = 0;
    //stop transmitter
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR,FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE,0);
    do
    {//wait until stopped
        regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
    }
    while (val & FSL_SAI_CSR_TERE);
   
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_FR, FSL_SAI_CSR_FR); //fifo reset
    for (i = 0; i < 32; i++)
        regmap_write(sai->regmap, FSL_SAI_TDR, 0); //send zeros
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_FEF, FSL_SAI_CSR_FEF); //reset fifo error flag

    //start transmitter
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR,FSL_SAI_CSR_TERE,FSL_SAI_CSR_TERE);
    timeout=1000;
    do
    {
        regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
    }
    while (((val & FSL_SAI_CSR_FRF) == 0) && (timeout-- > 0));
    
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR,
        FSL_SAI_CSR_BCE|FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE,0);
    msleep(1);
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR, FSL_SAI_CSR_SR);
    msleep(1);
    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR, 0);

    //remove m-clock to power down DAC        
    sai->iomux[MX7D_PAD_UART1_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART1_TX_DATA] |=  0x5; //make GPIO
    gpiod_direction_output(sai->sai_mclk, 1);
    gpiod_set_value(sai->sai_mclk, 0);

    sai->iomux[MX7D_PAD_UART3_TX_DATA] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_TX_DATA] |=  0x5; //make GPIO
    gpiod_direction_output(sai->sai_bclk, 0);
    
    sai->iomux[MX7D_PAD_UART3_CTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_CTS_B] |=  0x5; //make GPIO
    gpiod_direction_output(sai->sai_sync, 0);
    
    sai->iomux[MX7D_PAD_UART3_RTS_B] &= ~0x7;
    sai->iomux[MX7D_PAD_UART3_RTS_B] |=  0x5; //make GPIO
    gpiod_direction_output(sai->sai_data, 0);

    if (sai->psu_is_on)
    {
        int err;
        err = regulator_disable(ana_regulator);
        if (err < 0)
            dev_err(dev, "%s: regulator disable, err=%d", __func__,err);
        else
            sai->psu_is_on = false;
    }
    clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[1]]);
    clk_disable_unprepare(sai->amclk);

	release_bus_freq(BUS_FREQ_AUDIO);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int fsl_sai_suspend(struct device *dev)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);

	regcache_cache_only(sai->regmap, true);
	regcache_mark_dirty(sai->regmap);

	return 0;
}

static int fsl_sai_resume(struct device *dev)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);

	regcache_cache_only(sai->regmap, false);
	regmap_write(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_SR);
	regmap_write(sai->regmap, FSL_SAI_RCSR, FSL_SAI_CSR_SR);
	msleep(1);
	regmap_write(sai->regmap, FSL_SAI_TCSR, 0);
	regmap_write(sai->regmap, FSL_SAI_RCSR, 0);
	return regcache_sync(sai->regmap);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_sai_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_sai_runtime_suspend,fsl_sai_runtime_resume,NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_sai_suspend, fsl_sai_resume)
};

static int bksv_sai_dac_remove(struct platform_device *pdev)
{
	struct fsl_sai *sai = platform_get_drvdata(pdev);
    printk("%s active=%d\n",__func__,sai->active);
    if (sai->active == 0)
    {
        return 0;
    }
    complete(&sai->dma_completion); //to end BK_Analog waiting in a read call

    regmap_update_bits(sai->regmap, FSL_SAI_TCSR, FSL_SAI_CSR_xIE_MASK, 0);
    if (sai->dma_is_runing)
    {
        uint32_t val;
        dmaengine_terminate_all(sai->dma_chan);
       //transmit value to get zero on output
        do
        {
            regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
        }
        while ((val & FSL_SAI_CSR_FRF)==0);
        for (val=0;val<32;val++)
            regmap_write(sai->regmap, FSL_SAI_TDR, 0);
        do
        {
            regmap_read(sai->regmap, FSL_SAI_TCSR, &val);
        }
        while ((val & FSL_SAI_CSR_FRF)==0);
        //remove m-clock to power down DAC        
        sai->iomux[75] &= ~0x7;
        sai->iomux[75] |=  0x5; //make GPIO
        gpiod_set_value(sai->sai_mclk, 1);
        gpiod_set_value(sai->sai_mclk, 0);
        
        regmap_update_bits(sai->regmap, FSL_SAI_TCSR,
            FSL_SAI_CSR_BCE|FSL_SAI_CSR_FRDE|FSL_SAI_CSR_TERE,0);
        gpiod_set_value(sai->gen_disable_gpio,  1); //mute amplifier
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
	return 0;
}

static void bksv_sai_dac_shutdown(struct platform_device *pdev)
{
    printk("%s\n",__func__);
    bksv_sai_dac_remove(pdev);
}

static struct platform_driver bksv_sai_dac_driver = {
	.probe    = bksv_sai_dac_probe,
	.remove   = bksv_sai_dac_remove,
	.shutdown = bksv_sai_dac_shutdown,
	.driver = {
        .owner = THIS_MODULE,
		.name = "bksv-sai_dac",
		.pm = &fsl_sai_pm_ops,
		.of_match_table = bksv_sai_dac_ids,
	},
};

static void __exit sai_dac_module_exit(void)
{
    if (bkdac_platform_driver_done ){platform_driver_unregister(&bksv_sai_dac_driver);}
    if (bk_sys_class     != NULL)   {class_destroy(bk_sys_class);}
    if (dac_devt != 0)              {unregister_chrdev_region(dac_devt, 0);}
}


static int __init sai_dac_module_init(void)
{
    int retval = 0;
        
    retval = alloc_chrdev_region(&dac_devt, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number\n", DRIVER_NAME);
        dac_devt = 0;
        goto failed;
    }

    if (bk_sys_class==NULL)
    {
        bk_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
        if (IS_ERR_OR_NULL(bk_sys_class)) {
            printk(KERN_ERR "%s: couldn't create sys class\n", DRIVER_NAME);
            retval = PTR_ERR(bk_sys_class);
            bk_sys_class = NULL;
            goto failed;
        }
    }
    retval = platform_driver_register(&bksv_sai_dac_driver);
    if (retval) {
        printk(KERN_ERR "%s: couldn't register platform driver\n", DRIVER_NAME);
    } 
    return 0;

failed:
   sai_dac_module_exit();
   return retval;
}

module_init(sai_dac_module_init);
module_exit(sai_dac_module_exit);
