#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <asm/uaccess.h>



#define DRIVER_NAME        "bksv_timer"
#define DEVICE_NAME_FORMAT "bksv_timer%d"
extern struct class *bk_sys_class;
static dev_t timer_devt = 0;

#define BKTIMER_SOURCE _IOW(0xB5, 20, int)

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
struct bksv_timer_driver_t
{
	struct device		dev;
	struct platform_device *pdev;
	struct regmap *regmap;
    struct device *device;
	struct cdev cdev;
	struct resource *res;
    struct completion gpt_completion;
    uint64_t icr1;
    uint64_t icr2;
    uint64_t count64;
    char source[4];
    int  debug;

    ktime_t wall_clock;
};

static struct bksv_timer_driver_t *bksv_timer_driver=NULL;
//int pps_gpt_event_handler(uint64_t count64, uint64_t icr2);

//used in bksv_adc.c
int time_server_useGPS = 0; //global vaiable

struct bksv_timer_interface_t
{
    int status_ok;
    uint64_t icr1;
    uint64_t icr2;
    uint64_t count64;
    int debug;

    struct timespec wall_clock;
};

static void __iomem *timer_base=NULL;

static ssize_t get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
    if (pdev)
    {
        struct bksv_timer_driver_t *bksv_timer_driver = platform_get_drvdata(pdev);
        if (bksv_timer_driver)
            strncpy(buf,bksv_timer_driver->source,4);
        else
             strncpy(buf,"ups",4);
    }
    else
            strncpy(buf,"guf",4);
	return 3;
}
static ssize_t get_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bksv_timer_driver_t *bksv_timer_driver = platform_get_drvdata(pdev);
	return sprintf(buf, "%d\n", bksv_timer_driver->debug);
}

static ssize_t set_debug(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bksv_timer_driver_t *bksv_timer_driver = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    bksv_timer_driver->debug=bit;
    return count;
}


static DEVICE_ATTR(source,                   S_IRUSR, get_status,           NULL);
static DEVICE_ATTR(debug,            S_IWUSR|S_IRUSR, get_debug,       set_debug);

static struct attribute *bksv_timer_attributes[] = {
     &dev_attr_source.attr,
     &dev_attr_debug.attr,
     NULL
};
 
static const struct attribute_group bksv_timer_attr_group = {
     .attrs = bksv_timer_attributes,
};

static inline void gpt_irq_disable(void)
{
	__raw_writel(0, timer_base + GPT_IR);
}

static inline void gpt_irq_enable(void)
{
	__raw_writel(GPT_IR_IF2IE | GPT_IR_ROVIE, timer_base + GPT_IR);
}

static void gpt_irq_acknowledge(void)
{
	__raw_writel(0x3F, timer_base + GPT_SR);
}


static int __init mxc_clocksource_init(struct clk *timer_clk)
{
	unsigned int c = clk_get_rate(timer_clk);
	void __iomem *reg = timer_base + GPT_CNT;

	pr_info("mxc_clocksource_init %d\n", c);
	return clocksource_mmio_init(reg, "mxc_timer2", c, 200, 32, clocksource_mmio_readl_up);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t mxc_timer_interrupt(int irq, void *dev_id)
{
    struct bksv_timer_driver_t *bksv_timer_driver = (struct bksv_timer_driver_t *)dev_id;
	uint32_t tstat;
    uint32_t cnt,ir,cr;
    uint32_t *tid64 = (uint32_t*) &bksv_timer_driver->count64;
    uint32_t *icr164 = (uint32_t*) &bksv_timer_driver->icr1;
    uint32_t *icr264 = (uint32_t*) &bksv_timer_driver->icr2;
    static uint32_t old_icr1 = 0;
    static uint32_t old_icr2 = 0;

    if (bksv_timer_driver->debug == 2)
    {
        bksv_timer_driver->wall_clock = ktime_get_real();
    }
	cnt   = __raw_readl(timer_base + GPT_CNT);
	cr    = __raw_readl(timer_base + GPT_CR);
	tstat = __raw_readl(timer_base + GPT_SR);
	ir    = __raw_readl(timer_base + GPT_IR);
    tid64[0]=cnt;

    if (tstat & GPT_SR_ROV)
    {
        tid64[1]++;                                 //counter has rollover
    }

    if (tstat & GPT_SR_IF1)
    {//got 1PPS from GPS
        icr164[0]  = __raw_readl(timer_base + GPT_ICR1);
        if (((old_icr1 & 0xF0000000) == 0xF0000000)  && ((icr164[0] & 0xF0000000) != 0xF0000000))//has icr1 a rollover
            icr164[1] = tid64[1];
        old_icr1 = icr164[0];
    }
    
    if (tstat & GPT_SR_IF2)
    {//got 1PPS from RTC
        icr264[0]  = __raw_readl(timer_base + GPT_ICR2);
        if (((old_icr2 & 0xF0000000) == 0xF0000000)  && ((icr264[0] & 0xF0000000) != 0xF0000000))//has icr1 a rollover
            icr264[1] = tid64[1];
        old_icr2 = icr264[0];
        //pps_gpt_event_handler(bksv_timer_driver->count64, bksv_timer_driver->icr2);
        complete(&bksv_timer_driver->gpt_completion);
     }

#if 0
    if (tstat & GPT_SR_IF2)
    	printk("GPT_SR_IF2: count: 0x%llx, icr1: 0x%llx tstat=%x ir=%x bksv_timer_driver=%p\n", bksv_timer_driver->count64, bksv_timer_driver->icr1,tstat,ir,bksv_timer_driver);
 #endif 
    gpt_irq_acknowledge();
	return IRQ_HANDLED;
}

static struct irqaction mxc_timer_irq = {
	.name		= "i.MX Timer Tick",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= mxc_timer_interrupt,
};

static void __init _mxc_timer_init(int irq,
                   struct clk *clk_per, struct clk *clk_ipg,
                   struct bksv_timer_driver_t *bksv_timer_driver)
{
    uint32_t tctl_val;
	unsigned int cr;

    if (IS_ERR(clk_per)) {
		pr_err("i.MX timer: unable to get clk\n");
		return;
	}

	if (!IS_ERR(clk_ipg))
		clk_prepare_enable(clk_ipg);
	clk_prepare_enable(clk_per);
	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */

	__raw_writel(0, timer_base + GPT_CR);
	__raw_writel(0, timer_base + GPT_PR); /* see datasheet note */
	__raw_writel(0x3F, timer_base + GPT_SR);
    
    tctl_val = GPT_CR_FRR | GPT_CR_WAITEN | GPT_CR_EN | (2 << GPT_CR_IM2_SH) | (1 << GPT_CR_IM1_SH);
    if (clk_get_rate(clk_per) == V2_TIMER_RATE_OSC_DIV8)
    {
        dev_dbg(bksv_timer_driver->device, "%s rate\n",__func__);
        tctl_val |= GPT_CR_CLK_OSC_DIV8;
        /* 24 / 8 = 3 MHz */
        __raw_writel(7 << GPT_PR_PRE24M_SH,timer_base + GPT_PR);
        tctl_val |= GPT_CR_EN_24M;
    }
    else
    {
        dev_dbg(bksv_timer_driver->device, "%s per\n",__func__);
        tctl_val |= GPT_CR_CLK_PER;
    }

	__raw_writel(tctl_val, timer_base + GPT_CR);

	mxc_clocksource_init(clk_per);

    cr = __raw_readl(timer_base + GPT_CR);
    cr &= ~GPT_CR_OM1_MASK;
    cr |= 2 << GPT_CR_OM1_SH;
    __raw_writel(cr, timer_base + GPT_CR);
    cr |= GPT_CR_FO1;
    __raw_writel(cr, timer_base + GPT_CR); //stop sampling

    /* Make irqs happen */
    mxc_timer_irq.dev_id = (void*)bksv_timer_driver;
	setup_irq(irq, &mxc_timer_irq);
    gpt_irq_enable();
}

ssize_t bksv_timer_fop_read(struct file *file,
    char __user *buffer,
    size_t length,
    loff_t *ppos)
{
    struct bksv_timer_interface_t buf;
    
    struct bksv_timer_driver_t *bksv_timer_driver = file->private_data;
    if (wait_for_completion_timeout(&bksv_timer_driver->gpt_completion,msecs_to_jiffies(2000))==0)
    {
        buf.status_ok=0;
    }
    else
    {
        buf.status_ok=1;
        buf.icr1=bksv_timer_driver->icr1;
        buf.icr2=bksv_timer_driver->icr2;
        if (bksv_timer_driver->debug == 2)
            buf.wall_clock = ns_to_timespec(ktime_to_ns(bksv_timer_driver->wall_clock));
        else
        {
            buf.wall_clock.tv_sec  = 0;
            buf.wall_clock.tv_nsec = 0;
        }
    }
    buf.count64=bksv_timer_driver->count64;
    buf.debug = bksv_timer_driver->debug;

    if (arm_copy_to_user(buffer,&buf,sizeof(struct bksv_timer_interface_t)) != 0)
    {
        dev_err(bksv_timer_driver->device, "arm_copy_to_user failed\n");
        return 0;
    }
    else
        return sizeof(struct bksv_timer_interface_t);
}

ssize_t bksv_timer_fop_write(struct file *file, const char __user *buffer,
    size_t length, loff_t *ppos)
{
  //  struct bksv_timer_driver_t *bksv_timer_driver = file->private_data;
    return 0;
}

int bksv_timer_fop_open(struct inode *inode, struct file *file)
{
    struct bksv_timer_driver_t *bksv_timer_driver;
    int status = 0;
    
    bksv_timer_driver = container_of(inode->i_cdev, struct bksv_timer_driver_t, cdev);
    file->private_data = bksv_timer_driver;
    while (try_wait_for_completion(&bksv_timer_driver->gpt_completion))
    {}; //empty the queue from last run
    return status;
}

int bksv_timer_fop_close(struct inode *inode, struct file *file)
{
//    struct bksv_timer_driver_t *bksv_timer_driver = file->private_data;
    printk("%s\n",__func__);
    return 0;
}

long bksv_timer_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct bksv_timer_driver_t *bksv_timer_driver = file->private_data;
    static int old_source=-1;
    int source_changed=0;
    if (arg != old_source)
    {
        source_changed = 1;
        old_source = arg;
    }
    switch (cmd)
    {
        case    BKTIMER_SOURCE:
            switch (arg)
            {
                case 0:  strncpy(bksv_timer_driver->source,"RTC",4); break;
                case 1:  strncpy(bksv_timer_driver->source,"GPS",4); break;
                case 2:  strncpy(bksv_timer_driver->source,"MAL",4); break;
                default: strncpy(bksv_timer_driver->source,"???",4); break;
            }
            break;
    }
    if (source_changed)
        printk("%s servo source changed to %s\n",__func__,bksv_timer_driver->source);

    if (arg == 1)
    {
        time_server_useGPS = true;
    }
    else
    {
        time_server_useGPS = false;
    }
    return 0;
}

static const struct file_operations bksv_timer_fops = {
	.owner		= THIS_MODULE,
	.read		= bksv_timer_fop_read,
	.write		= bksv_timer_fop_write,
	.poll		= NULL,
	.unlocked_ioctl	= bksv_timer_fop_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= bksv_timer_fop_ioctl,
#endif
	.open		= bksv_timer_fop_open,
	.release	= bksv_timer_fop_close,
    .llseek		= NULL,
    .mmap       = NULL,
};

static int bksv_timer_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk *clk_per, *clk_ipg;
	int irq,ret=0;
	void __iomem *base;

	bksv_timer_driver = devm_kzalloc(&pdev->dev, sizeof(struct bksv_timer_driver_t), GFP_KERNEL);
	if (!bksv_timer_driver)
		return -ENOMEM;
    bksv_timer_driver->dev.driver_data = bksv_timer_driver;
 	platform_set_drvdata(pdev, bksv_timer_driver);
    strncpy(bksv_timer_driver->source,"RTC",4);  
    //get addr of GPT registers
	bksv_timer_driver->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, bksv_timer_driver->res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (timer_base)
		return 0;
    bksv_timer_driver->pdev = pdev;
    timer_base = of_iomap(np, 0);
	WARN_ON(!timer_base);
	irq = irq_of_parse_and_map(np, 0);
    
	clk_ipg = of_clk_get_by_name(np, "ipg");

	/* Try osc_per first, and fall back to per otherwise */
	clk_per = of_clk_get_by_name(np, "osc_per");
	if (IS_ERR(clk_per))
		clk_per = of_clk_get_by_name(np, "per");
    bksv_timer_driver->count64 = 0;
    bksv_timer_driver->icr1    = 0;
    init_completion(&bksv_timer_driver->gpt_completion);
	_mxc_timer_init(irq, clk_per, clk_ipg,bksv_timer_driver);

    //create char-device
    bksv_timer_driver->device = device_create(bk_sys_class, &pdev->dev, MKDEV(MAJOR(timer_devt), 0),
                  (void *)bksv_timer_driver, DEVICE_NAME_FORMAT, MINOR(MKDEV(MAJOR(timer_devt), 0)));

    cdev_init(&bksv_timer_driver->cdev, &bksv_timer_fops);
    bksv_timer_driver->cdev.owner = THIS_MODULE;
    
    if (cdev_add(&bksv_timer_driver->cdev, MKDEV(MAJOR(timer_devt), 0), 1)) 
    {
        dev_err(&pdev->dev, "unable to add timer c-device\n");
        goto err;
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &bksv_timer_attr_group);
    dev_info(&pdev->dev, "bksv_timer started ret=%d\n",ret);
err:
    return ret;
}

static const struct of_device_id bksv_timer_of_match[] = {
    {
        .compatible = "bksv_timer",
        .data = &bksv_timer_attr_group,
    }, 
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bksv_timer_ids);

static int bksv_timer_remove(struct platform_device *pdev )
{
	struct bksv_timer_driver_t *bksv_timer_driver = platform_get_drvdata(pdev);
    sysfs_remove_group(&bksv_timer_driver->dev.kobj, &bksv_timer_attr_group);
    return 0;
}

static struct platform_driver bksv_timer_platform_driver = {
	.probe  = bksv_timer_probe,
	.remove = bksv_timer_remove,
	.driver = {
        .name = DRIVER_NAME,
		.of_match_table = of_match_ptr(bksv_timer_of_match),
        .pm = NULL,
	},
};

static int __init bksv_timer_module_init(void)
{
    int retval = 0;    
        
    retval = alloc_chrdev_region(&timer_devt, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number\n", DRIVER_NAME);
        timer_devt = 0;
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
 
    retval = platform_driver_register(&bksv_timer_platform_driver);
    if (retval)
    {
        printk(KERN_ERR "%s: couldn't register platform driver\n", DRIVER_NAME);
        goto failed;
    } 
    return 0;

failed:
   return retval;
}
//module_init(bksv_timer_module_init);
subsys_initcall(bksv_timer_module_init);

static void __exit bksv_timer_exit(void)
{
	platform_driver_unregister(&bksv_timer_platform_driver);
}
module_exit(bksv_timer_exit);

MODULE_AUTHOR("Lars Thestrup <LarsErling.Thestrup@bksv.com>");
MODULE_DESCRIPTION("GTP for SXU");
MODULE_LICENSE("GPL v2");
