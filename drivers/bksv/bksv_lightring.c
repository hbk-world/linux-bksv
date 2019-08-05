/*
 * leds-tca6507
 *
 * The TCA6507 is a programmable LED controller that can drive 7
 * separate lines either by holding them low, or by pulsing them
 * with modulated width.
 * The modulation can be varied in a simple pattern to produce a
 * blink or double-blink.
 *
 * This driver can configure each line either as a 'GPIO' which is
 * out-only (pull-up resistor required) or as an LED with variable
 * brightness and hardware-assisted blinking.
 *
 * Apart from OFF and ON there are three programmable brightness
 * levels which can be programmed from 0 to 15 and indicate how many
 * 500usec intervals in each 8msec that the led is 'on'.  The levels
 * are named MASTER, BANK0 and BANK1.
 *
 * There are two different blink rates that can be programmed, each
 * with separate time for rise, on, fall, off and second-off.  Thus if
 * 3 or more different non-trivial rates are required, software must
 * be used for the extra rates. The two different blink rates must
 * align with the two levels BANK0 and BANK1.  This driver does not
 * support double-blink so 'second-off' always matches 'off'.
 *
 * Only 16 different times can be programmed in a roughly logarithmic
 * scale from 64ms to 16320ms.  To be precise the possible times are:
 *    0, 64, 128, 192, 256, 384, 512, 768,
 *    1024, 1536, 2048, 3072, 4096, 5760, 8128, 16320
 *
 * Times that cannot be closely matched with these must be handled in
 * software.  This driver allows 12.5% error in matching.
 *
 * This driver does not allow rise/fall rates to be set explicitly.
 * When trying to match a given 'on' or 'off' period, an appropriate
 * pair of 'change' and 'hold' times are chosen to get a close match.
 * If the target delay is even, the 'change' number will be the
 * smaller; if odd, the 'hold' number will be the smaller.

 * Choosing pairs of delays with 12.5% errors allows us to match
 * delays in the ranges: 56-72, 112-144, 168-216, 224-27504,
 * 28560-36720.
 * 26% of the achievable sums can be matched by multiple pairings.
 * For example 1536 == 1536+0, 1024+512, or 768+768.
 * This driver will always choose the pairing with the least
 * maximum - 768+768 in this case.  Other pairings are not available.
 *
 * Access to the 3 levels and 2 blinks are on a first-come,
 * first-served basis.  Access can be shared by multiple leds if they
 * have the same level and either same blink rates, or some don't
 * blink.  When a led changes, it relinquishes access and tries again,
 * so it might lose access to hardware blink.
 *
 * If a blink engine cannot be allocated, software blink is used.  If
 * the desired brightness cannot be allocated, the closest available
 * non-zero brightness is used.  As 'full' is always available, the
 * worst case would be to have two different blink rates at '1', with
 * Max at '2', then other leds will have to choose between '2' and
 * '16'.  Hopefully this is not likely.
 *
 * Each bank (BANK0 and BANK1) has two usage counts - LEDs using the
 * brightness and LEDs using the blink.  It can only be reprogrammed
 * when the appropriate counter is zero.  The MASTER level has a
 * single usage count.
 *
 * Each LED has programmable 'on' and 'off' time as milliseconds.
 * With each there is a flag saying if it was explicitly requested or
 * defaulted.  Similarly the banks know if each time was explicit or a
 * default.  Defaults are permitted to be changed freely - they are
 * not recognised when matching.
 *
 *
 * An led-tca6507 device must be provided with platform data or
 * configured via devicetree.
 *
 * The platform-data lists for each output: the name, default trigger,
 * and whether the signal is being used as a GPIO rather than an LED.
 * 'struct led_plaform_data' is used for this.  If 'name' is NULL, the
 * output isn't used.  If 'flags' is TCA6507_MAKE_GPIO, the output is
 * a GPO.  The "struct led_platform_data" can be embedded in a "struct
 * lr_tca6507_platform_data" which adds a 'gpio_base' for the GPIOs, and
 * a 'setup' callback which is called once the GPIOs are available.
 *
 * When configured via devicetree there is one child for each output.
 * The "reg" determines the output number and "compatible" determines
 * whether it is an LED or a GPIO.  "linux,default-trigger" can set a
 * default trigger.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
//#include <linux/leds-tca6507.h>
#include <linux/of.h>
#include <linux/sched.h>   //wake_up_process() 
#include <linux/kthread.h> //kthread_create(), kthread_run() 
#include <asm/uaccess.h>

#define DRIVER_NAME        "lightring"
#define DEVICE_NAME_FORMAT "lightring"
extern struct class *bk_sys_class;
static dev_t ring_devt = 0;
/* LED select registers determine the source that drives LED outputs */
#define TCA6507_LS_LED_OFF	0x0	/* Output HI-Z (off) */
#define TCA6507_LS_LED_OFF1	0x1	/* Output HI-Z (off) - not used */
#define TCA6507_LS_LED_PWM0	0x2	/* Output LOW with Bank0 rate */
#define TCA6507_LS_LED_PWM1	0x3	/* Output LOW with Bank1 rate */
#define TCA6507_LS_LED_ON	0x4	/* Output LOW (on) */
#define TCA6507_LS_LED_MIR	0x5	/* Output LOW with Master Intensity */
#define TCA6507_LS_BLINK0	0x6	/* Blink at Bank0 rate */
#define TCA6507_LS_BLINK1	0x7	/* Blink at Bank1 rate */

struct lr_tca6507_platform_data {
	struct led_platform_data leds;
    int rgb[3][2];
};

enum {
	BANK0,
	BANK1,
	MASTER,
};
static int bank_source[3] = {
	TCA6507_LS_LED_PWM0,
	TCA6507_LS_LED_PWM1,
	TCA6507_LS_LED_MIR,
};
/*
static int blink_source[2] = {
	TCA6507_LS_BLINK0,
	TCA6507_LS_BLINK1,
};
*/
/* PWM registers */
#define	TCA6507_REG_CNT			11

/*
 * 0x00, 0x01, 0x02 encode the TCA6507_LS_* values, each output
 * owns one bit in each register
 */
#define	TCA6507_FADE_ON			0x03
#define	TCA6507_FULL_ON			0x04
#define	TCA6507_FADE_OFF		0x05
#define	TCA6507_FIRST_OFF		0x06
#define	TCA6507_SECOND_OFF		0x07
#define	TCA6507_MAX_INTENSITY		0x08
#define	TCA6507_MASTER_INTENSITY	0x09
#define	TCA6507_INITIALIZE		0x0A

#define	INIT_CODE			0x8

/*#define TIMECODES 16
static int time_codes[TIMECODES] = {
	0, 64, 128, 192, 256, 384, 512, 768,
	1024, 1536, 2048, 3072, 4096, 5760, 8128, 16320
};*/

/* Convert an led.brightness level (0..255) to a TCA6507 level (0..15) */
static inline int TO_LEVEL(int brightness)
{
	return brightness >> 4;
}

/* ...and convert back */
static inline int TO_BRIGHT(int level)
{
	if (level)
		return (level << 4) | 0xf;
	return 0;
}

#pragma pack(push,1)
struct blink_setup_t
{
    struct color1
    {
        unsigned char red;   //values truncatede to 0,16,32,64,80,96 ...
        unsigned char green;
        unsigned char blue;
                 int  time; // time in milliseconds, -1 for infinity
    } color1;

    struct color2
    {
        unsigned char red;
        unsigned char green;
        unsigned char blue;
                 int  time; // time in milliseconds, -1 for infinity
    } color2;
    unsigned int sync_metod; //0 = now, 1 color1 starts at a whole second, 5 color1 starts at a 5 second change
};
#pragma pack(pop)

#define LR_SET_COLOR _IOWR(0xB5,10, struct blink_setup_t)

#define NUM_LEDS 7
struct tca6507_chip {
	int			reg_set;	/* One bit per register where
						 * a '1' means the register
						 * should be written */
	u8			reg_file[TCA6507_REG_CNT];
	/* Bank 2 is Master Intensity and doesn't use times */
	struct bank {
		int level;
	} bank[3];
	struct i2c_client	*client;
	struct work_struct	work;
    struct device *device;
	struct cdev cdev;
    spinlock_t		lock;
    int rgb[3][2];
    struct blink_setup_t blink_setup;
	struct tca6507_led {
		struct tca6507_chip	*chip;
		struct led_classdev	led_cdev;
		int			num;
		int			bank;	/* Bank used */
	} leds[NUM_LEDS+1];
    struct task_struct *blink_task;
};

static const struct i2c_device_id tca6507_id[] = {
	{ "tca6507" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tca6507_id);


/*
 * Update the register file with the appropriate 3-bit state for the
 * given led.
 */
static void set_select(struct tca6507_chip *tca, int led, int val)
{
	int mask = (1 << led);
	int bit;

	for (bit = 0; bit < 3; bit++) {
		int n = tca->reg_file[bit] & ~mask;
		if (val & (1 << bit))
			n |= mask;
		if (tca->reg_file[bit] != n) {
			tca->reg_file[bit] = n;
			tca->reg_set |= (1 << bit);
		}
	}
}

/* Update the register file with the appropriate 4-bit code for one
 * bank or other.  This can be used for timers, for levels, or for
 * initialization.
 */
static void set_code(struct tca6507_chip *tca, int reg, int bank, int new)
{
	int mask = 0xF;
	int n;
	if (bank) {
		mask <<= 4;
		new <<= 4;
	}
	n = tca->reg_file[reg] & ~mask;
	n |= new;
	if (tca->reg_file[reg] != n) {
		tca->reg_file[reg] = n;
		tca->reg_set |= 1 << reg;
	}
}

/* Update brightness level. */
static void set_level(struct tca6507_chip *tca, int bank, int level)
{
	switch (bank) {
	case BANK0:
	case BANK1:
		set_code(tca, TCA6507_MAX_INTENSITY, bank, level);
		break;
	case MASTER:
		set_code(tca, TCA6507_MASTER_INTENSITY, 0, level);
		break;
	}
	tca->bank[bank].level = level;
}


/* Write all needed register of tca6507 */

static void tca6507_work(struct work_struct *work)
{
	struct tca6507_chip *tca = container_of(work, struct tca6507_chip,
						work);
	struct i2c_client *cl = tca->client;
	int set;
	u8 file[TCA6507_REG_CNT];
	int r;

	spin_lock_irq(&tca->lock);
	set = tca->reg_set;
	memcpy(file, tca->reg_file, TCA6507_REG_CNT);
	tca->reg_set = 0;
	spin_unlock_irq(&tca->lock);

	for (r = 0; r < TCA6507_REG_CNT; r++)
    {
//        printk("out r=%x v=%x\n",r,file[r]);
		if (set & (1<<r))
			i2c_smbus_write_byte_data(cl, r, file[r]);
    }
}
static void init_regs(struct tca6507_chip *tca)
{
    int r;
	struct i2c_client *cl = tca->client;

	for (r = 0; r < TCA6507_REG_CNT; r++)
    {
        tca->reg_file[r] = i2c_smbus_read_byte_data(cl, r);
//        printk("init r=%x v=%x\n",r,tca->reg_file[r]);
    }
    tca->reg_file[TCA6507_MASTER_INTENSITY] &= 0x0F; //all four mode bits = 0;
}

static void tca6507_brightness_set(struct tca6507_chip *tca, int color, unsigned char brightness)
{
	struct tca6507_led *led;
    int i,level;
    level = TO_LEVEL(brightness);
    for (i=0;i<2;i++)
    {
        led = &tca->leds[tca->rgb[color][i]];
        led->led_cdev.brightness = brightness;
        switch (level)
        {
            case 15:
                set_select(tca, led->num, TCA6507_LS_LED_ON);
                break;
            case 0:
        		set_select(tca, led->num, TCA6507_LS_LED_OFF);
                break;
            default:
                set_level(tca, color, level);
                set_select(tca, led->num, bank_source[color]);
            break;
        }
    }
}

static struct lr_tca6507_platform_data *
tca6507_led_dt_init(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node, *child;
	struct lr_tca6507_platform_data *pdata;
	struct led_info *tca_leds;
	int count, count_red,count_green,count_blue;

	count_red = count_green = count_blue = 0;
    count = of_get_child_count(np);
	if (!count || count > NUM_LEDS)
		return ERR_PTR(-ENODEV);

	tca_leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * NUM_LEDS, GFP_KERNEL);
	if (!tca_leds)
		return ERR_PTR(-ENOMEM);
	pdata = devm_kzalloc(&client->dev,
			sizeof(struct lr_tca6507_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct led_info led;
		u32 reg;
		int ret;

		led.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		led.flags = 0;
		ret = of_property_read_u32(child, "reg", &reg);
		if (ret != 0 || reg < 0 || reg >= NUM_LEDS)
			continue;
        if (child->name[0]=='r')
            pdata->rgb[0][count_red++]   = reg;
        else
        if (child->name[0]=='g')
            pdata->rgb[1][count_green++] = reg;
        else
        if (child->name[0]=='b')
            pdata->rgb[2][count_blue++]  = reg;
            
		tca_leds[reg] = led;
	}

	pdata->leds.leds = tca_leds;
	pdata->leds.num_leds = NUM_LEDS;
	return pdata;
}

static const struct of_device_id of_tca6507_leds_match[] = {
	{ .compatible = "bksv-lightring", },
	{},
};
MODULE_DEVICE_TABLE(of, of_tca6507_leds_match);
/*
*/
static int blink_thread(void *data)
{
	struct tca6507_chip *tca = (struct tca6507_chip*)data;
    int led_state=0;
    int leftover;
    int timeout=MAX_SCHEDULE_TIMEOUT;
//    printk("blink thread started\n");
    set_current_state(TASK_INTERRUPTIBLE); 
    for(;;)
    { 
        set_current_state(TASK_INTERRUPTIBLE); 
        leftover=schedule_timeout(timeout);
        if(kthread_should_stop())
            break;
        
        if (leftover != 0)
        {
            led_state=0;
//            printk("set state 0\n");
        }
        if (led_state==0)
        {
 //           printk("set led on\n");
            tca6507_brightness_set(tca,0,tca->blink_setup.color1.red);
            tca6507_brightness_set(tca,1,tca->blink_setup.color1.green);
            tca6507_brightness_set(tca,2,tca->blink_setup.color1.blue);
            led_state=1;
            if (tca->blink_setup.color1.time == -1)
                timeout=MAX_SCHEDULE_TIMEOUT;
            else
            {
                timeout=(tca->blink_setup.color1.time * HZ)/1000;
//            printk("timeout = %d time1=%d HZ=%d\n",timeout,tca->blink_setup.color1.time,HZ);
            }
        }
        else
        {
//            printk("set led off\n");
            led_state=0;
            tca6507_brightness_set(tca,0,tca->blink_setup.color2.red);
            tca6507_brightness_set(tca,1,tca->blink_setup.color2.green);
            tca6507_brightness_set(tca,2,tca->blink_setup.color2.blue);
            if (tca->blink_setup.sync_metod != 0)
            {
                struct timespec ts;
                int ms;
                unsigned int sec;
                ktime_get_real_ts(&ts);
                sec = ts.tv_sec % tca->blink_setup.sync_metod;
                ms  = ts.tv_nsec / 1000000;
                ms  = (sec * 1000) + ms;
                ms  = (tca->blink_setup.sync_metod * 1000) - ms;
                timeout=(ms * HZ)/1000;
            }
            else
            {
                if (tca->blink_setup.color2.time == -1)
                    timeout=MAX_SCHEDULE_TIMEOUT;
                else
                    timeout=(tca->blink_setup.color2.time * HZ)/1000;
            }
//            printk("timeout = %d time2=%d HZ=%d\n",timeout,tca->blink_setup.color2.time,HZ);
        }
        if (tca->reg_set)
            schedule_work(&tca->work);
    }
//    printk("blink thread end\n");
    
    return 0;
} 

ssize_t lightring_fop_read(struct file *file,
    char __user *buffer,
    size_t length,
    loff_t *ppos)
{
   // struct tca6507_chip *tca = file->private_data;
    printk("%s\n",__func__);
    return 0;
}

ssize_t lightring_fop_write(struct file *file, const char __user *buffer,
    size_t length, loff_t *ppos)
{
    struct tca6507_chip *tca = file->private_data;
    int h=0;
    char buf[20];
    printk("%s\n",__func__);
    if ((length <= 20) && (arm_copy_from_user(buf,buffer,length) == 0))
    {
        buf[length]='\0';
        if (strncmp(buf,"c1r",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color1.red = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c1g",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color1.green = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c1b",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color1.blue = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c2r",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color2.red = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c2g",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color2.green = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c2b",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color2.blue = (unsigned char)h;
        }
        else
        if (strncmp(buf,"c1t",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color1.time = h;
        }
        else
        if (strncmp(buf,"c2t",3)==0)
        {
            sscanf(&buf[4],"%d",&h);
            tca->blink_setup.color2.time = h;
        }
        else
            printk("%s code not found\n",__func__);

printk("tca->blink_setup.color1.red=%d \"%s\" %d \n",tca->blink_setup.color1.red,buf,h);
        wake_up_process(tca->blink_task);

    }
    return length;
}
long lightring_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct tca6507_chip *tca = file->private_data;
    struct blink_setup_t *blink_setup = (struct blink_setup_t*)arg;
//    printk("%s\n",__func__);
    switch (cmd)
    {
        case LR_SET_COLOR:
            if (arm_copy_from_user(&tca->blink_setup, blink_setup, sizeof(struct blink_setup_t))==0)
            {
//                printk("%s set color %d %d %d\n",__func__,blink_setup->color1.red,blink_setup->color1.green,blink_setup->color1.blue);
                wake_up_process(tca->blink_task);
            }
        break;
        default:
            printk("%s unknown code\n",__func__);
        break;
    }
    return 0;
}

int lightring_fop_open(struct inode *inode, struct file *file)
{
    struct tca6507_chip *tca;
    int status = 0;
//    printk("%s\n",__func__);
    
    tca = container_of(inode->i_cdev, struct tca6507_chip, cdev);
    file->private_data = tca;
    return status;
}

int lightring_fop_close(struct inode *inode, struct file *file)
{
//    struct tca6507_chip *tca = file->private_data;
//    printk("%s\n",__func__);
    return 0;
}

static const struct file_operations lightring_fops = {
	.owner		= THIS_MODULE,
	.read		= lightring_fop_read,
	.write		= lightring_fop_write,
	.poll		= NULL,
	.unlocked_ioctl	= lightring_fop_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= lightring_fop_ioctl,
#endif
	.open		= lightring_fop_open,
	.release	= lightring_fop_close,
    .llseek		= NULL,
};

static int tca6507_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tca6507_chip *tca;
	struct i2c_adapter *adapter;
	struct lr_tca6507_platform_data *pdata;
	int err=0;
	int i = 0,j;
//    struct tca6507_led *lightring; //light ring blink

    int retval = 0;    
        
    retval = alloc_chrdev_region(&ring_devt, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number\n", DRIVER_NAME);
        ring_devt = 0;
        goto exit;
    }
    if (bk_sys_class==NULL)
    {
        bk_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
        if (IS_ERR_OR_NULL(bk_sys_class)) {
            printk(KERN_ERR "%s: couldn't create sys class\n", DRIVER_NAME);
            retval = PTR_ERR(bk_sys_class);
            bk_sys_class = NULL;
            goto exit;
        }
    }

	adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

    pdata = tca6507_led_dt_init(client);
    if (IS_ERR(pdata)) {
        dev_err(&client->dev, "Need %d entries in platform-data list\n",
            NUM_LEDS);
        return PTR_ERR(pdata);
    }
	tca = devm_kzalloc(&client->dev, sizeof(*tca), GFP_KERNEL);
	if (!tca)
		return -ENOMEM;
    for (i=0;i<3;i++)
    {
        for (j=0;j<2;j++)
        {
            tca->rgb[i][j] = pdata->rgb[i][j];
        }
    }
	tca->client = client;
	INIT_WORK(&tca->work, tca6507_work);
	spin_lock_init(&tca->lock);
	i2c_set_clientdata(client, tca);
    init_regs(tca); //get all regs from chip, initialized by uboot
	for (i = 0; i < NUM_LEDS; i++) {
		struct tca6507_led *l = tca->leds + i;

		l->chip = tca;
		l->num = i;
		if (pdata->leds.leds[i].name && !pdata->leds.leds[i].flags)
        {
			l->led_cdev.name = pdata->leds.leds[i].name;
		}
	}

    tca->device = device_create(bk_sys_class, &client->dev, MKDEV(MAJOR(ring_devt), 0),
                               (void *)tca, DEVICE_NAME_FORMAT, MINOR(MKDEV(MAJOR(ring_devt), 0)));
    //create char-device
    cdev_init(&tca->cdev, &lightring_fops);
    tca->cdev.owner = THIS_MODULE;
    if (cdev_add(&tca->cdev, MKDEV(MAJOR(ring_devt), 0), 1)) 
    {
        dev_err(&client->dev, "unable to add lightring c-device\n");
        goto exit;
    }
	/* set all registers to known state */
    tca6507_brightness_set(tca,0,0);
    tca6507_brightness_set(tca,1,0);
    tca6507_brightness_set(tca,2,0);
	tca->reg_set = 0x7f;
	schedule_work(&tca->work);

    tca->blink_task = kthread_create(blink_thread, (void*)tca, "blink_task");
    if(IS_ERR(tca->blink_task))
    { 
        printk("Unable to start kernel thread, blink_task.\n");
        err = PTR_ERR(tca->blink_task);
        tca->blink_task = NULL;
        goto exit;
    }
    wake_up_process(tca->blink_task); 

	return 0;
exit:
	return err;
}

static int tca6507_remove(struct i2c_client *client)
{
	struct tca6507_chip *tca = i2c_get_clientdata(client);
    if (bk_sys_class     != NULL)   {class_destroy(bk_sys_class);}

    if(tca->blink_task)
    {
        kthread_stop(tca->blink_task);
        tca->blink_task = NULL;
    }

	cancel_work_sync(&tca->work);

	return 0;
}

static struct i2c_driver tca6507_driver = {
	.driver   = {
		.name    = "bksv-lightring",
		.of_match_table = of_match_ptr(of_tca6507_leds_match),
	},
	.probe    = tca6507_probe,
	.remove   = tca6507_remove,
	.id_table = tca6507_id
};

module_i2c_driver(tca6507_driver);

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("TCA6507 LED/GPO driver");
MODULE_LICENSE("GPL v2");
