/*
 *  bksv_gpio.c - Linux kernel module for
 * 	Brüel & Kjær SXU gpio functions
 *
 *  Copyright (c) 2017 Lars Thestrup <LarsErling.Thestrup@bksv.com>
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

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

#define DRIVER_NAME  "bksv_gpio"
 
struct bk_gpio_device {
	struct device		dev;
	struct regulator *regulator;
    struct notifier_block nb;
    bool regulator_is_on;
	struct gpio_desc  *usb_en_n_gpio;
	struct gpio_desc  *ptb_en_gpio;
    struct gpio_descs *calibration_signal_gpio;
    struct gpio_desc  *cic_enable_gpio;
    struct gpio_desc  *input_cal_enable_gpio;
    struct gpio_desc  *ext_freq_range_enable_gpio;
    struct gpio_desc  *input_dc_enable_gpio;
    struct gpio_desc  *gps_reset_gpio;
	struct gpio_desc  *psu_burst_gpio;
	struct gpio_desc  *ext_access_gpio;
	struct gpio_desc  *tp500_gpio;
    unsigned int calibration_signal_val;
    unsigned int cic_enable_val;
    unsigned int input_cal_enable_val;
    unsigned int ext_freq_range_enable_val;
    unsigned int input_dc_enable_val;
    struct mxc_gpio_hwdata *gpio4;
};

static inline struct bk_gpio_device *to_bk_gpio_device(struct device *dev)
{
	return dev ? container_of(dev, struct bk_gpio_device, dev) : NULL;
}
/*
static ssize_t set_usb_en_n(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    gpiod_set_value(bk_gpio->usb_en_n_gpio, bit & 1);
    return count;
}
*/ 
static ssize_t get_usb_en_n(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->usb_en_n_gpio);
	return sprintf(buf, "%d\n", val);
}

/* Enables USB through the USB connector,              disable =false*/
/* Enables analog audio out through the USB connector, disable =true */ 
int bksv_gpio_usb_en_n(struct device *dev, bool disable)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);

    gpiod_set_value(bk_gpio->usb_en_n_gpio, disable ? 1 : 0);
    return 0;
}


static ssize_t set_ptb_en(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    gpiod_set_value(bk_gpio->ptb_en_gpio, bit & 1);
    return count;
}

static ssize_t get_ptb_en(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->ptb_en_gpio);
	return sprintf(buf, "%d\n", val);
}
static ssize_t set_tp500(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    gpiod_set_value(bk_gpio->tp500_gpio, bit & 1);
    return count;
}

static ssize_t get_tp500(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->tp500_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t get_calibration_signal(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int cal[2];
    unsigned int calval;
    
    cal[0] = gpiod_get_value(bk_gpio->calibration_signal_gpio->desc[0]);
    cal[1] = gpiod_get_value(bk_gpio->calibration_signal_gpio->desc[1]);
    calval = ((cal[1] & 1) << 1) | (cal[0] & 1);
    switch (calval)
    {
        case 0: strcpy(buf,"GenSignal");break;
        case 1: strcpy(buf,"V200_REF");break;
        case 2: strcpy(buf,"3V_REF");break;
        case 3: strcpy(buf,"AGND");break;
    }
	return strlen(buf);
}

void  to_uppercase(char *t,const char *s)
{
    int i = 0;
    while ((s[i]) && (i < 19) && (s[i] > '\r'))
    {
        if (s[i] >= 97 && s[i] <= 122)
            t[i] = s[i]-32;
        else
            t[i] = s[i];
        i++;
    }
    t[i] = '\0';
}

static ssize_t set_calibration_signal(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit = 0;
    int bitarray[2];
    char t[20];
    if ((buf[0] >= '0') && (buf[0] <= '3') && (buf[1] <=' '))
    {
        ret = kstrtoul(buf, 10, &bit);
        if (ret)
            return ret;
    }
    else
    {
        to_uppercase(t,buf);
        if (strcmp(t,"GENSIGNAL") == 0)
            bit=0;
        else
        if (strcmp(t,"V200_REF") == 0)
            bit=1;
        else
        if (strcmp(t,"3V_REF") == 0)
            bit=2;
        else
        if (strcmp(t,"AGND") == 0)
            bit=3;
    }
    bitarray[0] = bit & 1;
    bitarray[1] = (bit >> 1) & 1;
    if (bk_gpio->regulator_is_on)
    {
        gpiod_set_array_value(2,
                        bk_gpio->calibration_signal_gpio->desc,
                        bitarray);
    }
    bk_gpio->calibration_signal_val = bit;

    return count;
}

static ssize_t get_cic_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->cic_enable_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_cic_enable(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    if (bk_gpio->regulator_is_on)
    {
        gpiod_set_value(bk_gpio->cic_enable_gpio, bit & 1);
    }
    bk_gpio->cic_enable_val = bit & 1;
    return count;
}
 
static ssize_t get_input_cal_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->input_cal_enable_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_input_cal_enable(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    if (bk_gpio->regulator_is_on)
    {
        gpiod_set_value(bk_gpio->input_cal_enable_gpio, bit & 1);
    }
    bk_gpio->input_cal_enable_val = bit & 1;
    return count;
}
 
static ssize_t get_ext_freq_range_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->ext_freq_range_enable_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_ext_freq_range_enable(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    if (bk_gpio->regulator_is_on)
    {
        gpiod_set_value(bk_gpio->ext_freq_range_enable_gpio, bit & 1);
    }
    bk_gpio->ext_freq_range_enable_val = bit & 1;
    return count;
}
 
static ssize_t get_input_dc_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->input_dc_enable_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_input_dc_enable(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    if (bk_gpio->regulator_is_on)
    {
        gpiod_set_value(bk_gpio->input_dc_enable_gpio, bit & 1);
    }
    bk_gpio->input_dc_enable_val = bit & 1;
    return count;
}
 

static ssize_t get_gps_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->gps_reset_gpio);
    if (val & 1)
    {
        val = 0;
    }
    else
    {
        val = 1;
    }
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_gps_reset(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    if (bit & 1)
        bit = 0;
    else 
        bit = 1;
    gpiod_set_value(bk_gpio->gps_reset_gpio, bit);
    return count;
}


static ssize_t get_psu_burst(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->psu_burst_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_psu_burst(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;
    gpiod_set_value(bk_gpio->psu_burst_gpio, bit);
    return count;
}

static ssize_t get_ext_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = gpiod_get_value(bk_gpio->ext_access_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t get_enable_wir(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    unsigned int val;

    val = (bk_gpio->gpio4->dr_reg & (1 << 10)) ? 1 : 0;

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_enable_wir(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
	struct platform_device *pdev   = to_platform_device(dev);
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    int ret;
    unsigned long bit;
    
    ret = kstrtoul(buf, 10, &bit);
    if (ret)
        return ret;

    if(bit)
        bk_gpio->gpio4->dr_reg |=  (1 << 10);
    else
        bk_gpio->gpio4->dr_reg &= ~(1 << 10);

    return count;
}

static DEVICE_ATTR(usb_en_n,                     S_IRUSR, get_usb_en_n,             NULL);//set by /drivers/bksv/bksv_tcpci.c
static DEVICE_ATTR(ptb_en,               S_IWUSR|S_IRUSR, get_ptb_en,               set_ptb_en);
static DEVICE_ATTR(calibration_signal,   S_IWUSR|S_IRUSR, get_calibration_signal,   set_calibration_signal);
static DEVICE_ATTR(cic_enable,           S_IWUSR|S_IRUSR, get_cic_enable,           set_cic_enable);
static DEVICE_ATTR(input_cal_enable,     S_IWUSR|S_IRUSR, get_input_cal_enable,     set_input_cal_enable);
static DEVICE_ATTR(ext_freq_range_enable,S_IWUSR|S_IRUSR, get_ext_freq_range_enable,set_ext_freq_range_enable);
static DEVICE_ATTR(input_dc_enable,      S_IWUSR|S_IRUSR, get_input_dc_enable,      set_input_dc_enable);
static DEVICE_ATTR(gps_reset,            S_IWUSR|S_IRUSR, get_gps_reset,            set_gps_reset);
static DEVICE_ATTR(psu_burst,            S_IWUSR|S_IRUSR, get_psu_burst,            set_psu_burst);
static DEVICE_ATTR(ext_access,                   S_IRUSR, get_ext_access,           NULL);
static DEVICE_ATTR(enable_wir,           S_IWUSR|S_IRUSR, get_enable_wir,           set_enable_wir);
static DEVICE_ATTR(tp500,                S_IWUSR|S_IRUSR, get_tp500,                set_tp500);

static struct attribute *bksv_gpio_attributes[] = {
     &dev_attr_usb_en_n.attr,
     &dev_attr_ptb_en.attr,
     &dev_attr_calibration_signal.attr,
     &dev_attr_cic_enable.attr,
     &dev_attr_input_cal_enable.attr,
     &dev_attr_ext_freq_range_enable.attr,
     &dev_attr_input_dc_enable.attr,
     &dev_attr_gps_reset.attr,
     &dev_attr_psu_burst.attr,
     &dev_attr_ext_access.attr,
     &dev_attr_enable_wir.attr,
     &dev_attr_tp500.attr,
     NULL
 };
 
 static const struct attribute_group bksv_gpio_attr_group = {
     .attrs = bksv_gpio_attributes,
 };
 
/*
disable
ana_notify action=1024    REGULATOR_EVENT_PRE_DISABLE
ana_notify action=8192    REGULATOR_EVENT_PRE_DO_DISABLE
ana_notify action=128     REGULATOR_EVENT_DISABLE

enable
ana_notify action=4096    REGULATOR_EVENT_PRE_DO_ENABLE
ana_notify action=16384   REGULATOR_EVENT_AFT_DO_ENABLE
*/
static int ana_notify(struct notifier_block *self, unsigned long action, void *data)
{
	struct bk_gpio_device *bk_gpio;
    unsigned int cal[2];
    unsigned int calval;
    int bitarray[2];
	bk_gpio = container_of(self, struct bk_gpio_device, nb); 

    switch(action)
    {
        case REGULATOR_EVENT_AFT_DO_ENABLE:
            bitarray[0] =  bk_gpio->calibration_signal_val & 1;
            bitarray[1] = (bk_gpio->calibration_signal_val >> 1) & 1;
            gpiod_set_array_value(2,
                            bk_gpio->calibration_signal_gpio->desc,
                            bitarray);
            gpiod_set_value(bk_gpio->cic_enable_gpio,            bk_gpio->cic_enable_val);
            gpiod_set_value(bk_gpio->input_cal_enable_gpio,      bk_gpio->input_cal_enable_val);
            gpiod_set_value(bk_gpio->ext_freq_range_enable_gpio, bk_gpio->ext_freq_range_enable_val);
            gpiod_set_value(bk_gpio->input_dc_enable_gpio,       bk_gpio->input_dc_enable_val);
            bk_gpio->regulator_is_on = true;
        break;
        case REGULATOR_EVENT_PRE_DISABLE:
            if (bk_gpio->regulator_is_on)
            {
                bk_gpio->regulator_is_on = false;

                cal[0] = gpiod_get_value(bk_gpio->calibration_signal_gpio->desc[0]);
                cal[1] = gpiod_get_value(bk_gpio->calibration_signal_gpio->desc[1]);
                calval = ((cal[1] & 1) << 1) | (cal[0] & 1);
                bk_gpio->calibration_signal_val = calval;

                bk_gpio->cic_enable_val = gpiod_get_value(bk_gpio->cic_enable_gpio);
                gpiod_set_value(bk_gpio->cic_enable_gpio, 0);

                bk_gpio->input_cal_enable_val = gpiod_get_value(bk_gpio->input_cal_enable_gpio);
                gpiod_set_value(bk_gpio->input_cal_enable_gpio, 0);

                bk_gpio->ext_freq_range_enable_val = gpiod_get_value(bk_gpio->ext_freq_range_enable_gpio);
                gpiod_set_value(bk_gpio->ext_freq_range_enable_gpio, 0);

                bk_gpio->input_dc_enable_val = gpiod_get_value(bk_gpio->input_dc_enable_gpio);
                gpiod_set_value(bk_gpio->input_dc_enable_gpio, 0);
            }
        break;
    }
	return NOTIFY_OK;
}
 
static int bksv_gpio_probe(struct platform_device *pdev)
{
    int ret;
    struct bk_gpio_device *bk_gpio;
	struct regulator *reg;

    bk_gpio = devm_kzalloc(&pdev->dev, sizeof(struct bk_gpio_device), GFP_KERNEL);
    if (!bk_gpio)
        return -ENOMEM;
    bk_gpio->dev.driver_data=bk_gpio;
	platform_set_drvdata(pdev, bk_gpio);
    dev_dbg(&pdev->dev, "bksv_gpio driver probing\n");
 
	reg = devm_regulator_get(&pdev->dev, "ana_on");
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "reg get err: %d\n", ret);
		return ret;
	}
	bk_gpio->regulator = reg;

    if (regulator_is_enabled(bk_gpio->regulator) > 0)
        bk_gpio->regulator_is_on = true;
    else
        bk_gpio->regulator_is_on = false;

    memset(&bk_gpio->nb, 0, sizeof(struct notifier_block));
    bk_gpio->nb.notifier_call = ana_notify;
    devm_regulator_register_notifier(reg,&bk_gpio->nb);
 
    bk_gpio->ext_access_gpio = devm_gpiod_get(&pdev->dev,"ext_access",GPIOD_IN);
    if (IS_ERR(bk_gpio->ext_access_gpio))
    {
 		dev_err(&pdev->dev, "ext_access not found in devicetree\n");
		return PTR_ERR(bk_gpio->ext_access_gpio);
    }

    /* EN_PSU_BURST_N gpio pin, when power to analog board or wifi running, set  burst mode off => gpio low*/

    bk_gpio->psu_burst_gpio = devm_gpiod_get(&pdev->dev,"psu_burst",GPIOD_OUT_LOW);
    if (IS_ERR(bk_gpio->psu_burst_gpio))
    {
 		dev_err(&pdev->dev, "psu_burst not found in devicetree\n");
		return PTR_ERR(bk_gpio->psu_burst_gpio);
    }
    /* usb_en_n gpio pin */
    bk_gpio->usb_en_n_gpio = devm_gpiod_get(&pdev->dev,"usb_en_n",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->usb_en_n_gpio))
    {
 		dev_err(&pdev->dev, "usb_en_n not found in devicetree\n");
		return PTR_ERR(bk_gpio->usb_en_n_gpio);
    }

    /* ptb_en gpio pin */
    bk_gpio->ptb_en_gpio = devm_gpiod_get(&pdev->dev,"ptb_en",GPIOD_OUT_LOW);
    if (IS_ERR(bk_gpio->ptb_en_gpio))
    {
 		dev_err(&pdev->dev, "ptb_en not found in devicetree\n");
		return PTR_ERR(bk_gpio->ptb_en_gpio);
    }

    /* tp500_en gpio pin */
    bk_gpio->tp500_gpio = devm_gpiod_get(&pdev->dev,"tp500",GPIOD_OUT_LOW);
    if (IS_ERR(bk_gpio->tp500_gpio))
    {
 		dev_err(&pdev->dev, "tp500 not found in devicetree\n");
		return PTR_ERR(bk_gpio->tp500_gpio);
    }

    /* calibration_signal gpio pin */
    bk_gpio->calibration_signal_gpio = devm_gpiod_get_array(&pdev->dev,"calibration_signal",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->calibration_signal_gpio))
    {
 		dev_err(&pdev->dev, "calibration_signal not found in devicetree\n");
		return PTR_ERR(bk_gpio->calibration_signal_gpio);
    }
    bk_gpio->calibration_signal_val = 0;

    /* cic_enable gpio pin */
    bk_gpio->cic_enable_gpio = devm_gpiod_get(&pdev->dev,"cic_enable",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->cic_enable_gpio))
    {
 		dev_err(&pdev->dev, "cic_enable not found in devicetree\n");
		return PTR_ERR(bk_gpio->cic_enable_gpio);
    }
    bk_gpio->cic_enable_val = 0;

    /* input_cal_enable gpio pin */
    bk_gpio->input_cal_enable_gpio = devm_gpiod_get(&pdev->dev,"input_cal_enable",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->input_cal_enable_gpio))
    {
 		dev_err(&pdev->dev, "input_cal_enable not found in devicetree\n");
		return PTR_ERR(bk_gpio->input_cal_enable_gpio);
    }
    bk_gpio->input_cal_enable_val = 0;

    /* ext_freq_range_enable gpio pin */
    bk_gpio->ext_freq_range_enable_gpio = devm_gpiod_get(&pdev->dev,"ext_freq_range_enable",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->ext_freq_range_enable_gpio))
    {
 		dev_err(&pdev->dev, "ext_freq_range_enable not found in devicetree\n");
		return PTR_ERR(bk_gpio->ext_freq_range_enable_gpio);
    }
    bk_gpio->ext_freq_range_enable_val = 0;

    /* input_dc_enable gpio pin */
    bk_gpio->input_dc_enable_gpio = devm_gpiod_get(&pdev->dev,"input_dc_enable",GPIOD_OUT_LOW );
    if (IS_ERR(bk_gpio->input_dc_enable_gpio))
    {
 		dev_err(&pdev->dev, "input_dc_enable not found in devicetree\n");
		return PTR_ERR(bk_gpio->input_dc_enable_gpio);
    }
    bk_gpio->input_dc_enable_val = 0;

    /* gnss_reset gpio pin */
    bk_gpio->gps_reset_gpio = devm_gpiod_get(&pdev->dev,"gps_reset",GPIOD_ASIS);
    if (IS_ERR(bk_gpio->gps_reset_gpio))
    {
 		dev_err(&pdev->dev, "gps_reset not found in devicetree\n");
		return PTR_ERR(bk_gpio->gps_reset_gpio);
    }

    bk_gpio->gpio4 = (struct mxc_gpio_hwdata *)ioremap_nocache(0x30230000, 4096);
    if (!bk_gpio->gpio4)
    {
 		dev_err(&pdev->dev, "could not map gpio4\n");
		return PTR_ERR(bk_gpio->gpio4);
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &bksv_gpio_attr_group);
    dev_dbg(&pdev->dev,"bksv_gpio driver probed\n");
    msleep(100); //wait for power on

    return ret;
}

static int bksv_gpio_remove(struct platform_device *pdev )
{
	struct bk_gpio_device *bk_gpio = platform_get_drvdata(pdev);
    sysfs_remove_group(&bk_gpio->dev.kobj, &bksv_gpio_attr_group);
    iounmap(bk_gpio->gpio4);
    return 0;
}

static const struct of_device_id bksv_gpio_of_match[] = {
     { 
         .compatible = "bksv_gpio",
        .data = &bksv_gpio_attr_group,
     },
     { }
};
MODULE_DEVICE_TABLE(of, bksv_gpio_of_match);

static struct platform_driver bksv_gpio_driver = {
	.probe    = bksv_gpio_probe,
	.remove   = bksv_gpio_remove,
	.driver   = {
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(bksv_gpio_of_match),
		   .pm = NULL,
	},
};

static int __init bksv_gpio_init(void)
{
	return platform_driver_register(&bksv_gpio_driver);
}

subsys_initcall(bksv_gpio_init);

static void __exit bksv_gpio_exit(void)
{
	platform_driver_unregister(&bksv_gpio_driver);
}
module_exit(bksv_gpio_exit);

MODULE_AUTHOR("Lars Thestrup <LarsErling.Thestrup@bksv.com>");
MODULE_DESCRIPTION("GPIO for SXU");
MODULE_LICENSE("GPL v2");
