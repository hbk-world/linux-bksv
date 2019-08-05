/*
 * An I2C and SPI driver for the NXP PCF2127/29 RTC
 * Copyright 2013 Til-Technologies
 *
 * Author: Renaud Cerrato <r.cerrato@til-technologies.fr>
 *
 * based on the other drivers in this same directory.
 *
 * Datasheet: http://cache.nxp.com/documents/data_sheet/PCF2127.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/regmap.h>
#define RTC_START_1HZ     _IO(0xB5,20)
#define RTC_STOP_1HZ      _IO(0xB5,21)
#define RTC_BK_SET_TIME   _IO(0xB5,22) //set RTC time  to systime
#define RTC_BK_GET_AGING  _IOWR(0xB5, 23, int) //set RTC aging
#define RTC_BK_SET_AGING  _IOR(0xB5, 24, int)  //get RTC aging
#define RTC_BK_DUMP       _IO(0xB5, 25)  //Dump RTC registers


#define PCF2127_REG_CTRL1       (0x00)  /* Control Register 1  */
#define PCF2127_REG_CTRL1_STOP      BIT(5) /* Stop prescaler   */
#define PCF2127_REG_CTRL1_SI        BIT(0) /* Second interrupt */
#define PCF2127_EXT_TEST            BIT(7) /* external clock test mode */

#define PCF2127_REG_CTRL2       (0x01)  /* Control Register 2 */
#define PCF2127_REG_CTRL2_AIE       BIT(1) /* Alarm interrupt enable */
#define PCF2127_REG_CTRL2_AF        BIT(4) /* Alarm flag */

#define PCF2127_REG_CTRL3       (0x02)  /* Control Register 3 */
#define PCF2127_REG_CTRL3_BLF		BIT(2)
#define PCF2127_REG_CTRL3_PWRMNG    0XE0

#define PCF2127_REG_SC          (0x03)  /* datetime */
#define PCF2127_OSF                 BIT(7)  /* Oscillator Fail flag */
#define PCF2127_REG_MN          (0x04)
#define PCF2127_REG_HR          (0x05)
#define PCF2127_REG_DM          (0x06)
#define PCF2127_REG_DW          (0x07)
#define PCF2127_REG_MO          (0x08)
#define PCF2127_REG_YR          (0x09)

/* Alarm registers */
#define PCF2127_REG_SCA         (0x0a)
#define PCF2127_REG_MNA         (0x0b)
#define PCF2127_REG_HRA         (0x0c)
#define PCF2127_REG_DMA         (0x0d)
#define PCF2127_REG_DWA         (0x0e)

#define PCF2127_REG_CLKOUT      (0x0F) // CLKOUT_ctl
#define PCF2127_REG_CLKOUT_OTPR     BIT(5) // OTP refresh
#define PCF2127_REG_WDOG_CTL    (0x10)
#define PCF2127_REG_WDOG_CTL_TITP   BIT(5)
#define PCF2127_REG_WDOG_VAL    (0x11)
#define PCF2127_REG_AGING       (0x19) // Aging offset register

/* Mask to OR onto alarm secs, minutes, hours, days registers */
#define ALARM_ENABLE            (0x00)
#define ALARM_DISABLE           (0x80)

struct pcf2127_t {
	struct rtc_device *rtc;
	struct regmap *regmap;
    struct hrtimer hr_timer;
    struct work_struct work;
};
struct rtc_time tm;

void dump_regs(unsigned char *buf, int bufsize)
{
    int i,j,k;
    char s[400];
    k=0;
    for (i=0;i<bufsize;i++)
    {
        j = sprintf(&s[k],"0x%02x = 0x%02x\n",i,buf[i]);
        k += j;
    }
    printk("rtc regs:\n%s",s);
}
/*
 * In the routines that deal directly with the pcf2127 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int pcf2127_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	unsigned char buf[16];
	int ret;
    int err;
    unsigned char sec_old;
    int try=1;

rtc_try_again: 
	ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, buf,sizeof(buf));
	if (ret) {
		dev_err(dev, "%s: read error\n", __func__);
		return ret;
	}

	if (buf[PCF2127_REG_CTRL1] & PCF2127_EXT_TEST)
    {
        if (try)
        {
		    dev_warn(dev,
			 "RTC: external clock test mode detected, trying again\n");
            try--;
            goto rtc_try_again;
        }
		dev_warn(dev,
			 "RTC: external clock test mode detected, date/time is not reliable\n resetting bit\n");
        dump_regs(buf,sizeof(buf));
        buf[PCF2127_REG_CTRL1] &= ~PCF2127_EXT_TEST;
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL1, &buf[PCF2127_REG_CTRL1], 1);
        if (err) {
            dev_err(dev,"%s: err=%d", __func__, err);
            return err;
        }
		return -EINVAL;
	}

	if ((buf[PCF2127_REG_CTRL3] & PCF2127_REG_CTRL3_PWRMNG) != 0 )
    {
        if (try)
        {
		    dev_warn(dev,
			 "RTC: Battery switch-over fault detected, trying again\n");
            try--;
            goto rtc_try_again;
        }
		dev_warn(dev,
			 "RTC: Battery switch-over fault detected, date/time is not reliable\n resetting bits\n");
        dump_regs(buf,sizeof(buf));
        buf[PCF2127_REG_CTRL3] = 0;
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL3, &buf[PCF2127_REG_CTRL3], 1);
        if (err) {
            dev_err(dev,"%s: err=%d", __func__, err);
            return err;
        }
		return -EINVAL;
	}

	if (buf[PCF2127_REG_SC] & PCF2127_OSF)
    {
		/*
		 * no need clear the flag here,
		 * it will be cleared once the new date is saved
		 */
        if (try)
        {
		    dev_warn(dev,
			 "RTC: oscillator stop detected, trying again\n");
            try--;
            goto rtc_try_again;
        }
        dump_regs(buf,sizeof(buf));
        buf[PCF2127_REG_CLKOUT] = 0; //clear OTPR bit + default to other bits
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CLKOUT, &buf[PCF2127_REG_CLKOUT], 1);
        if (err) 
        {
            dev_err(dev,"%s: err=%d", __func__, err);
            return err;
        }
        buf[PCF2127_REG_CLKOUT] = PCF2127_REG_CLKOUT_OTPR;
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CLKOUT, &buf[PCF2127_REG_CLKOUT], 1);
        if (err) 
        {
            dev_err(dev,"%s: err=%d", __func__, err);
            return err;
        }
		dev_warn(dev,"RTC: oscillator stop detected, date/time is not reliable\n");
		return -EINVAL;
	}

	dev_dbg(dev,
		"%s: raw data is cr1=%02x, cr2=%02x, cr3=%02x, "
		"sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2],
		buf[3], buf[4], buf[5],
		buf[6], buf[7], buf[8], buf[9]);

    sec_old=buf[PCF2127_REG_SC] & 0x7F;
    do
    {//wait for second change
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, buf,sizeof(buf));
        if (ret)
        {
            dev_err(dev, "%s: read error\n", __func__);
            return ret;
        }

    } while ((buf[PCF2127_REG_SC] & 0x7F) == sec_old);
    
	tm->tm_sec = bcd2bin(buf[PCF2127_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF2127_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF2127_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF2127_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF2127_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF2127_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF2127_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	dev_info(dev, "%s: read RTC time %04d-%02d-%02d %02d:%02d:%02d\n",
		__func__,
		tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return rtc_valid_tm(tm);
}

static int pcf2127_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	unsigned char buf[16],reg0;
	unsigned char bufr[7];
	unsigned char bufr2[7];
	int i = 0, err;
	int ret;
    time64_t time;
    struct timespec64 t64;

	ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_SC, bufr,sizeof(bufr));
	if (ret) {
		dev_err(dev, "%s: read error\n", __func__);
		return ret;
	}

    //NTP will set the time at ex second 2.5 to 3
    //using the STOP bit we know that next second shift will happen after 0.5 sec
    //ex from 3 to 4, then the RTC is 1 second ahead
    //we need to subtract one second
    //this will not be precise, if time is set at another fraction of a second
    if (tm->tm_sec == 0)
    {
        time = rtc_tm_to_time64(tm);
        time--;
        rtc_time64_to_tm(time, tm);
    }
    else
    {
        tm->tm_sec--;
    }

	/* hours, minutes and seconds */
	buf[i++] = bin2bcd(tm->tm_sec);	/* this will also clear OSF flag */
	buf[i++] = bin2bcd(tm->tm_min);
	buf[i++] = bin2bcd(tm->tm_hour);
	buf[i++] = bin2bcd(tm->tm_mday);
	buf[i++] = tm->tm_wday & 0x07;

	/* month, 1 - 12 */
	buf[i++] = bin2bcd(tm->tm_mon + 1);

	/* year */
	buf[i++] = bin2bcd(tm->tm_year % 100);

    //get control 1 register
    ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0,sizeof(reg0));
    if (ret) {
        dev_err(dev, "%s: read error\n", __func__);
        return ret;
    }
    //Set STOP bit
    reg0 |= PCF2127_REG_CTRL1_STOP;
    //write control 1 register
	err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0, 1);
	if (err) {
		dev_err(dev,
			"%s: err=%d", __func__, err);
		return err;
	}

	/* write time register's data */
	err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_SC, buf, i);
	if (err) {
		dev_err(dev,
			"%s: err=%d", __func__, err);
		return err;
	}

   //Remove STOP bit
    reg0 &= ~PCF2127_REG_CTRL1_STOP;
    //write control 1 register
	err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0, 1);
	if (err) {
		dev_err(dev,
			"%s: err=%d", __func__, err);
		return err;
	}
	ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_SC, bufr2,sizeof(bufr2));
	if (ret) {
		dev_err(dev, "%s: read error\n", __func__);
		return ret;
	}

    ktime_get_real_ts64(&t64);
	dev_info(dev, "%lld.%09ld setting RTC time %04d-%02d-%02d %02d:%02d:%02d, rtc before=%02x-%02x-%02x %02x:%02x:%02x OSF: before=%d now=%d\n",
        t64.tv_sec,t64.tv_nsec,
		tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec,
        bufr[9-3],       bufr[8-3] & 0x1F,bufr[6-3] & 0x3F,
        bufr[5-3] & 0x3F,bufr[4-3] & 0x7F,bufr[3-3] & 0x7F,
        bufr[3-3]>>7,bufr2[3-3]>>7);

	return 0;
}

static int pcf2127_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	struct rtc_time *tm = &alarm->time;
	unsigned char buf[15];
	int ret;

	ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, buf,
				sizeof(buf));
	if (ret) {
		dev_err(dev, "%s: read error\n", __func__);
		return ret;
	}

	if (buf[PCF2127_REG_SC] & PCF2127_OSF) {
		/*
		 * no need clear the flag here,
		 * it will be cleared once the new date is saved
		 */
		dev_warn(dev,
			 "oscillator stop detected, date/time is not reliable\n");
		return -EINVAL;
	}

	dev_dbg(dev,
		"%s: raw data is cr1=%02x, cr2=%02x, cr3=%02x, "
		"sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x, "
		"seca=%02x, mina=%02x, hra=%02x, "
		"mdaya=%02x, wdaya=%02x\n",
		__func__,
		buf[0], buf[1], buf[2],
		buf[3], buf[4], buf[5],
		buf[6], buf[7], buf[8],
		buf[9], buf[10], buf[11],
		buf[12], buf[13], buf[14]);

	tm->tm_sec = bcd2bin(buf[PCF2127_REG_SCA] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF2127_REG_MNA] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF2127_REG_HRA] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF2127_REG_DMA] & 0x3F);
	tm->tm_wday = buf[PCF2127_REG_DWA] & 0x07;

	/* The alarm doesn't store the month and year so get them from the rtc section */
	tm->tm_mon = bcd2bin(buf[PCF2127_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF2127_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	alarm->enabled = !!(buf[PCF2127_REG_CTRL2] & PCF2127_REG_CTRL2_AIE);

	if (alarm->enabled)
		dev_dbg(dev, "%s: rtc alarm is %04d-%02d-%02d %02d:%02d:%02d\n",
			__func__,
			tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
	else
		dev_dbg(dev, "%s: rtc alarm is disabled\n", __func__);

	return 0;
}

static int pcf2127_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	int err, ctrl2;

    /* clear alarm flag and configure alarm interrupt */
	err = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL2, &ctrl2, 1);
	if (err) {
		dev_err(dev, "%s: read error=%d\n", __func__, err);
		return err;
	}

	if (ctrl2 & PCF2127_REG_CTRL2_AF)
		dev_info(dev, "rtc alarm is active, clearing alarm\n");

	ctrl2 &= ~PCF2127_REG_CTRL2_AF;

	if(enable)
		ctrl2 |= PCF2127_REG_CTRL2_AIE;
	else
		ctrl2 &= ~PCF2127_REG_CTRL2_AIE;

	err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL2, &ctrl2, 1);
	if (err) {
		dev_err(dev, "%s: write error=%d", __func__, err);
		return err;
	}

	return 0;
}

static int pcf2127_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
    struct rtc_time *alarm_tm = &alarm->time, rtc_tm;
	unsigned long rtc_secs, alarm_secs;
	unsigned char buf[5];
	int err, enable, alarm_config;

	dev_dbg(dev, "%s: setting rtc alarm %04d-%02d-%02d %02d:%02d:%02d\n",
		__func__,
		alarm_tm->tm_year + 1900, alarm_tm->tm_mon + 1, alarm_tm->tm_mday,
		alarm_tm->tm_hour, alarm_tm->tm_min, alarm_tm->tm_sec);

	err = pcf2127_rtc_read_time(dev, &rtc_tm);
	if (err)
		return err;
	err = rtc_tm_to_time(&rtc_tm, &rtc_secs);
	if (err)
		return err;
	err = rtc_tm_to_time(alarm_tm, &alarm_secs);
	if (err)
		return err;

	/* If the alarm time is before the current time disable the alarm */
	enable = (!alarm->enabled || alarm_secs <= rtc_secs) ? 0 : 1;

	/* days, hours, minutes and seconds */
	alarm_config = enable ? ALARM_ENABLE : ALARM_DISABLE;
	buf[0] = bin2bcd(alarm_tm->tm_sec) | alarm_config;
	buf[1] = bin2bcd(alarm_tm->tm_min) | alarm_config;
	buf[2] = bin2bcd(alarm_tm->tm_hour) | alarm_config;
	buf[3] = bin2bcd(alarm_tm->tm_mday) | alarm_config;
	buf[4] = (alarm_tm->tm_wday & 0x07) | alarm_config;

	/* write alarm time */
	err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_SCA, buf, sizeof(buf));
	if (err) {
		dev_err(dev, "%s: write error=%d", __func__, err);
		return err;
	}

	return pcf2127_rtc_alarm_irq_enable(dev, enable);
}
/*
static irqreturn_t pcf2127_rtc_interrupt(int irq, void *dev)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	int status = IRQ_NONE, err, ctrl2;

	err = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL2, &ctrl2, 1);
	if (err < 0) {
		dev_err(dev, "%s: reading ctrl2 failed\n", __func__);
		return err;
	}

	if (ctrl2 & PCF2127_REG_CTRL2_AF) {
		dev_info(dev, "rtc alarm!\n");

		rtc_update_irq(pcf2127->rtc, 1, RTC_IRQF | RTC_AF);

		if (pcf2127_rtc_alarm_irq_enable(dev, 0) == 0)
			status = IRQ_HANDLED;
	}

	return status;
}
*/
#ifdef CONFIG_RTC_INTF_DEV
void rtc_set_work_handler(struct work_struct * work)
{
	struct pcf2127_t *pcf2127 = container_of(work, struct pcf2127_t,work);
    struct device *dev;
    struct timespec64 ts64;

    dev=pcf2127->rtc->dev.parent;

    ktime_get_real_ts64(&ts64);
    pcf2127_rtc_set_time(dev, &tm);
//    printk("%s %lld.%09ld\n",__func__,ts64.tv_sec,ts64.tv_nsec);
}

enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer )
{
    struct pcf2127_t *pcf2127=container_of(timer,struct pcf2127_t,hr_timer);
    schedule_work(&pcf2127->work); //rtc_set_work_handler
    return HRTIMER_NORESTART;
}

/*
*/
static int rtc_set_precise(struct pcf2127_t *pcf2127)
{
    struct timespec64 ts64;
    ktime_t kt;
    ktime_get_real_ts64(&ts64);
    if  (ts64.tv_nsec > 500000000)
    {
        kt = ktime_set(ts64.tv_sec + 1, 500000000);
        rtc_time64_to_tm(ts64.tv_sec+2, &tm);
    }
    else
    {
        kt = ktime_set(ts64.tv_sec    , 500000000);
        rtc_time64_to_tm(ts64.tv_sec+1, &tm);
    }

    pcf2127->hr_timer.function=my_hrtimer_callback;
    hrtimer_start(&pcf2127->hr_timer,kt,HRTIMER_MODE_ABS);
//    printk("%s %lld.%09ld\n",__func__,ts64.tv_sec,ts64.tv_nsec);
    return 0;
}

static int pcf2127_rtc_ioctl(struct device *dev,
				unsigned int cmd, unsigned long arg)
{
	struct pcf2127_t *pcf2127 = dev_get_drvdata(dev);
	unsigned char reg0,reg16;
    int user_arg;
    int err;
	int ret;
	int touser;
    unsigned char aging, old_aging;
	switch (cmd) {
	case RTC_VL_READ:
		ret = regmap_read(pcf2127->regmap, PCF2127_REG_CTRL3, &touser);
		if (ret)
			return ret;

		touser = touser & PCF2127_REG_CTRL3_BLF ? 1 : 0;

		if (copy_to_user((void __user *)arg, &touser, sizeof(int)))
			return -EFAULT;
		return 0;

    case RTC_START_1HZ:		/* Periodic int. enable on	*/
//        printk("%s RTC_START_1HZ\n",__func__);
        //get Watchdg_tim_ctl register
//#define PCF2127_REG_WDOG_CTL    (0x10)
//#define PCF2127_REG_WDOG_CTL_TITP   BIT(5)
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_WDOG_CTL, &reg16,sizeof(reg16));
        if (ret) {
            dev_err(dev, "%s: read error\n", __func__);
            return ret;
        }
        //Set SI bit
        reg16 |= PCF2127_REG_WDOG_CTL_TITP;
        reg16 &= ~(1<<6); //must bee zero
        //write Watchdg_tim_ctl register
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_WDOG_CTL, &reg16, 1);
        if (err) {
            dev_err(dev,
                "%s: err=%d", __func__, err);
            return err;
        }
        //get control 1 register
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0,sizeof(reg0));
        if (ret) {
            dev_err(dev, "%s: read error\n", __func__);
            return ret;
        }
        //Set SI bit
        reg0 |= PCF2127_REG_CTRL1_SI;
        //write control 1 register
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0, 1);
        if (err) {
            dev_err(dev,
                "%s: err=%d", __func__, err);
            return err;
        }

        return 0;

    case RTC_STOP_1HZ:	/* ... off			*/
//        printk("%s RTC_STOP_1HZ\n",__func__);
        //get control 1 register
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0,sizeof(reg0));
        if (ret) {
            dev_err(dev, "%s: read error\n", __func__);
            return ret;
        }
        //Clear SI bit
        reg0 &= ~PCF2127_REG_CTRL1_SI;
        //write control 1 register
        err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_CTRL1, &reg0, 1);
        if (err) {
            dev_err(dev,
                "%s: err=%d", __func__, err);
            return err;
        }
    return 0;
    case RTC_BK_SET_TIME:
        rtc_set_precise(pcf2127);
        return 0;
    case RTC_BK_GET_AGING:
    case RTC_BK_SET_AGING:
        if (arg == 0)
            return -EINVAL;
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_AGING, &old_aging,sizeof(old_aging));
        if (ret) {
            dev_err(dev, "%s: RTC_BK_SET_AGING read error\n", __func__);
            return ret;
        }
        old_aging &= 0xF;
        touser = 8 - old_aging;
        if (cmd == RTC_BK_SET_AGING)
        {
            if (arm_copy_from_user(&user_arg,(void __user *)arg,sizeof(int)))
                return -EFAULT;
            aging = (8 - user_arg) & 0xF;
//            printk("%s RTC_BK_SET_AGING %d reg_value=0x%x old=%d\n",__func__,user_arg,aging,touser);
            err = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_AGING, &aging, 1);
            if (err) {
                dev_err(dev,
                    "%s: err=%d", __func__, err);
                return err;
            }
        }
		if (copy_to_user((void __user *)arg, &touser, sizeof(int)))
			return -EFAULT;
        return 0;
    case RTC_BK_DUMP:
    {
        unsigned char buf[26];
        ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, buf,sizeof(buf));
        if (ret) {
            dev_err(dev, "%s: read error\n", __func__);
            return ret;
        }
        dump_regs(buf,sizeof(buf));
        return 0;
    }
	default:
		return -ENOIOCTLCMD;
	}
}
#else
#define pcf2127_rtc_ioctl NULL
#endif

static const struct rtc_class_ops pcf2127_rtc_ops = {
	.ioctl		= pcf2127_rtc_ioctl,
	.read_time	= pcf2127_rtc_read_time,
	.set_time	= pcf2127_rtc_set_time,
	.read_alarm = pcf2127_rtc_read_alarm,
	.set_alarm = pcf2127_rtc_set_alarm,
	.alarm_irq_enable = pcf2127_rtc_alarm_irq_enable,
};

static int pcf2127_probe(struct device *dev, struct regmap *regmap,
			const char *name)
{
	struct pcf2127_t *pcf2127;

	printk("%s name=%s dev=%p\n", __func__,name,dev);

	pcf2127 = devm_kzalloc(dev, sizeof(*pcf2127), GFP_KERNEL);
	if (!pcf2127)
		return -ENOMEM;

	pcf2127->regmap = regmap;

	dev_set_drvdata(dev, pcf2127);
	INIT_WORK(&pcf2127->work, rtc_set_work_handler);

	pcf2127->rtc = devm_rtc_device_register(dev, name, &pcf2127_rtc_ops,
						THIS_MODULE);
	printk("%s %p %p\n", __func__,pcf2127,pcf2127->rtc);
	printk("%s %s\n", __func__,pcf2127->rtc->name);
    hrtimer_init(&pcf2127->hr_timer, CLOCK_REALTIME, HRTIMER_MODE_ABS );
	return PTR_ERR_OR_ZERO(pcf2127->rtc);
}

#ifdef CONFIG_OF
static const struct of_device_id pcf2127_of_match[] = {
	{ .compatible = "nxp,pcf2127" },
	{ .compatible = "nxp,pcf2129" },
	{}
};
MODULE_DEVICE_TABLE(of, pcf2127_of_match);
#endif

#if IS_ENABLED(CONFIG_I2C)

static int pcf2127_i2c_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = i2c_master_send(client, data, count);
	if (ret != count)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int pcf2127_i2c_gather_write(void *context,
				const void *reg, size_t reg_size,
				const void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	char buf[40];

	if (WARN_ON(reg_size != 1))
		return -EINVAL;

	if (WARN_ON(val_size >= (40-1)))
		return -EINVAL;

	memcpy(buf, reg, 1);
	memcpy(buf + 1, val, val_size);

	ret = i2c_master_send(client, buf, val_size + 1);
	if (ret != val_size + 1)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int pcf2127_i2c_read(void *context, const void *reg, size_t reg_size,
				void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	if (WARN_ON(reg_size != 1))
		return -EINVAL;

	if (in_atomic() || irqs_disabled()) {
		ret = i2c_trylock_bus(adap, I2C_LOCK_SEGMENT);
		if (!ret)
			/* I2C activity is ongoing. */
			return -EAGAIN;
	} else {
		i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	}

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 1;
	msg.buf = (char *)reg;

	ret = __i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
		return ret < 0 ? ret : -EIO;
	}

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = val_size;
	msg.buf = (char *)val;

	ret = __i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
		return ret < 0 ? ret : -EIO;
	}

	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
	return 0;
}

/*
 * The reason we need this custom regmap_bus instead of using regmap_init_i2c()
 * is that the STOP condition is required between set register address and
 * read register data when reading from registers.
 */
static const struct regmap_bus pcf2127_i2c_regmap = {
	.write = pcf2127_i2c_write,
	.gather_write = pcf2127_i2c_gather_write,
	.read = pcf2127_i2c_read,
};

static struct i2c_driver pcf2127_i2c_driver;

static int pcf2127_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct regmap *regmap;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	regmap = devm_regmap_init(&client->dev, &pcf2127_i2c_regmap,
					&client->dev, &config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

/*
	if (client->irq >= 0) {
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					       pcf2127_rtc_interrupt,
					       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					       pcf2127_i2c_driver.driver.name,
					       &client->dev);
		if (!err) {
*/
		err = device_init_wakeup(&client->dev, 1);
/*
		} else {
			dev_warn(&client->dev,
				"%s: Unable to request irq %d, no alarm support, err %d\n",
				__func__, client->irq, err);
			client->irq = 0;
		}
	}
*/
printk("pcf2127_i2c_driver.driver.name=%s err=%d\n", pcf2127_i2c_driver.driver.name,err);
	err = pcf2127_probe(&client->dev, regmap, pcf2127_i2c_driver.driver.name);
	if (err) {
		dev_err(&client->dev, "%s: Probe failed, err %d\n", __func__, err);
		return err;
	}

	return pcf2127_rtc_alarm_irq_enable(&client->dev, 0);
}

static const struct i2c_device_id pcf2127_i2c_id[] = {
	{ "pcf2127", 0 },
	{ "pcf2129", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf2127_i2c_id);

static int pcf2127_i2c_remove(struct i2c_client *client)
{
    struct pcf2127_t *pcf2127 = (struct pcf2127_t *)i2c_get_clientdata(client);
    if (pcf2127)
    {
        hrtimer_cancel(&pcf2127->hr_timer);
        flush_scheduled_work();
    }
	return 0;
}

static struct i2c_driver pcf2127_i2c_driver = {
	.driver		= {
		.name	= "rtc-pcf2127-i2c",
		.of_match_table = of_match_ptr(pcf2127_of_match),
	},
	.probe		= pcf2127_i2c_probe,
	.remove     = pcf2127_i2c_remove,
	.id_table	= pcf2127_i2c_id,
};

static int pcf2127_i2c_register_driver(void)
{
	return i2c_add_driver(&pcf2127_i2c_driver);
}

static void pcf2127_i2c_unregister_driver(void)
{
	i2c_del_driver(&pcf2127_i2c_driver);
}

#else

static int pcf2127_i2c_register_driver(void)
{
	return 0;
}

static void pcf2127_i2c_unregister_driver(void)
{
}

#endif

#if IS_ENABLED(CONFIG_SPI_MASTER)

static struct spi_driver pcf2127_spi_driver;

static int pcf2127_spi_probe(struct spi_device *spi)
{
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
		.read_flag_mask = 0xa0,
		.write_flag_mask = 0x20,
	};
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return pcf2127_probe(&spi->dev, regmap, pcf2127_spi_driver.driver.name);
}

static const struct spi_device_id pcf2127_spi_id[] = {
	{ "pcf2127", 0 },
	{ "pcf2129", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, pcf2127_spi_id);

static struct spi_driver pcf2127_spi_driver = {
	.driver		= {
		.name	= "rtc-pcf2127-spi",
		.of_match_table = of_match_ptr(pcf2127_of_match),
	},
	.probe		= pcf2127_spi_probe,
	.id_table	= pcf2127_spi_id,
};

static int pcf2127_spi_register_driver(void)
{
	return spi_register_driver(&pcf2127_spi_driver);
}

static void pcf2127_spi_unregister_driver(void)
{
	spi_unregister_driver(&pcf2127_spi_driver);
}

#else

static int pcf2127_spi_register_driver(void)
{
	return 0;
}

static void pcf2127_spi_unregister_driver(void)
{
}

#endif

static int __init pcf2127_init(void)
{
	int ret;

	ret = pcf2127_i2c_register_driver();
	if (ret) {
		pr_err("Failed to register pcf2127 i2c driver: %d\n", ret);
		return ret;
	}

	ret = pcf2127_spi_register_driver();
	if (ret) {
		pr_err("Failed to register pcf2127 spi driver: %d\n", ret);
		pcf2127_i2c_unregister_driver();
	}

	return ret;
}
module_init(pcf2127_init)

static void __exit pcf2127_exit(void)
{
	pcf2127_spi_unregister_driver();
	pcf2127_i2c_unregister_driver();
}
module_exit(pcf2127_exit)

MODULE_AUTHOR("Renaud Cerrato <r.cerrato@til-technologies.fr>");
MODULE_DESCRIPTION("NXP PCF2127/29 RTC driver");
MODULE_LICENSE("GPL v2");
