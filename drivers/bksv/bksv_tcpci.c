/*
 * Copyright 2015-2017 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * USB Type-C Port Controller Interface.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include "bksv_tcpci.h"

#define PD_RETRY_COUNT 3
#define TCPC_POWER_CTRL_DEF_VAL 0x60

#if 0
static void  tcpci_dump(struct tcpc_dev *tcpc);
#endif
extern void _tcpm_log(struct tcpm_port *port, const char *fmt, ...);
extern void  tcpm_log(struct tcpm_port *port, const char *fmt, ...);

#if 0
#include <linux/of_address.h>
#include <linux/of_gpio.h>

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

void blip(void)
{     
    struct mxc_gpio_hwdata *gpio7_addr=(struct mxc_gpio_hwdata *)ioremap_nocache(0x30260000,4096);

    if (gpio7_addr!=NULL)
    {
        gpio7_addr->gdir_reg |=  (1<<13);
        gpio7_addr->dr_reg   &= ~(1<<13);
        msleep(2);
        gpio7_addr->dr_reg   |=  (1<<13);
        msleep(2);
        gpio7_addr->dr_reg   &= ~(1<<13);
        iounmap(gpio7_addr);
   }
}
#endif
/* Enables/disables VBUS supplied by the SXU */
/* Implemented in drivers/power/supply/bq25890_charger.c */
extern int bq25890_set_vbus(struct device *dev, bool enable);
extern int bq25890_get_vbus(struct device *dev, int *stat, int *fault, int *otg);

/* Configures the amount of current the SXU is allowed to draw from USB */
/* Implemented in drivers/power/supply/bq25890_charger.c */
extern int bq25890_set_current_limit(struct device *dev, u32 max_ma);

/* Must match the definition in drivers/usb/chipidea/ci.h */
enum ci_role {
	CI_ROLE_HOST = 0,
	CI_ROLE_GADGET,
	CI_ROLE_END,
};

/* Selects whether the SXU should be USB host or USB device (gadget) */
/* Implemented in drivers/usb/chipidea/core.c */
extern int ci_hdrc_select_role(struct device *dev, enum ci_role role);

/* Returns non-zero if the port partner supports USB Battery Charging,
 * i.e. charging at up to 1.5 A from a standard Type-A port */
int ci_hdrc_bc_supported(struct device *dev);

/* Enables USB through the USB connector,              disable =false*/
/* Enables analog audio out through the USB connector, disable =true */ 
/* Implemented in drivers/bksv/bksv_gpio.c */
extern int bksv_gpio_usb_en_n(struct device *dev, bool disable);

struct tcpci {
	struct device *dev;
	struct i2c_client *client;

	struct tcpm_port *port;

	struct regmap *regmap;

	bool controls_vbus;
	enum typec_acc_type acc_type;
	struct device *bq25890_dev;
	struct device *ci_hdrc_dev;
	struct device *bksv_gpio_dev;

	struct tcpc_dev tcpc;
};

struct tcpci *g_tcpci;

static void tcpci_put_refs(struct tcpci *tcpci)
{
	if (tcpci->bksv_gpio_dev) {
		put_device(tcpci->bksv_gpio_dev);
		tcpci->bksv_gpio_dev = NULL;
	}

	if (tcpci->bq25890_dev) {
		put_device(tcpci->bq25890_dev);
		tcpci->bq25890_dev = NULL;
	}

	if (tcpci->ci_hdrc_dev) {
		put_device(tcpci->ci_hdrc_dev);
		tcpci->ci_hdrc_dev = NULL;
	}
}

static inline struct tcpci *tcpc_to_tcpci(struct tcpc_dev *tcpc)
{
	return container_of(tcpc, struct tcpci, tcpc);
}

static int tcpci_read8(struct tcpci *tcpci, unsigned int reg,
			u8 *val)
{
	return regmap_raw_read(tcpci->regmap, reg, val, sizeof(u8));
}

static int tcpci_write8(struct tcpci *tcpci, unsigned int reg, u8 val)
{
	return regmap_raw_write(tcpci->regmap, reg, &val, sizeof(u8));
}

static int tcpci_read16(struct tcpci *tcpci, unsigned int reg,
			u16 *val)
{
	return regmap_raw_read(tcpci->regmap, reg, val, sizeof(u16));
}

static int tcpci_write16(struct tcpci *tcpci, unsigned int reg, u16 val)
{
	return regmap_raw_write(tcpci->regmap, reg, &val, sizeof(u16));
}

static int tcpci_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc,
			bool attached, enum typec_cc_polarity polarity)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = 0, reg_cc1 = 0, reg_cc2 = 0;
	int ret;

	switch (cc) {
	case TYPEC_CC_RA:
		reg = (TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	case TYPEC_CC_RD:
		reg_cc1 = TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT;
		reg_cc2 = TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT;
		break;
	case TYPEC_CC_RP_DEF:
		reg_cc1 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT;
		reg_cc2 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT;
		reg = TCPC_ROLE_CTRL_RP_VAL_DEF << TCPC_ROLE_CTRL_RP_VAL_SHIFT;
		break;
	case TYPEC_CC_RP_1_5:
		reg_cc1 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT;
		reg_cc2 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT;
		reg = TCPC_ROLE_CTRL_RP_VAL_1_5 << TCPC_ROLE_CTRL_RP_VAL_SHIFT;
		break;
	case TYPEC_CC_RP_3_0:
		reg_cc1 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT;
		reg_cc2 = TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT;
		reg = TCPC_ROLE_CTRL_RP_VAL_3_0 << TCPC_ROLE_CTRL_RP_VAL_SHIFT;
		break;
	case TYPEC_CC_OPEN:
	default:
		reg = (TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	}

	if (!attached)
		reg |= reg_cc1 | reg_cc2;
	else if (polarity == TYPEC_POLARITY_CC1)
		reg |= reg_cc1;
	else
		reg |= reg_cc2;

	ret = regmap_write(tcpci->regmap, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_start_drp_toggling(struct tcpc_dev *tcpc,
				    enum typec_cc_status cc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = TCPC_ROLE_CTRL_DRP;
	int ret;

	switch (cc) {
	default:
	case TYPEC_CC_RP_DEF:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	}

	if (cc == TYPEC_CC_RD)
		reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
	else
		reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);

	ret = regmap_write(tcpci->regmap, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;

	return regmap_write(tcpci->regmap, TCPC_COMMAND,
				TCPC_CMD_LOOK4CONNECTION);
}

static enum typec_cc_status tcpci_to_typec_cc(unsigned int cc, bool sink)
{
	switch (cc) {
	case 0x1:
		return sink ? TYPEC_CC_RP_DEF : TYPEC_CC_RA;
	case 0x2:
		return sink ? TYPEC_CC_RP_1_5 : TYPEC_CC_RD;
	case 0x3:
		if (sink)
			return TYPEC_CC_RP_3_0;
		/* fall through */
	case 0x0:
	default:
		return TYPEC_CC_OPEN;
	}
}

static int tcpci_get_cc(struct tcpc_dev *tcpc,
			enum typec_cc_status *cc1, enum typec_cc_status *cc2)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_CC_STATUS, &reg);
	if (ret < 0)
		return ret;

	*cc1 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC1_SHIFT) &
				 TCPC_CC_STATUS_CC1_MASK,
				 reg & TCPC_CC_STATUS_TERM);
	*cc2 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC2_SHIFT) &
				 TCPC_CC_STATUS_CC2_MASK,
				 reg & TCPC_CC_STATUS_TERM);

	return 0;
}

static int tcpci_set_polarity(struct tcpc_dev *tcpc,
			      enum typec_cc_polarity polarity)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	ret = regmap_write(tcpci->regmap, TCPC_TCPC_CTRL,
			   (polarity == TYPEC_POLARITY_CC2) ?
			   TCPC_TCPC_CTRL_ORIENTATION : 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_set_vconn(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);

	return regmap_update_bits(tcpci->regmap, TCPC_POWER_CTRL,
				TCPC_POWER_CTRL_VCONN_ENABLE,
				enable ? TCPC_POWER_CTRL_VCONN_ENABLE : 0);
}

static int tcpci_set_roles(struct tcpc_dev *tcpc, bool attached,
			   enum typec_role role, enum typec_data_role data)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	enum ci_role otg_role;
	int ret;

	if (!tcpci->ci_hdrc_dev)
		return -EIO;

	reg = PD_REV20 << TCPC_MSG_HDR_INFO_REV_SHIFT;
	if (role == TYPEC_SOURCE)
		reg |= TCPC_MSG_HDR_INFO_PWR_ROLE;
	if (data == TYPEC_HOST)
		reg |= TCPC_MSG_HDR_INFO_DATA_ROLE;
	ret = regmap_write(tcpci->regmap, TCPC_MSG_HDR_INFO, reg);
	if (ret < 0)
		return ret;

	if (attached && data == TYPEC_HOST)
		otg_role = CI_ROLE_HOST;
	else
		otg_role = CI_ROLE_GADGET;

	ret = ci_hdrc_select_role(tcpci->ci_hdrc_dev, otg_role);
	if (ret < 0)
		return ret;

	dev_notice(tcpci->dev, "Type-C data %s\n", (otg_role == CI_ROLE_HOST) ? "host" : "device");
	return 0;
}

static int tcpci_set_stack(struct tcpc_dev *tcpc, enum typec_data_role data)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	enum ci_role otg_role;
	int ret;

	if (!tcpci->ci_hdrc_dev)
		return -EIO;

	if (data == TYPEC_HOST)
		otg_role = CI_ROLE_HOST;
	else
		otg_role = CI_ROLE_GADGET;

	ret = ci_hdrc_select_role(tcpci->ci_hdrc_dev, otg_role);
	if (ret < 0)
		return ret;

	dev_notice(tcpci->dev, "Type-C data %s\n", (otg_role == CI_ROLE_HOST) ? "host" : "device");
	return 0;
}

/*
 * B&K extension to the tcpci driver, sets USB-C accessory mode.
 * Most of the work is done by the tcpm, but we'll need to route
 * audio depending on the accessory type
 */
static int tcpci_set_acc(struct tcpc_dev *tcpc, bool attached, enum typec_acc_type acc_type)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	bool enable_audio = (acc_type == TYPEC_ACC_AUDIO) && attached;

	if (!tcpci->bksv_gpio_dev)
		return -EIO;

	tcpci->acc_type = attached ? acc_type : TYPEC_ACC_NONE;

	dev_notice(tcpci->dev, "Type-C %s %s\n",
		acc_type == TYPEC_ACC_AUDIO ? "audio accessory" :
		acc_type == TYPEC_ACC_DEBUG_SOURCE_RP_DEF ? "debug accessory (host, 500 mA)" :
		acc_type == TYPEC_ACC_DEBUG_SOURCE_RP_1_5 ? "B&K debug board" :
		acc_type == TYPEC_ACC_DEBUG_SOURCE_RP_3_0 ? "B&K base station" :
		acc_type == TYPEC_ACC_DEBUG_SINK ? "debug accessory (device)" : "(unknown accessory)",
		attached ? "attached" : "detached"
	);
    //select USB or audio on the usb lines, if the signal from the DAC on the analog board is
    //  to bee send out as the audio signal, then the SLM app must configure the DAC and open the
    //  signal routing by setting gpio pin PTB_EN high.
	return bksv_gpio_usb_en_n(tcpci->bksv_gpio_dev, enable_audio);
}

static int tcpci_set_pd_rx(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = 0;
	int ret;

	if (enable)
		reg = TCPC_RX_DETECT_SOP | TCPC_RX_DETECT_HARD_RESET;
	ret = regmap_write(tcpci->regmap, TCPC_RX_DETECT, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_get_vbus(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
	if (ret < 0)
		return ret;

	return !!(reg & TCPC_POWER_STATUS_VBUS_PRES);
}

static int tcpci_set_vbus(struct tcpc_dev *tcpc, bool source, bool sink)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;
	unsigned int reg;
	bool is_source_usb, is_sink;
    int is_source_boost;
    int stat,fault,boost;

	if (!tcpci->bq25890_dev)
		return -EIO;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
	if (ret < 0)
		return ret;

	is_source_usb = !!(reg & TCPC_POWER_STATUS_VBUS_SRC);
	is_sink = !!(reg & TCPC_POWER_STATUS_VBUS_SNK);
    bq25890_get_vbus(tcpci->bq25890_dev,NULL,NULL,&is_source_boost);
#if 0
    if (tcpci->port)
        tcpm_log(tcpci->port, "tcpci_set_vbus source=%d, sink=%d is_source_usb=%d,is_source_boost=%d is_sink=%d",
                     source,  sink,is_source_usb,is_source_boost, is_sink);
#endif

	/* Disable both source and sink first before enabling anything */
	if (!source)
    { 
        if (is_source_usb)
        {
            ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
                    TCPC_CMD_DISABLE_SRC_VBUS);
            if (ret < 0)
                return ret;
        }
        if (is_source_boost)
        {
            ret = bq25890_set_vbus(tcpci->bq25890_dev, false);
            if (ret < 0)
                return ret;
	    }
    }

	if (!sink && is_sink) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	if (source && !is_source_usb) {
        int i;
        u16 low_vol;
        //enable the boost converter
		ret = bq25890_set_vbus(tcpci->bq25890_dev, true);
		if (ret < 0)
			return ret;
        //wait for the boost converter to start (0-1sec)
        for (i=0;i<120;i++)
        {
            msleep(10);
#if 1
            bq25890_get_vbus(tcpci->bq25890_dev,&stat,NULL,NULL);
#else            
            bq25890_get_vbus(tcpci->bq25890_dev,&stat,&fault,&boost);
            if (tcpci->port)
            {
                tcpm_log(tcpci->port, "bq25890_get_vbus i=%d stat=%d fault=%d",i,stat,fault);
            }
#endif
            if ((stat==7))
                break;
        }
        if (stat != 7)
        {
            dev_err(tcpci->dev, "Boost converter didn't start stat=%d\n",stat);
		    bq25890_set_vbus(tcpci->bq25890_dev, false);
            return -EIO;
        }
        //remember low voltage alarm
       	ret = tcpci_read16(tcpci, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, &low_vol);
		if (ret < 0)
			return ret;
        //set low voltage alarm to 0.25V
        //to avoid the chip disconnect extern VBUS
        ret = tcpci_write16(tcpci, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, 10);
		if (ret < 0)
			return ret;
        msleep(10);// wait for internal caps to charge
        bq25890_get_vbus(tcpci->bq25890_dev,&stat,&fault,&boost);
        if ((fault) || (stat!=7) || (!boost))
        {
            msleep(40);//extra wait for boost hickup doing internal caps charge
        }
        //turn on extern VBUS
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,TCPC_CMD_SRC_VBUS_DEFAULT);
		if (ret < 0)
			return ret;
        msleep(40); //wait for possible boost converter hicc-up.

        //set low voltage alarm to what it was
        ret = tcpci_write16(tcpci, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, low_vol);
		if (ret < 0)
			return ret;
	}

	if (sink && !is_sink) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	dev_notice(tcpci->dev, "Type-C power %s\n",
		source && !sink ? "source" :
		!source && sink ? "sink" :
		!source && !sink ? "none" : "illegal"
	);

	return 0;
}

// Called by the tcpm when configured as a snk and cc=Rp-def
static int tcpci_get_current_limit(struct tcpc_dev *dev)
{
	return 500; // 500 mA
}

static int tcpci_set_current_limit(struct tcpc_dev *dev, u32 max_ma, u32 mv)
{
	struct tcpci *tcpci = tcpc_to_tcpci(dev);
	u32 bc_supported;
	u32 max_ma_requested = max_ma;

	if (!tcpci->bq25890_dev || !tcpci->ci_hdrc_dev)
		return -EIO;

	/* We don't support anything other than 5V VBUS.
	 * The tcpm likes to pass 0 too, so accept that as well */
	if (mv != 5000 && mv != 0)
		return -EOPNOTSUPP;

	/* tcpm can request a current limit of 0, but the SXU doesn't
	 * support that, so set a minimum of 100 mA */
	if (max_ma < 100)
		max_ma = 100;

	dev_notice(tcpci->dev, "Type-C current limit %d mA (tcpm requested %d mV %d mA)\n",
		max_ma,
		mv,
		max_ma_requested
	);

	bc_supported = ci_hdrc_bc_supported(tcpci->ci_hdrc_dev);
	if (max_ma == 500 && bc_supported) {
		max_ma = 1500;
		dev_notice(tcpci->dev,
			"Partner supports USB Battery Charging, increasing current to %d mA\n",
			max_ma
		);
	}

	return bq25890_set_current_limit(tcpci->bq25890_dev, max_ma);
}

static int tcpci_pd_transmit(struct tcpc_dev *tcpc,
			     enum tcpm_transmit_type type,
			     const struct pd_message *msg)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	u16 header = msg ? le16_to_cpu(msg->header) : 0;
	unsigned int reg, cnt;
	int ret;

	cnt = msg ? pd_header_cnt(header) * 4 : 0;
	ret = regmap_write(tcpci->regmap, TCPC_TX_BYTE_CNT, cnt + 2);
	if (ret < 0)
		return ret;

	ret = tcpci_write16(tcpci, TCPC_TX_HDR, header);
	if (ret < 0)
		return ret;

	if (cnt > 0) {
		ret = regmap_raw_write(tcpci->regmap, TCPC_TX_DATA,
				       &msg->payload, cnt);
		if (ret < 0)
			return ret;
	}

	reg = (PD_RETRY_COUNT << TCPC_TRANSMIT_RETRY_SHIFT) |
		(type << TCPC_TRANSMIT_TYPE_SHIFT);
	ret = regmap_write(tcpci->regmap, TCPC_TRANSMIT, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_vbus_detect(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	if (enable) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_ENABLE_VBUS_DETECT);
		if (ret < 0)
			return ret;
	} else {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_VBUS_DETECT);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int tcpci_init(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned long timeout = jiffies + msecs_to_jiffies(2000); /* XXX */
	unsigned int reg;
	int ret;

	while (time_before_eq(jiffies, timeout)) {
		ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
		if (ret < 0)
			return ret;
		if (!(reg & TCPC_POWER_STATUS_UNINIT))
			break;
		usleep_range(10000, 20000);
	}
	if (time_after(jiffies, timeout))
		return -ETIMEDOUT;


	/* Clear the reset 'fault' which is expected after power-on */
	ret = tcpci_write8(tcpci, TCPC_FAULT_STATUS, TCPC_FAULT_STATUS_RESET);
	if (ret < 0)
		return ret;

	/* Clear all events */
	ret = tcpci_write16(tcpci, TCPC_ALERT, 0xffff);
	if (ret < 0)
		return ret;

	if (tcpci->controls_vbus)
		reg = TCPC_POWER_STATUS_VBUS_PRES;
	else
		reg = 0;
	ret = regmap_write(tcpci->regmap, TCPC_POWER_STATUS_MASK, reg);
	if (ret < 0)
		return ret;

	/* SLX-273: SXU doesn't charge if USB-C charger connected at power-up.
	 * Initialize POWER_CONTROL to default value as defined in TCPCI spec. */
	ret = regmap_write(tcpci->regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_DEF_VAL);
	if (ret < 0)
		return ret;

	/* Enable Vbus detection */
	tcpci_vbus_detect(tcpc, true);

	reg = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED |
		TCPC_ALERT_TX_DISCARDED | TCPC_ALERT_RX_STATUS |
		TCPC_ALERT_RX_HARD_RST | TCPC_ALERT_CC_STATUS;
	if (tcpci->controls_vbus)
		reg |= TCPC_ALERT_POWER_STATUS;
	return tcpci_write16(tcpci, TCPC_ALERT_MASK, reg);
}

static irqreturn_t tcpci_irq(int irq, void *dev_id)
{
	struct tcpci *tcpci = dev_id;
	u16 status;

	tcpci_read16(tcpci, TCPC_ALERT, &status);

#if 0
    if (tcpci->port)
        tcpm_log(tcpci->port, "irq status 0x%x", status);
#endif

	if (status & TCPC_ALERT_FAULT) {
		/* Need to clear the fault register, or interrupt will never get de-asserted */
		int ifault;
		u8 fault;
		tcpci_read8(tcpci, TCPC_FAULT_STATUS, &fault);
		tcpci_write8(tcpci, TCPC_FAULT_STATUS, fault);

		ifault = fault;
		dev_warn(tcpci->dev, "tcpc reported fault 0x%x\n", ifault);
	}

	/*
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 */
	if (status & ~TCPC_ALERT_RX_STATUS)
		tcpci_write16(tcpci, TCPC_ALERT,
			      status & ~TCPC_ALERT_RX_STATUS);

	if (status & TCPC_ALERT_CC_STATUS)
		tcpm_cc_change(tcpci->port);

	if (status & TCPC_ALERT_POWER_STATUS) {
		unsigned int reg;

		regmap_read(tcpci->regmap, TCPC_POWER_STATUS_MASK, &reg);

		/*
		 * If power status mask has been reset, then the TCPC
		 * has reset.
		 */
		if (reg == 0xff)
        {
            if (tcpci->port)
                tcpm_log(tcpci->port, "If power status mask has been reset, then the TCPC has reset");
			tcpm_tcpc_reset(tcpci->port);
        }
		else
			tcpm_vbus_change(tcpci->port);
	}

	if (status & TCPC_ALERT_RX_STATUS) {
		struct pd_message msg;
		unsigned int cnt;
		u16 header;

		regmap_read(tcpci->regmap, TCPC_RX_BYTE_CNT, &cnt);

		tcpci_read16(tcpci, TCPC_RX_HDR, &header);
		msg.header = cpu_to_le16(header);

		if (WARN_ON(cnt > sizeof(msg.payload)))
			cnt = sizeof(msg.payload);

		if (cnt > 0)
			regmap_raw_read(tcpci->regmap, TCPC_RX_DATA,
					&msg.payload, cnt);

		/* Read complete, clear RX status alert bit */
		tcpci_write16(tcpci, TCPC_ALERT, TCPC_ALERT_RX_STATUS);

		tcpm_pd_receive(tcpci->port, &msg);
	}

	if (status & TCPC_ALERT_RX_HARD_RST)
		tcpm_pd_hard_reset(tcpci->port);

	if (status & TCPC_ALERT_TX_SUCCESS)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_SUCCESS);
	else if (status & TCPC_ALERT_TX_DISCARDED)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_DISCARDED);
	else if (status & TCPC_ALERT_TX_FAILED)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_FAILED);

	return IRQ_HANDLED;
}

static const struct regmap_config tcpci_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x7F, /* 0x80 .. 0xFF are vendor defined */
};

/* Populate struct tcpc_config from device-tree */
static int tcpci_parse_config(struct tcpci *tcpci)
{
	struct tcpc_config *tcfg;
	int ret = -EINVAL;

 	tcpci->controls_vbus = true; /* XXX */

	tcpci->tcpc.config = devm_kzalloc(tcpci->dev, sizeof(*tcfg),
					  GFP_KERNEL);
	if (!tcpci->tcpc.config)
		return -ENOMEM;

	tcfg = tcpci->tcpc.config;

	/* Get port-type */
	ret = typec_get_port_type(tcpci->dev);
	if (ret < 0) {
		dev_err(tcpci->dev, "typec port type is NOT correct!\n");
		return ret;
	}
	tcfg->type = ret;

	if (tcfg->type == TYPEC_PORT_UFP)
		goto sink;

	/* Get source PDO */
	tcfg->nr_src_pdo = device_property_read_u32_array(tcpci->dev,
						"src-pdos", NULL, 0);
	if (tcfg->nr_src_pdo <= 0) {
		dev_err(tcpci->dev, "typec source pdo is missing!\n");
		return -EINVAL;
	}

	tcfg->src_pdo = devm_kzalloc(tcpci->dev,
		sizeof(*tcfg->src_pdo) * tcfg->nr_src_pdo, GFP_KERNEL);
	if (!tcfg->src_pdo)
		return -ENOMEM;

	ret = device_property_read_u32_array(tcpci->dev, "src-pdos",
				tcfg->src_pdo, tcfg->nr_src_pdo);
	if (ret) {
		dev_err(tcpci->dev, "Failed to read src pdo!\n");
		return -EINVAL;
	}

	if (tcfg->type == TYPEC_PORT_DFP)
		return 0;

	/* Get the preferred power role for drp */
	ret = typec_get_power_role(tcpci->dev);
	if (ret < 0) {
		dev_err(tcpci->dev, "typec preferred role is wrong!\n");
		return ret;
	}
	tcfg->default_role = ret;
sink:
	/* Get sink power capability */
	tcfg->nr_snk_pdo = device_property_read_u32_array(tcpci->dev,
						"snk-pdos", NULL, 0);
	if (tcfg->nr_snk_pdo <= 0) {
		dev_err(tcpci->dev, "typec sink pdo is missing!\n");
		return -EINVAL;
	}

	tcfg->snk_pdo = devm_kzalloc(tcpci->dev,
		sizeof(*tcfg->snk_pdo) * tcfg->nr_snk_pdo, GFP_KERNEL);
	if (!tcfg->snk_pdo)
		return -ENOMEM;

	ret = device_property_read_u32_array(tcpci->dev, "snk-pdos",
				tcfg->snk_pdo, tcfg->nr_snk_pdo);
	if (ret) {
		dev_err(tcpci->dev, "Failed to read sink pdo!\n");
		return -EINVAL;
	}

	if (device_property_read_u32(tcpci->dev, "max-snk-mv",
				     &tcfg->max_snk_mv) ||
		device_property_read_u32(tcpci->dev, "max-snk-ma",
					 &tcfg->max_snk_ma) ||
		device_property_read_u32(tcpci->dev, "max-snk-mw",
					 &tcfg->max_snk_mw) ||
		device_property_read_u32(tcpci->dev, "op-snk-mw",
					 &tcfg->operating_snk_mw)) {
		dev_err(tcpci->dev, "Failed to read sink capability!\n");
		return -EINVAL;
	}

	return 0;
}

static int tcpci_reboot_callback(struct notifier_block *self,
				unsigned long val, void *data)
{
	/*
	 * The SXU is rebooting or shutting down.
	 * Release (put) references to other kernel drivers
	 * for an orderly shutdown, and to prevent tcpci from
	 * calling into drivers that may have been removed.
	 */
	tcpci_put_refs(g_tcpci);

	return NOTIFY_DONE;
}

static struct notifier_block tcpci_reboot_notifier = {
	.notifier_call = tcpci_reboot_callback,
};

static inline int match_device(struct device *dev, void *data)
{
	return strcmp(dev_name(dev), (const char *)data) == 0;
}

static int tcpci_probe(struct i2c_client *client,
		       const struct i2c_device_id *i2c_id)
{
	struct tcpci *tcpci;
	int err;

	tcpci = devm_kzalloc(&client->dev, sizeof(*tcpci), GFP_KERNEL);
	if (!tcpci)
		return -ENOMEM;

	tcpci->client = client;
	tcpci->dev = &client->dev;
	i2c_set_clientdata(client, tcpci);
	tcpci->regmap = devm_regmap_init_i2c(client, &tcpci_regmap_config);
	if (IS_ERR(tcpci->regmap))
		return PTR_ERR(tcpci->regmap);

	tcpci->tcpc.init = tcpci_init;
	tcpci->tcpc.get_vbus = tcpci_get_vbus;
	tcpci->tcpc.set_vbus = tcpci_set_vbus;
	tcpci->tcpc.get_current_limit = tcpci_get_current_limit;
	tcpci->tcpc.set_current_limit = tcpci_set_current_limit;
	tcpci->tcpc.set_cc = tcpci_set_cc;
	tcpci->tcpc.get_cc = tcpci_get_cc;
	tcpci->tcpc.set_polarity = tcpci_set_polarity;
	tcpci->tcpc.set_vconn = tcpci_set_vconn;
	tcpci->tcpc.start_drp_toggling = tcpci_start_drp_toggling;

	tcpci->tcpc.set_pd_rx = tcpci_set_pd_rx;
	tcpci->tcpc.set_roles = tcpci_set_roles;
	tcpci->tcpc.set_stack = tcpci_set_stack;
	tcpci->tcpc.set_acc = tcpci_set_acc;
	tcpci->tcpc.pd_transmit = tcpci_pd_transmit;

	tcpci->acc_type = TYPEC_ACC_NONE;

	/* Get reference to USB OTG driver  */
	tcpci->ci_hdrc_dev = bus_find_device(
		&platform_bus_type, NULL, "ci_hdrc.0", match_device);
	if (!tcpci->ci_hdrc_dev) {
		dev_err(tcpci->dev, "ci_hdrc device not found\n");
		return -ENODEV;
	}

	/* Get reference to charger/supply driver  */
	tcpci->bq25890_dev = bus_find_device(
		&i2c_bus_type, NULL, "2-006b", match_device);
	if (!tcpci->bq25890_dev) {
		dev_err(tcpci->dev, "bq25890 device not found\n");
		return -ENODEV;
	}

	/* Get reference to GPIO driver  */
	tcpci->bksv_gpio_dev = bus_find_device(
		&platform_bus_type, NULL, "bksv_gpio", match_device);
	if (!tcpci->bksv_gpio_dev) {
		dev_err(tcpci->dev, "bksv_gpio device not found\n");
		return -ENODEV;
	}

	err = tcpci_parse_config(tcpci);
	if (err < 0)
		return err;

	g_tcpci = tcpci;
	err = register_reboot_notifier(&tcpci_reboot_notifier);
	if (err < 0)
		return err;

	/* Disable chip interrupts */
	tcpci_write16(tcpci, TCPC_ALERT_MASK, 0);

	tcpci->port = tcpm_register_port(tcpci->dev, &tcpci->tcpc);
	if (IS_ERR(tcpci->port))
		return PTR_ERR(tcpci->port);

	return devm_request_threaded_irq(tcpci->dev, client->irq, NULL,
					tcpci_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					dev_name(tcpci->dev), tcpci);
}

static int tcpci_remove(struct i2c_client *client)
{
	struct tcpci *tcpci = i2c_get_clientdata(client);

	tcpci_put_refs(tcpci);

	unregister_reboot_notifier(&tcpci_reboot_notifier);
	g_tcpci = NULL;

	tcpm_unregister_port(tcpci->port);

	return 0;
}

static const struct i2c_device_id tcpci_id[] = {
	{ "bksv_tcpci", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcpci_id);

#ifdef CONFIG_OF
static const struct of_device_id tcpci_of_match[] = {
	{ .compatible = "usb,bksv_tcpci", },
	{},
};
MODULE_DEVICE_TABLE(of, tcpci_of_match);
#endif

static struct i2c_driver tcpci_i2c_driver = {
	.driver = {
		.name = "bksv_tcpci",
		.of_match_table = of_match_ptr(tcpci_of_match),
	},
	.probe = tcpci_probe,
	.remove = tcpci_remove,
	.id_table = tcpci_id,
};
module_i2c_driver(tcpci_i2c_driver);

enum typec_acc_type tcpci_get_acc_type(struct device *dev)
{
	struct tcpci *tcpci = dev_get_drvdata(dev);
	return tcpci->acc_type;
}
EXPORT_SYMBOL_GPL(tcpci_get_acc_type);

#if 0
static void tcpci_dump(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int i;
	u8 reg8;
    if (tcpci->port)
    {
        tcpm_log(tcpci->port, "dump tcpci registers");
        for (i=0;i<256;i++)
        {
            tcpci_read8(tcpci, i, &reg8);
            tcpm_log(tcpci->port, "%02x %02x",i,reg8);
        }
    }
}
#endif

MODULE_DESCRIPTION("USB Type-C Port Controller Interface driver");
MODULE_LICENSE("GPL");
