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
 /*
  * 2017-03-13
  * Lars Thestrup:
  * Based on patch for iMX6UL and iMX7D by Qiang Li
  * who made MPU interface for displays using "Command-Parameter interface mode""
  * 
  * Added support for display COM24H2P39ULC with Himax HX8347-A01 controller
  * This display is 2.4" QVGA 240 x RGB x 320 Portrait, 
  *   using "Register-Content Interface Mode"
  * CPU LCD controller is set to 8080 mode
  * Bus interface is 18 bits
  * Input from layers above is 32 bits where lower 24 bits is used
  * Most significant 6 bits of every byte is sendt to display.
  */


#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>

#include "../mxsfb.h"

static struct fb_videomode com24h2p39_lcd_modedb[] =
{
	{
		.name = "COM24H2P39ULC",
		.refresh = 60,
		.xres = 240,
		.yres = 320,
		.pixclock = 120000,
		.left_margin =  0,
		.right_margin = 0,
		.upper_margin = 0,
		.lower_margin = 0,
		.hsync_len = 0,
		.vsync_len = 0,
		.sync  = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag  = 0,
	}

};

static struct mpu_lcd_config lcd_config = {
	.bus_mode = MPU_BUS_8080,
	.interface_width = 18,
	.panel_bpp = 18,      //bits pr pixel on the LCD
};
void mpu_com24h2p39_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mpu_lcd_config **data)
{
	*mode = &com24h2p39_lcd_modedb[0];
	*size = ARRAY_SIZE(com24h2p39_lcd_modedb);
	*data = &lcd_config;
}

typedef struct {
	int mode;
	int rw;
	unsigned int index;
	int data;
	unsigned int length;
	unsigned int msleep;
} startup_sequence_t;

startup_sequence_t startup_sequence[] = 
{
    //TEST1
	{MPU_DATA, MPU_WRITE,0x96, 0x01, 1,0},
	//OSC control 1
    {MPU_DATA, MPU_WRITE,0x19, 0x87, 1,12},

	/* Read display ID */
    {MPU_DATA, MPU_READ, 0x67, 0x00, 1,0},
 
	/* Display off setting */
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0x80, 1,0},
	// Power Control 1
    {MPU_DATA, MPU_WRITE,0x1B, 0x0C, 1,0},
	//VCOM Control 1
    {MPU_DATA, MPU_WRITE,0x43, 0x00, 1,0},

	/* Power supply setting initalizing*/
	// Power Control 6
    {MPU_DATA, MPU_WRITE,0x20, 0x00, 1,0},
	// Power Control 5
    {MPU_DATA, MPU_WRITE,0x1F, 0x07, 1,0},
	//VCOM Control 2
    {MPU_DATA, MPU_WRITE,0x44, 0x7F, 1,0},
	//VCOM Control 3
    {MPU_DATA, MPU_WRITE,0x45, 0x14, 1,0},
	// Power Control 3
    {MPU_DATA, MPU_WRITE,0x1D, 0x05, 1,0},
	// Power Control 4
    {MPU_DATA, MPU_WRITE,0x1E, 0x00, 1,0},

	/* Power supply operation start setting*/
	// Power Control 2
    {MPU_DATA, MPU_WRITE,0x1C, 0x04, 1,0},
	// Power Control 1
    {MPU_DATA, MPU_WRITE,0x1B, 0x14, 1,45},
	//VCOM Control 1
    {MPU_DATA, MPU_WRITE,0x43, 0x80, 1,0},


	/* Power supply control setting*/
	// BGP Control
    {MPU_DATA, MPU_WRITE,0x42, 0x08, 1,0},
	// Cycle Control 1
    {MPU_DATA, MPU_WRITE,0x23, 0x95, 1,0},
	// Cycle Control 2
    {MPU_DATA, MPU_WRITE,0x24, 0x95, 1,0},
	// Cycle Control 3
    {MPU_DATA, MPU_WRITE,0x25, 0xFF, 1,0},
	// Power Control 7
    {MPU_DATA, MPU_WRITE,0x21, 0x10, 1,0},
	// Power Control 11
    {MPU_DATA, MPU_WRITE,0x2B, 0x00, 1,0},
	// DCCLK SYNC TO CL1
    {MPU_DATA, MPU_WRITE,0x95, 0x01, 1,0},

	/* OSC control setting */
	//OSC control 2
    {MPU_DATA, MPU_WRITE,0x1A, 0x00, 1,0},
	//OSC control 3
    {MPU_DATA, MPU_WRITE,0x93, 0x0F, 1,0},
	//Internal Uses 28
    {MPU_DATA, MPU_WRITE,0x70, 0x66, 1,0},
	//Gate Scan control
    {MPU_DATA, MPU_WRITE,0x18, 0x01, 1,0},

	/* r control setting */
	//r Control 1
    {MPU_DATA, MPU_WRITE,0x46, 0x83, 1,0},
	//r Control 2
    {MPU_DATA, MPU_WRITE,0x47, 0x31, 1,0},
	//r Control 3
    {MPU_DATA, MPU_WRITE,0x48, 0x01, 1,0},
	//r Control 4
    {MPU_DATA, MPU_WRITE,0x49, 0x56, 1,0},
	//r Control 5
    {MPU_DATA, MPU_WRITE,0x4A, 0x24, 1,0},
	//r Control 6
    {MPU_DATA, MPU_WRITE,0x4B, 0x05, 1,0},
	//r Control 7
    {MPU_DATA, MPU_WRITE,0x4C, 0x00, 1,0},
	//r Control 8
    {MPU_DATA, MPU_WRITE,0x4D, 0x67, 1,0},
	//r Control 9
    {MPU_DATA, MPU_WRITE,0x4E, 0x01, 1,0},
	//r Control 10
    {MPU_DATA, MPU_WRITE,0x4F, 0x1F, 1,0},
	//r Control 11
    {MPU_DATA, MPU_WRITE,0x50, 0x0F, 1,0},
	//r Control 12
    {MPU_DATA, MPU_WRITE,0x51, 0x0A, 1,0},

	/* RGB interface control setting */
	// RGB interface control 1
    {MPU_DATA, MPU_WRITE,0x38, 0x00, 1,0},
	// RGB interface control 2
    {MPU_DATA, MPU_WRITE,0x39, 0x00, 1,0},

	/* Display control setting */
	// Display control 2
    {MPU_DATA, MPU_WRITE,0x27, 0x02, 1,0},
	// Display control 3
    {MPU_DATA, MPU_WRITE,0x28, 0x03, 1,0},
	// Display control 4
    {MPU_DATA, MPU_WRITE,0x29, 0x08, 1,0},
	// Display control 5
    {MPU_DATA, MPU_WRITE,0x2A, 0x08, 1,0},
	// Display control 6
   // {MPU_DATA, MPU_WRITE,0x2B, 0x08, 1,0},
	// Display control 7
    {MPU_DATA, MPU_WRITE,0x2C, 0x08, 1,0},
	// Display control 8
    {MPU_DATA, MPU_WRITE,0x2D, 0x08, 1,0},
	// Display control 9
    {MPU_DATA, MPU_WRITE,0x35, 0x09, 1,0},
	// Display control 10
    {MPU_DATA, MPU_WRITE,0x36, 0x09, 1,0},
	// Display control 11
    {MPU_DATA, MPU_WRITE,0x91, 0x14, 1,0},
	// Display control 12
    {MPU_DATA, MPU_WRITE,0x37, 0x00, 1,0},
	// Display Mode control
    {MPU_DATA, MPU_WRITE,0x01, 0x02, 1,0},
	// Cycle Control 1
    {MPU_DATA, MPU_WRITE,0x3A, 0xA1, 1,0},
	// Cycle Control 2
    {MPU_DATA, MPU_WRITE,0x3B, 0xA1, 1,0},
	// Cycle Control 3
    {MPU_DATA, MPU_WRITE,0x3C, 0xA0, 1,0},
	// Cycle Control 4
    {MPU_DATA, MPU_WRITE,0x3D, 0x00, 1,0},
	// Cycle Control 5
    {MPU_DATA, MPU_WRITE,0x3E, 0x2D, 1,0},
	// Cycle Control 6
    {MPU_DATA, MPU_WRITE,0x40, 0x03, 1,0},
	// Cycle Control 7
    {MPU_DATA, MPU_WRITE,0x41, 0xCC, 1,0},

	/* Partial Image Display setting */
	// Partial area start row
    {MPU_DATA, MPU_WRITE,0x0A, 0x0000, 2,0},
	// Partial area end row
    {MPU_DATA, MPU_WRITE,0x0C, 0x013F, 2,0}, //319

	/* Vertical Scroll setting */
	// Vertical Scroll Top fixed area
    {MPU_DATA, MPU_WRITE,0x0E, 0x0000, 2,0},
	// Vertical Scroll Top hight area
    {MPU_DATA, MPU_WRITE,0x10, 0x0140, 2,0}, //320
	// Vertical Scroll Button area
    {MPU_DATA, MPU_WRITE,0x12, 0x0000, 2,0},
	// Vertical Scroll Start address
    {MPU_DATA, MPU_WRITE,0x14, 0x0000, 2,0},

	/* Window address setting */
	// Column address start
    {MPU_DATA, MPU_WRITE,0x02, 0x0000, 2,0},
	// Column address end
    {MPU_DATA, MPU_WRITE,0x04, 0x00EF, 2,0}, //239
	// Row address start
    {MPU_DATA, MPU_WRITE,0x06, 0x0000, 2,0},
	// Row address end
    {MPU_DATA, MPU_WRITE,0x08, 0x013F, 2,0}, //319
	// Memory Access control
    {MPU_DATA, MPU_WRITE,0x16, 0x08, 1,0},
	// Data control
    {MPU_DATA, MPU_WRITE,0x72, 0x00, 1,0},

	/* Window adress setting */
    {MPU_CMD, MPU_WRITE,0x22, 0x00, 1,0},

	/* Display on setting */
	// SAP idle mode
    {MPU_DATA, MPU_WRITE,0x94, 0x0A, 1,0},
	// Display Control 8
    {MPU_DATA, MPU_WRITE,0x90, 0x7F, 1,0},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0x84, 1,45},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0xA4, 1,0},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0xAC, 1,45},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0xBC, 1,0},
	// TEST MODE Control
    {MPU_DATA, MPU_WRITE,0x57, 0x00, 1,0},
	// source Driver Control1 100-???-01b  ???=internal gamma bias current, select 100
    {MPU_DATA, MPU_WRITE,0x5C, 0x91, 1,0},

	/* TEST1 setting */
	// TEST1
    {MPU_DATA, MPU_WRITE,0x96, 0x00, 1,0},

	// GRAM Write Data
    {MPU_CMD,  MPU_WRITE,0x22, 0x00, 1,0}
};

int mpu_com24h2p39_lcd_setup(struct mxsfb_info * mxsfb)
{

	unsigned int manufacturer_id;
	unsigned int tmp;
	unsigned int i;
	static int Manufacture_printed=0;

	//printk(KERN_INFO "lcd OM24H2P39ULC driver.\n",);

	if (mxsfb == NULL)
		return -1;
    for (i=0;i<ARRAY_SIZE(startup_sequence);i++)
	{
		if (-1 ==(tmp = mxsfb_mpu_access_rcim(mxsfb,
		                      startup_sequence[i].mode,
		                      startup_sequence[i].rw,
		                      startup_sequence[i].index,
		                      startup_sequence[i].data,
		                      startup_sequence[i].length)))
		{
			pr_err("startup_sequence failed in step =%d\n",i);
			return -1;
	    }
		if ((i==2) && (Manufacture_printed==0)){
			/* Read display ID */
             manufacturer_id=tmp;
			if (manufacturer_id==0x47)
				printk(KERN_INFO "LCD Manufacturer ID = 0x%02x = Himax HX8347-A01\n", manufacturer_id);
			else
				printk(KERN_INFO "LCD Manufacturer ID = 0x%02x.\n", manufacturer_id);
			Manufacture_printed=1;
		}
		if (startup_sequence[i].msleep > 0)
		    msleep(startup_sequence[i].msleep);
	}
   mxsfb_mpu_setup_refresh_data(mxsfb);

	return 0;
}

startup_sequence_t poweroff_sequence[] = 
{
	/* TEST1 setting */
	// TEST1
    {MPU_DATA, MPU_WRITE,0x96, 0x01, 1,0},

	/**/
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0xB8, 1,45},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0xA8, 1,0},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0x84, 1,45},
	// Display Control 1
    {MPU_DATA, MPU_WRITE,0x26, 0x80, 1,0},

	/* Power off setting */
	// Display Control 8
    {MPU_DATA, MPU_WRITE,0x90, 0x00, 1,0},
	// Power Control 2
    {MPU_DATA, MPU_WRITE,0x1C, 0x00, 1,0},
	// Power Control 1
    {MPU_DATA, MPU_WRITE,0x1B, 0x04, 1,0},
	//VCOM Control 1
    {MPU_DATA, MPU_WRITE,0x43, 0x00, 1,0},
	// Power Control 1
    {MPU_DATA, MPU_WRITE,0x1B, 0x0C, 1,0},

	/* TEST1 setting */
	// TEST1
    {MPU_DATA, MPU_WRITE,0x96, 0x00, 1,0},

	/* Power off setting */
	// Power Control 1
    {MPU_DATA, MPU_WRITE,0x1B, 0x0D, 1,0},
	//OSC control 1
    {MPU_DATA, MPU_WRITE,0x19, 0x86, 1,0}
};

int mpu_com24h2p39_lcd_poweroff(struct mxsfb_info * mxsfb)
{
	int i;
	if (mxsfb == NULL)
		return -1;
    for (i=0;i<ARRAY_SIZE(poweroff_sequence);i++)
	{
		if (-1 == mxsfb_mpu_access_rcim(mxsfb,
		                      poweroff_sequence[i].mode,
		                      poweroff_sequence[i].rw,
		                      poweroff_sequence[i].index,
		                      poweroff_sequence[i].data,
		                      poweroff_sequence[i].length))
		{
			pr_err("poweroff_sequence failed in step =%d\n",i);
			return -1;
	    }
		if (startup_sequence[i].msleep > 0)
		    msleep(startup_sequence[i].msleep);
	}
	return 0;
}
