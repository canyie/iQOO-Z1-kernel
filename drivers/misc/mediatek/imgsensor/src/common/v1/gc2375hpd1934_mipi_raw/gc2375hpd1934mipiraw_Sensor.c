/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define PFX "GC2375HPD1934"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
/*#include <asm/atomic.h>*/

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"

#include "gc2375hpd1934mipiraw_Sensor.h"


static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {

	/*record sensor id defined in Kd_imgsensor.h*/
	.sensor_id = GC2375HPD1934_SENSOR_ID,
	.checksum_value = 0xf7375923,        	//checksum value for Camera Auto Test
	.pre = {
		.pclk = 39000000,	/*record different mode's pclk*/
		.linelength = 1040,                	//record different mode's linelength
		.framelength = 1240,               	//record different mode's framelength
		.startx = 0, /*record different mode's startx of grabwindow*/
		.starty = 0,	/*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width  = 1600,

		/*record different mode's height of grabwindow*/
		.grabwindow_height = 1200,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * nby different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 62400000,
		.max_framerate = 300,
		},
	.cap = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 300,
	},
	.normal_video = { /* cap*/
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 300,
		},
	.hs_video = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,

		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 300,
	},
	.slim_video = {/*pre*/
		.pclk = 39000000, /*record different mode's pclk*/
		.linelength = 1040,
		.framelength = 1240,
		.startx = 0, /*record different mode's startx of grabwindow*/
		.starty = 0, /*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width  = 1600,
		/*record different mode's height of grabwindow*/
		.grabwindow_height = 1200,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * by different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 300,
		},
	.custom1 = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1562,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 240,
	},
	.custom2 = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1562,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 240,
	},
	.custom3 = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1562,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 240,
	},
	.custom4 = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1562,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 240,
	},
	.custom5 = {
		.pclk = 39000000,
		.linelength = 1040,
		.framelength = 1562,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 62400000,
		.max_framerate = 240,
	},
	.margin = 0,            				//sensor framelength & shutter margin
	.min_shutter = 1,        				//min shutter

	/*max framelength by sensor register's limitation*/
	.max_frame_length = 0x3fff,				//max framelength by sensor register's limitation

	.ae_shut_delay_frame = 0,    			//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,		//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.frame_time_delay_frame = 2,
	.ae_ispGain_delay_frame = 2,			//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/

	/*support sensor mode num ,don't support Slow motion*/
	.sensor_mode_num = 10,
	.cap_delay_frame = 1,        			//enter capture delay frame num
	.pre_delay_frame = 1,         			//enter preview delay frame num
	.video_delay_frame = 1,        			//enter video delay frame num
	.hs_video_delay_frame = 1,    			//enter high speed video  delay frame num
	.slim_video_delay_frame = 1,			//enter slim video delay frame num
	.custom1_delay_frame = 1,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_2MA, /*mclk driving current*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,	//sensor output first pixel color
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_1_LANE,/*mipi lane num*/

/*record sensor support all write id addr, only supprt 4must end with 0xff*/
	.i2c_addr_table = {0x2e, 0xff},
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3ED,                    	//current shutter
	.gain = 0x40,                        	//current gain
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,

	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x2e, /*record current sensor's i2c write id*/
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200}, // Preview 
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200}, // capture 
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200}, // video 
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},// hight video 120
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},// slim video 
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200}, // custom1
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},  //custom2
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},  //custom3
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},  //custom4
	{ 1600, 1200,	 0,  0, 1600, 1200, 1600,  1200, 0000, 0000, 1600, 1200, 0,    0, 1600,  1200},  //custom5
};

/****vivo caofeixiang add for CameraEM otp errorcode****/
extern int vivo_main2_otp_read(void);
int vivo_main2_otp_read_when_power_on;
extern otp_error_code_t MAIN2_OTP_ERROR_CODE_GC2375HPD1934;
MUINT32  sn_inf_main2_gc2375hpd1934[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main2_gc2375hpd1934[4];
/****vivo caofeixiang add end****/
static kal_uint16 read_cmos_sensor_8_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 2
#endif

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 2 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								2, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 2, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;

#endif
	}
	return 0;
}




static void set_dummy(void)
{
	pr_debug("dummy_line %d", imgsensor.dummy_line);
	write_cmos_sensor_8_8(0x07, imgsensor.dummy_line >> 8);
	write_cmos_sensor_8_8(0x08, imgsensor.dummy_line & 0xFF);
}/*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_8_8(0xf0) << 8) | read_cmos_sensor_8_8(0xf1));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution*/
/* if shutter bigger than frame_length, should extend frame length first*/

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	// Update Shutter
	if (shutter == (imgsensor.frame_length-1))
		shutter += 1;

	if(shutter > 16383) shutter = 16383;
	if(shutter < 1) shutter = 1;

	//Update Shutter
	write_cmos_sensor_8_8(0xfe, 0x00);	
	write_cmos_sensor_8_8(0x03, (shutter>>8) & 0x3F);
	write_cmos_sensor_8_8(0x04, shutter & 0xFF);

	pr_debug("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */

// The value is from experience value
#define ACTIVE_LINES 1224
// This function is for dual camera exposure and frame sync
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.dummy_line = (frame_length > ACTIVE_LINES) ? (frame_length - ACTIVE_LINES) : 0;
	imgsensor.frame_length = ACTIVE_LINES + imgsensor.dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("frame_length: %d, shutter: %d, dummy_line: %d", imgsensor.frame_length, shutter, imgsensor.dummy_line);

	set_dummy();

	set_shutter(shutter);
} /* set_shutter_frame_length */

/*
static kal_uint16 gain2reg(const kal_uint16 gain)
{
kal_uint16 reg_gain = 0x0000;

reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
reg_gain = reg_gain & 0xFFFF;
return (kal_uint16)reg_gain;
}*/

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

#define ANALOG_GAIN_1 64   // 1.00x
#define ANALOG_GAIN_2 92   // 1.43x
#define ANALOG_GAIN_3 128  // 2.00x
#define ANALOG_GAIN_4 182  // 2.84x
#define ANALOG_GAIN_5 254  // 3.97x
#define ANALOG_GAIN_6 363  // 5.68x
#define ANALOG_GAIN_7 521  // 8.14x
#define ANALOG_GAIN_8 725  // 11.34x
#define ANALOG_GAIN_9 1038 // 16.23x

static kal_uint16 set_gain(kal_uint16 gain)
{	
	kal_uint16 iReg,temp; 	

	iReg = gain;

	if(iReg < 0x40)
		iReg = 0x40;
	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		write_cmos_sensor_8_8(0x20,0x0b);
		write_cmos_sensor_8_8(0x22,0x0c);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x00);
		temp = iReg;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 1x, GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x01);
		temp = 64*iReg/ANALOG_GAIN_2;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 1.43x , GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x02);
		temp = 64*iReg/ANALOG_GAIN_3;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 2x , GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x03);
		temp = 64*iReg/ANALOG_GAIN_4;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 2.84x , GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_5<= iReg)&&(iReg < ANALOG_GAIN_6))
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x04);
		temp = 64*iReg/ANALOG_GAIN_5;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 3.97x , GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
		write_cmos_sensor_8_8(0x20,0x0e);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x05);
		temp = 64*iReg/ANALOG_GAIN_6;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 5.68x , GC2375MIPI add pregain = %d\n",temp);
	}	
	else if((ANALOG_GAIN_7<= iReg)&&(iReg < ANALOG_GAIN_8))
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0c);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x06);
		temp = 64*iReg/ANALOG_GAIN_7;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 8.14x , GC2375MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_8<= iReg)&&(iReg < ANALOG_GAIN_9))
	{
		write_cmos_sensor_8_8(0x20,0x0e);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x07);
		temp = 64*iReg/ANALOG_GAIN_8;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 11.34x , GC2375MIPI add pregain = %d\n",temp);
	}	
	else 
	{
		write_cmos_sensor_8_8(0x20,0x0c);
		write_cmos_sensor_8_8(0x22,0x0e);
		write_cmos_sensor_8_8(0x26,0x0e);		
		//analog gain
		write_cmos_sensor_8_8(0xb6,0x08);
		temp = 64*iReg/ANALOG_GAIN_9;
		write_cmos_sensor_8_8(0xb1,temp>>6);
		write_cmos_sensor_8_8(0xb2,(temp<<2)&0xfc);
		pr_debug("GC2375MIPI analogic gain 16.23x , GC2375MIPI add pregain = %d\n",temp);
	}
	
	return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	pr_debug("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}

/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
pr_debug("image_mirror = %d\n", image_mirror);

}
*/
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}				/*    night_mode    */

static kal_uint16 addr_data_pair_init[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x00,
	0x08,0x10,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00,
};

static kal_uint16 addr_data_pair_preview[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                                                         
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x00,
	0x08,0x10,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,                                         
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	                                  
	0x68,0x00,	                                 
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,                                          
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00,  
};

static kal_uint16 addr_data_pair_capture[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x00,
	0x08,0x10,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
		
};

static kal_uint16 addr_data_pair_custom1[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x01,
	0x08,0x52,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
};


static kal_uint16 addr_data_pair_custom2[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x01,
	0x08,0x52,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
};

static kal_uint16 addr_data_pair_custom3[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x01,
	0x08,0x52,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
};

static kal_uint16 addr_data_pair_custom4[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x01,
	0x08,0x52,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
};

static kal_uint16 addr_data_pair_custom5[] = {
	0xfe,0x00,
	0xfe,0x00,
	0xfe,0x00,
	0xf7,0x01,
	0xf8,0x0c,
	0xf9,0x42,
	0xfa,0x88,
	0xfc,0x8e,
	0xfe,0x00,
	0x88,0x03,                      
	0x03,0x04,
	0x04,0x65,
	0x05,0x02,
	0x06,0x5a,
	0x07,0x01,
	0x08,0x52,
	0x09,0x00,
	0x0a,0x08, 
	0x0b,0x00,
	0x0c,0x18,
	0x0d,0x04,
	0x0e,0xb8,
	0x0f,0x06,
	0x10,0x48,
	0x17,0xd7,
	0x1c,0x10,
	0x1d,0x13,
	0x20,0x0b,
	0x21,0x6d,	
	0x22,0x0c,
	0x25,0xc1,
	0x26,0x0e,
	0x27,0x22,
	0x29,0x5f,
	0x2b,0x88,
	0x2f,0x12,
	0x38,0x86,
	0x3d,0x00,
	0xcd,0xa3,
	0xce,0x57,
	0xd0,0x09,
	0xd1,0xca,
	0xd2,0x34,
	0xd3,0xbb,
	0xd8,0x60,
	0xe0,0x08,
	0xe1,0x1f,
	0xe4,0xf8,
	0xe5,0x0c,
	0xe6,0x10,
	0xe7,0xcc,
	0xe8,0x02,
	0xe9,0x01,
	0xea,0x02,
	0xeb,0x01,
	0x90,0x01, 
	0x92,0x02,
	0x94,0x00,
	0x95,0x04,
	0x96,0xb0,
	0x97,0x06,
	0x98,0x40, 
	0x18,0x02,
	0x1a,0x18,
	0x28,0x00,
	0x3f,0x40,
	0x40,0x26,
	0x41,0x00,
	0x43,0x03,
	0x4a,0x00,	
	0x4e,0x3c,
	0x4f,0x00,
	0x66,0xc0,
	0x67,0x00,	
	0x68,0x00,	
	0xb0,0x58,
	0xb1,0x01,
	0xb2,0x00,	
	0xb6,0x00,
	//0xef,0x90,
	0xfe,0x03,
	0x01,0x03,
	0x02,0x33,
	0x03,0x90,
	0x04,0x04,
	0x05,0x00,
	0x06,0x80,
	0x11,0x2b,
	0x12,0xd0,
	0x13,0x07,
	0x42,0x40, 
	0x43,0x06,
	0x15,0x00,
	0x21,0x08,
	0x22,0x05,
	0x23,0x13,
	0x24,0x02,
	0x25,0x13,
	0x26,0x08,
	0x29,0x06,
	0x2a,0x08,
	0x2b,0x08, 
	0xfe,0x00, 	
};
static void sensor_init(void)
{
	pr_debug("v2 E\n");
	table_write_cmos_sensor(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
}	/*	sensor_init  */

/*************************************************************************
 * FUNCTION
 *	preview_setting
 *
 * DESCRIPTION
 *	Sensor preview
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void preview_setting(void)
{
	
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	
}	/*	preview_setting  */
/*************************************************************************
 * FUNCTION
 *	Capture
 *
 * DESCRIPTION
 *	Sensor capture
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void capture_setting(kal_uint16 currefps)
{
	pr_debug("E! currefps:%d\n", currefps);
	
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
}

#if 0
static void normal_video_setting(kal_uint16 currefps)
{
}
#endif
static void hs_video_setting(void)
{
	pr_debug("E\n");
}

static void slim_video_setting(void)
{
	pr_debug("E\n");
}


static void custom1_setting(kal_uint16 currefps)
{
	pr_debug("custom1_setting E\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	pr_debug("custom1_setting() end! \n");
}

static void custom2_setting(kal_uint16 currefps)
{
	pr_debug("custom2_setting E\n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	pr_debug("custom2_setting() end! \n");
}

static void custom3_setting(kal_uint16 currefps)
{
	pr_debug("custom3_setting E\n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	pr_debug("custom3_setting() end! \n");
}

static void custom4_setting(kal_uint16 currefps)
{
	pr_debug("custom4_setting E\n");
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	pr_debug("custom4_setting() end! \n");
}

static void custom5_setting(kal_uint16 currefps)
{
	pr_debug("custom5_setting E\n");
	table_write_cmos_sensor(addr_data_pair_custom5,
		   sizeof(addr_data_pair_custom5) / sizeof(kal_uint16));
	pr_debug("custom5_setting() end! \n");
}
static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor_8_8(0xfe,0x00);
		write_cmos_sensor_8_8(0x8c, 0x11);		
	} else {
		write_cmos_sensor_8_8(0xfe,0x00);
		write_cmos_sensor_8_8(0x8c, 0x10);		
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	int I2C_BUS = -1;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	pr_debug("gc2375hpd1934mipiraw_Sensor I2C_BUS = %d\n", I2C_BUS);	
	if(I2C_BUS != 4){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id()+1;
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  

					/*vivo caofeixiang add for CameraEM otp errorcode*/
					pr_debug("cfx_add:start read eeprom ---vivo_main2_otp_read_when_power_on = %d\n", vivo_main2_otp_read_when_power_on);
					vivo_main2_otp_read_when_power_on = vivo_main2_otp_read();
					pr_debug("cfx_add:end read eeprom ---vivo_main2_otp_read_when_power_on = %d,MAIN2_OTP_ERROR_CODE_GC2375HPD1934 =%d\n", vivo_main2_otp_read_when_power_on,MAIN2_OTP_ERROR_CODE_GC2375HPD1934);
					/*vivo caofeixiang add end*/
					return ERROR_NONE;
			}
			pr_debug("Read sensor id fail,write_id:0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
/*const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};*/
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	pr_debug("PLATFORM:Vison,MIPI 4LANE\n");
	pr_debug("read_cmos_sensor_8_8(0x302A): 0x%x\n", read_cmos_sensor_8_8(0x302A));
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id()+1;
			if (sensor_id == imgsensor_info.sensor_id) {

				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			pr_debug(
			    "Read sensor id fail, write: 0x%x, sensor: 0x%x\n",
			    imgsensor.i2c_write_id, sensor_id);

			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	sensor_init();
	/*mdelay(10);*/
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	pr_debug("E\n");

	/*No Need to implement this function*/
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/*imgsensor.video_mode = KAL_FALSE;*/
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	/*set_mirror_flip(imgsensor.mirror);*/
	/*mdelay(10);*/
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;



		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",

		imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	/*mdelay(10);*/
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	/*normal_video_setting(imgsensor.current_fps);*/
	/*set_mirror_flip(imgsensor.mirror);*/
	/*mdelay(10);*/
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/
	/*mdelay(10);*/

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/
	/*mdelay(10);*/

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom1's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom1.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom2.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom2's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom2.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom3.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom3's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom3.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom3.pclk;
		imgsensor.line_length = imgsensor_info.custom3.linelength;
		imgsensor.frame_length = imgsensor_info.custom3.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom3_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom4.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom4's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom4.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom4.pclk;
		imgsensor.line_length = imgsensor_info.custom4.linelength;
		imgsensor.frame_length = imgsensor_info.custom4.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom4_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}
static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom5.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom5's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom5.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom5.pclk;
		imgsensor.line_length = imgsensor_info.custom5.linelength;
		imgsensor.frame_length = imgsensor_info.custom5.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom5_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}

static kal_uint32 get_resolution(
			MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
	pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	=
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
		
		
	sensor_resolution->SensorCustom1Width = 
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
		
	sensor_resolution->SensorCustom2Width = 
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;
	
	sensor_resolution->SensorCustom3Width = 
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;
		
   sensor_resolution->SensorCustom4Width = 
		imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;
		
   sensor_resolution->SensorCustom5Width = 
		imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =
		imgsensor_info.custom5.grabwindow_height;
		
		
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				  MSDK_SENSOR_INFO_STRUCT *sensor_info,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */
	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX =
				imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX =
				imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
			break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			sensor_info->SensorGrabStartX =
				imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.normal_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			  imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
			break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX =
				imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
			break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX =
				imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
			break;
    case MSDK_SCENARIO_ID_CUSTOM1:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;

			break;

	default:
			sensor_info->SensorGrabStartX =
				imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY =
				imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	return ERROR_NONE;
}	/*	get_info  */

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8_8(0xef, 0X90);
	else
		write_cmos_sensor_8_8(0xef, 0x00);
	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
    case MSDK_SCENARIO_ID_CUSTOM1:
			custom1(image_window, sensor_config_data);
			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
			custom2(image_window, sensor_config_data);
			break;
	 case MSDK_SCENARIO_ID_CUSTOM3:
			custom3(image_window, sensor_config_data);
			break;
	 case MSDK_SCENARIO_ID_CUSTOM4:
			custom4(image_window, sensor_config_data);
			break;
	 case MSDK_SCENARIO_ID_CUSTOM5:
			custom5(image_window, sensor_config_data);
			break;
	default:
			pr_debug("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0)
		/* Dynamic frame rate*/
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	pr_debug("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker	  */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
		    / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);
		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		  imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;

		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);

		if (frame_length
			   > imgsensor_info.normal_video.framelength)
			imgsensor.dummy_line =
			 frame_length - imgsensor_info.normal_video.framelength;

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk
		    / framerate * 10 / imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.cap.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.cap.framelength);
		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		  imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
			imgsensor_info.hs_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.hs_video.framelength)
			imgsensor.dummy_line =
		(frame_length - imgsensor_info.hs_video.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
	    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
			imgsensor_info.slim_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);

		if (frame_length > imgsensor_info.slim_video.framelength)
			imgsensor.dummy_line =
		(frame_length - imgsensor_info.slim_video.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk /
			framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom1.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom1.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom1.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk /
			framerate * 10 / imgsensor_info.custom2.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom2.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom2.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom2.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
		case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk /
			framerate * 10 / imgsensor_info.custom3.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom3.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom3.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom3.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk /
			framerate * 10 / imgsensor_info.custom4.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom4.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom4.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom4.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk /
			framerate * 10 / imgsensor_info.custom5.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom5.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom5.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom5.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk
		    / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);
		else
			imgsensor.dummy_line = 0;
		imgsensor.frame_length =
		  imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		pr_debug(
		"error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		    enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
			*framerate = imgsensor_info.custom1.max_framerate;	
			break;
	case MSDK_SCENARIO_ID_CUSTOM2:
			*framerate = imgsensor_info.custom2.max_framerate;	
			break;
	case MSDK_SCENARIO_ID_CUSTOM3:
			*framerate = imgsensor_info.custom3.max_framerate;	
			break;
	case MSDK_SCENARIO_ID_CUSTOM4:
			*framerate = imgsensor_info.custom4.max_framerate;	
			break;
	case MSDK_SCENARIO_ID_CUSTOM5:
			*framerate = imgsensor_info.custom5.max_framerate;	
			break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
				  (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	pr_debug("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_8_8(sensor_reg_data->RegAddr);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;

	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(
			(BOOL)*feature_data_16, *(feature_data_16+1));
		break;

	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
		  (enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;

	/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);

		wininfo =
	    (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[9],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));

		ihdr_write_shutter_gain(
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(
		    (UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
		break;

	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		pr_debug("This sensor can't support temperature get\n");
		break;
		
        case SENSOR_FEATURE_GET_CUSTOM_INFO:
		pr_debug("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN2_OTP_ERROR_CODE_GC2375HPD1934:%d \n", *feature_data,MAIN2_OTP_ERROR_CODE_GC2375HPD1934);
            switch (*feature_data) {
                case 0:    //info type: otp state
				pr_debug("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
				if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
				    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = MAIN2_OTP_ERROR_CODE_GC2375HPD1934;//otp_state
					memcpy( feature_data+2, sn_inf_main2_gc2375hpd1934, sizeof(MUINT32)*13); 
					memcpy( feature_data+10, material_inf_main2_gc2375hpd1934, sizeof(MUINT32)*4);
					#if 0
							for (i = 0 ; i<12 ; i++ ){
							printk("sn_inf_main2_hi846pd1934[%d]= 0x%x\n", i, sn_inf_main2_gc2375hpd1934[i]);
							}
						
					#endif
					}
				break;
			}
			break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);

		if (*feature_data != 0)
			set_shutter(*feature_data);

		streaming_control(KAL_TRUE);
	break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
        {
               kal_uint32 rate;
 
               switch (*feature_data) {
               case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                       rate = imgsensor_info.cap.mipi_pixel_rate;
                       break;
               case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                       rate = imgsensor_info.normal_video.mipi_pixel_rate;
                       break;
               case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                       rate = imgsensor_info.hs_video.mipi_pixel_rate;
                       break;
               case MSDK_SCENARIO_ID_SLIM_VIDEO:
                       rate = imgsensor_info.slim_video.mipi_pixel_rate;
                       break;
               case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                       rate = imgsensor_info.pre.mipi_pixel_rate;
                       break;
   			   case MSDK_SCENARIO_ID_CUSTOM1:
						rate = imgsensor_info.custom1.mipi_pixel_rate;
						break;
	    		case MSDK_SCENARIO_ID_CUSTOM2:
						rate = imgsensor_info.custom2.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM3:
						rate = imgsensor_info.custom3.mipi_pixel_rate;
						break;	
				case MSDK_SCENARIO_ID_CUSTOM4:
						rate = imgsensor_info.custom4.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM5:
						rate = imgsensor_info.custom5.mipi_pixel_rate;
						break;					
               default:
                       rate = 0;
                       break;
               }
               *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
        }
        break;

	default:
		break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 GC2375HPD1934_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}	/*	GC2375HPD1934_MIPI_RAW_SensorInit	*/
