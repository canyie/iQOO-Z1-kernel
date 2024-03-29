/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * OVT setting release hitory: 
 * 20200602: Setting is based on OV64C AM07 released.
 * 20200610: MirrorFlip enable.
 * 202006/17: PDAF PD location
 * ***************************************************************************
 *
 * Filename:
 * ---------
 *	 ov32alqmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/


#define PFX "ov32alq_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "ov32alqmipiraw_Sensor.h"
#include "ov32alqmipiraw_freq.h"
#include "../imgsensor_sensor.h"

#include "../imgsensor_i2c.h" //vivo lishuo add MTK patch for [H202012312161] 20210218

/*
 * #define printk(format, args...) pr_debug(
 * PFX "[%s] " format, __func__, ##args)
 */

static DEFINE_SPINLOCK(imgsensor_drv_lock);
//static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV32ALQ_SENSOR_ID,
	.checksum_value = 0x67b95889,

	.pre = {
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
	},
	.cap = {
		.pclk =97500000,
		.linelength = 1320,
		.framelength = 5120,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 6528,
		.grabwindow_height = 4896,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 542000000,
		
		.max_framerate = 150,

	},
	.normal_video = {
		.pclk = 97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1840,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334530000,
		.max_framerate = 300,
	},
	.hs_video = { /*hs_video 120fps*/
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
		},
	.slim_video = { /* hs_video 240fps*/
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
		},
	.custom1 = { /*cpy from preview*/
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
		},
	.custom2 = { /*cpy from capture*/
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
		},
	.custom3 = { /* 4608*2592@60fps*/
		.pclk =97500000,
		.linelength = 450,
		.framelength = 3611,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 600,
		},
	.custom4 = { /* copy from preview*/
		.pclk =97500000,
		.linelength = 555,
		.framelength = 5854,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
	},
	.custom5 = { /* copy from preview*/
		.pclk =97500000,
		.linelength = 1080,
		.framelength = 3008,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 334000000,
		
		.max_framerate = 300,
	},
   
		.margin = 14,
		.min_shutter = 8,
		.min_gain = 64,
		.max_gain = 992,
		.min_gain_iso = 100,
		.gain_step = 2,
		.gain_type = 2,
		.max_frame_length = 0xffff, // Max VTS=0xffff.
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.frame_time_delay_frame = 2,	/* The delay frame of setting frame length  *//*vivo xuyongfu change for bug B201019-3890*/
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/
		.sensor_mode_num = 10,	  /*support sensor mode num*/

		.cap_delay_frame = 2,/*3 guanjd modify for cts*/
		.pre_delay_frame = 2,/*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 2,
		.slim_video_delay_frame = 2,
		.custom1_delay_frame = 2,
		.custom2_delay_frame = 2,
		.custom3_delay_frame = 2,
        .custom4_delay_frame = 2,
		.custom5_delay_frame = 2,
		.isp_driving_current = ISP_DRIVING_4MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 0, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		//.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,

        .mclk = 26, /* 6MHz mclk for 26fps */
	    .mipi_lane_num = SENSOR_MIPI_4_LANE,


		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 1000,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0xB00,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 300,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,
	.freq_setting = 0,
  	.present_freq_setting = 0,
	
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {  
  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,   0,    0, 3264, 2448}, /*Preview*/ 
  { 6528, 4896,    0,    0, 6528, 4896, 6528, 4896, 0,    0, 6528, 4896,   0,    0, 6528, 4896}, /*capture*/

  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    304, 3264, 1840,  0,    0, 3264, 1840}, /*video*/

  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448},	 /*hight speed video 120fps*/
\
  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448}, /*slim video  */

  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448}, /*custom1*/

  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448}, /*custom2*/

  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448}, /*custom3*/

  { 6528, 4896,    0,    0, 6528, 4896, 1632, 1224, 0,    0, 1632, 1224,    0,    0, 1632, 1224}, /*custom4*/
  { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448, 0,    0, 3264, 2448,    0,    0, 3264, 2448}, /*custom5*/
};


/*no mirror flip*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = 
{
    .i4OffsetX = 32,
    .i4OffsetY = 8,
    .i4PitchX = 16,
    .i4PitchY = 16,
    .i4PairNum = 8,
    .i4SubBlkW = 8,
    .i4SubBlkH = 4,
    .i4PosL = {{39,10},{47,10},{35,14},{43,14},{39,18},{47,18},{35,22},{43,22}},
    .i4PosR = {{38,10},{46,10},{34,14},{42,14},{38,18},{46,18},{34,22},{42,22}},
    .i4BlockNumX = 284,
    .i4BlockNumY = 215,
    .i4LeFirst = 0,
    .i4Crop = {
        {0, 0}, {0, 0}, {0, 376}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
    },
	.iMirrorFlip = 3,
};





/*VC1 None , VC2 for PDAF(DT=0X30), unit : 10bit*/
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
	/* Preview mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1200, 0x0D80, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x2b, 0x0470, 0x035c, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Capture mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x2400, 0x1B00, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x2b, 0x0470, 0x035c, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Video mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1200, 0x0A20, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x2b, 0x0470, 0x0288, 0x03, 0x00, 0x0000, 0x0000
	},	
};

/*hope add otp check start*/
static int vivo_otp_read_when_power_on;
extern int ov32alq_otp_read(void);
extern otp_error_code_t OV32ALQ_OTP_ERROR_CODE;
MUINT32  sn_inf_main_ov32alq[13];  /*0 flag   1-12 data*/
MUINT32  crosstalk_main_ov32alq[299]; 
MUINT32  material_inf_main_ov32alq[4];  
/*hope add otp check end*/



static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	 /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
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
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}

#if MULTI_WRITE
	if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 3, imgsensor_info.i2c_speed);
			tosend = 0;
	}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;
#endif

	}
	return 0;
}



static void set_dummy(void)
{

	PK_DBG("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	 
    write_cmos_sensor_16_8(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor_16_8(0x380d, imgsensor.line_length & 0xFF);
    write_cmos_sensor_16_8(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
    write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);	 

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	PK_DBG("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
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

static kal_uint32 streaming_control(kal_bool enable)
{

	PK_DBG("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0X01);
		mdelay(10);
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		mdelay(10);
		}

	return ERROR_NONE;
}


static void write_shutter(kal_uint32 shutter)
{

	kal_uint16 realtime_fps = 0;
	//int framecnt = 0;
	PK_DBG("===hope\n");
			
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	
	if (shutter < imgsensor_info.min_shutter) 
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		/* Extend frame length*/
		/*
		  	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor_16_8(0x3840, imgsensor.frame_length >> 16);
			write_cmos_sensor_16_8(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
			write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
		*/
	    }
	} else {
		/* Extend frame length*/
		/*
		  	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor_16_8(0x3840, imgsensor.frame_length >> 16);
			write_cmos_sensor_16_8(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
			write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
		*/
	}
	/* Update frame length & Shutter*/
	
	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
	write_cmos_sensor_16_8(0x3208, 0x00);
	write_cmos_sensor_16_8(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
	write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_16_8(0x3500,(shutter >> 16)); 
	write_cmos_sensor_16_8(0x3501,(shutter >> 8)&0xFF); 
	write_cmos_sensor_16_8(0x3502,(shutter & 0xFF));
	write_cmos_sensor_16_8(0x3208, 0x10);
	write_cmos_sensor_16_8(0x3208, 0xa0);
	PK_DBG("shutter = %d, framelength = %d\n", shutter,imgsensor.frame_length);
	
}	/*	write_shutter  */



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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			/*write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);*/
		}
	} else {
			/* Extend frame length */
			/*write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);*/
	}

	/* Update Shutter */

	write_cmos_sensor_16_8(0x3208, 0x00);
	write_cmos_sensor_16_8(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
	write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_16_8(0x3500,(shutter >> 16)); 
	write_cmos_sensor_16_8(0x3501,(shutter >> 8)&0xFF); 
	write_cmos_sensor_16_8(0x3502,(shutter & 0xFF));
	write_cmos_sensor_16_8(0x3208, 0x10);
	write_cmos_sensor_16_8(0x3208, 0xa0);	

	PK_DBG("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);

}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	reg_gain = gain*256/BASEGAIN;

	if(reg_gain < 0x100)// sensor 1xGain
	{
		reg_gain = 0X100;
	}
	if(reg_gain > 0xf80)// sensor 15.5xGain
	{
		reg_gain = 0Xf80;
	}

	 return (kal_uint16)reg_gain;	
}

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
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/*gain= 1024;for test*/
	/*return; for test*/

	if (gain < BASEGAIN || gain > (15* BASEGAIN + BASEGAIN/2)) {
	    PK_DBG("Error gain setting");

	    if (gain < BASEGAIN)
	        gain = BASEGAIN;
	    else if (gain > (15* BASEGAIN + BASEGAIN/2))
	        gain = (15* BASEGAIN + BASEGAIN/2);
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	PK_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_8(0x3508,(reg_gain>>8));
	write_cmos_sensor_16_8(0x3509,(reg_gain&0xff));

	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {

	    case IMAGE_NORMAL:
	       
	        break;

	    case IMAGE_H_MIRROR:
	     
	        break;

	    case IMAGE_V_MIRROR:       
	        break;

	    case IMAGE_HV_MIRROR:

	        break;
	    default:
	    PK_DBG("Error image_mirror setting\n");
	}
}

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
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif



static kal_uint16 addr_data_pair_init[] = { 
0x0103,0x01,
0x0102,0x01,
0x0302,0x31,
0x0323,0x05,
0x0324,0x01,
0x0325,0x68,
0x0326,0xcb,
0x0327,0x05,
0x0343,0x05,
0x0346,0xcf,
0x300e,0x22,
0x3016,0x96,
0x3017,0x78,
0x3018,0x70,
0x3019,0xd2,
0x301b,0x16,
0x3022,0xd0,
0x3028,0xc3,
0x3102,0x00,
0x3103,0x0a,
0x3221,0x0a,
0x3400,0x04,
0x3408,0x06,
0x3508,0x0f,
0x3548,0x08,
0x360b,0xfc,
0x360c,0x11,
0x3619,0x80,
0x361c,0x80,
0x361e,0x87,
0x3623,0x55,
0x3626,0x89,
0x3627,0x11,
0x3628,0x88,
0x3629,0xce,
0x3634,0x0c,
0x363b,0x09,
0x363c,0x14,
0x363d,0x24,
0x363e,0x43,
0x3641,0x0a,
0x3642,0xc8,
0x3654,0x0a,
0x3655,0xdc,
0x3656,0x0f,
0x3663,0x81,
0x3664,0x00,
0x3681,0x08,
0x3683,0x15,
0x3684,0x40,
0x3685,0x10,
0x3686,0x11,
0x368b,0x06,
0x3704,0x22,
0x3706,0x4f,
0x3709,0x8e,
0x3711,0x00,
0x3724,0x40,
0x373f,0x02,
0x374f,0x05,
0x3765,0x08,
0x3767,0x00,
0x37cb,0x11,
0x37cc,0x0f,
0x37d9,0x08,
0x37dc,0x20,
0x37e3,0x18,
0x3823,0x04,
0x382a,0x01,
0x3834,0x04,
0x383d,0x00,
0x3860,0x00,
0x3861,0x00,
0x3889,0x03,
0x388b,0x02,
0x388d,0x01,
0x3d85,0x85,
0x3d8c,0x77,
0x3d8d,0xa0,
0x3d96,0x0a,
0x3f01,0x12,
0x4009,0x02,
0x4021,0x00,
0x4023,0x00,
0x4024,0x04,
0x4025,0x00,
0x4026,0x04,
0x4027,0x00,
0x40c3,0x0a,
0x4506,0x0a,
0x460a,0x0a,
0x464a,0x0a,
0x4850,0x41,
0x4a00,0x10,
0x4d01,0x00,
0x4d02,0xb7,
0x4d03,0xca,
0x4d04,0x30,
0x4d05,0x1d,
0x5004,0x02,
0x5181,0x10,
0x5182,0x05,
0x5183,0x8f,
0x5184,0x01,
0x522a,0x44,
0x522b,0x44,
0x522c,0x14,
0x522d,0x44,
0x5300,0x7b,
0x5311,0x01,
0x5312,0x01,
0x5313,0x02,
0x5314,0x04,
0x5315,0x06,
0x5316,0x08,
0x5317,0x0a,
0x5318,0x0c,
0x5319,0x0e,
0x531a,0x10,
0x531b,0x12,
0x531c,0x14,
0x531d,0x16,
0x531e,0x18,
0x5330,0x12,
0x5331,0x15,
0x5332,0x17,
0x5333,0x19,
0x5334,0x1b,
0x5335,0x1d,
0x5336,0x1e,
0x5337,0x1f,
0x5338,0x20,
0x5339,0x20,
0x533a,0x0c,
0x533b,0x02,
0x533c,0x68,
0x5d80,0x21,
0x5d82,0x04,
0x5d85,0x19,
0x5e07,0x0a,
0x600b,0x03,
0x53C0,0x67,
0x53C1,0x7A,
0x53C2,0x73,
0x53C3,0x75,
0x53C4,0x89,
0x53C5,0x8F,
0x53C6,0x6D,
0x53C7,0x63,
0x53C8,0x7E,
0x53C9,0x74,
0x53CA,0x89,
0x53CB,0x8D,
0x53CC,0x6C,
0x53CD,0x62,
0x53CE,0x7F,
0x53CF,0x8B,
0x53D0,0x81,
0x53D1,0x84,
0x53D2,0x61,
0x53D3,0x7A,
0x53D4,0x8A,
0x53D5,0x81,
0x53D6,0x99,
0x53D7,0x99,
0x53D8,0x7F,
0x53D9,0x71,
0x53DA,0x83,
0x53DB,0x9B,
0x53DC,0x9D,
0x53DD,0x9C,
0x53DE,0x76,
0x53DF,0x8B,
0x53E0,0x81,
0x53E1,0x9A,
0x53E2,0x9D,
0x53E3,0x9F,
0x53E4,0x89,
0x53E5,0x75,
0x53E6,0x76,
0x53E7,0x7D,
0x53E8,0x61,
0x53E9,0x6E,
0x53EA,0x8F,
0x53EB,0x75,
0x53EC,0x71,
0x53ED,0x78,
0x53EE,0x6F,
0x53EF,0x6B,
0x53F0,0x86,
0x53F1,0x8D,
0x53F2,0x74,
0x53F3,0x78,
0x53F4,0x6F,
0x53F5,0x6B,
0x53F6,0x9B,
0x53F7,0x85,
0x53F8,0x8D,
0x53F9,0x76,
0x53FA,0x66,
0x53FB,0x6C,
0x53FC,0x98,
0x53FD,0x9A,
0x53FE,0x86,
0x53FF,0x8E,
0x5400,0x7F,
0x5401,0x64,
0x5402,0x85,
0x5403,0x84,
0x5404,0x81,
0x5405,0x8C,
0x5406,0x71,
0x5407,0x7F,
0x5408,0x70,
0x5409,0x8A,
0x540A,0x87,
0x540B,0x99,
0x540C,0x93,
0x540D,0x93,
0x540E,0x7F,
0x540F,0x71,
0x5410,0x80,
0x5411,0x98,
0x5412,0x93,
0x5413,0x93,
0x5414,0x66,
0x5415,0x7A,
0x5416,0x8B,
0x5417,0x87,
0x5418,0x9C,
0x5419,0x9D,
0x541A,0x6C,
0x541B,0x62,
0x541C,0x7C,
0x541D,0x89,
0x541E,0x84,
0x541F,0x9B,
0x5420,0x6C,
0x5421,0x63,
0x5422,0x79,
0x5423,0x74,
0x5424,0x8F,
0x5425,0x83,
0x5426,0x67,
0x5427,0x65,
0x5428,0x72,
0x5429,0x8A,
0x542A,0x8E,
0x542B,0x8D,
0x542C,0x9B,
0x542D,0x9B,
0x542E,0x84,
0x542F,0x80,
0x5430,0x74,
0x5431,0x72,
0x5432,0x9F,
0x5433,0x99,
0x5434,0x84,
0x5435,0x8D,
0x5436,0x72,
0x5437,0x78,
0x5438,0x9E,
0x5439,0x9B,
0x543A,0x83,
0x543B,0x75,
0x543C,0x65,
0x543D,0x63,
0x543E,0x84,
0x543F,0x83,
0x5440,0x8A,
0x5441,0x7E,
0x5442,0x6D,
0x5443,0x6E,
0x5444,0x8F,
0x5445,0x8B,
0x5446,0x71,
0x5447,0x78,
0x5448,0x6F,
0x5449,0x6B,
0x544A,0x89,
0x544B,0x8A,
0x544C,0x76,
0x544D,0x7C,
0x544E,0x60,
0x544F,0x6F,
0x5450,0x65,
0x5451,0x63,
0x5452,0x6A,
0x5453,0x6B,
0x5454,0x61,
0x5455,0x79,
0x5456,0x78,
0x5457,0x64,
0x5458,0x6F,
0x5459,0x63,
0x545A,0x79,
0x545B,0x71,
0x545C,0x7D,
0x545D,0x72,
0x545E,0x7E,
0x545F,0x72,
0x5460,0x8A,
0x5461,0x8F,
0x5462,0x74,
0x5463,0x88,
0x5464,0x88,
0x5465,0x8F,
0x5466,0x83,
0x5467,0x81,
0x5468,0x82,
0x5469,0x80,
0x546A,0x81,
0x546B,0x85,
0x546C,0x99,
0x546D,0x98,
0x546E,0x86,
0x546F,0x87,
0x5470,0x85,
0x5471,0x98,
0x5472,0x9F,
0x5473,0x99,
0x5474,0x7E,
0x5475,0x65,
0x5476,0x6F,
0x5477,0x6B,
0x5478,0x6C,
0x5479,0x60,
0x547A,0x70,
0x547B,0x7F,
0x547C,0x66,
0x547D,0x6C,
0x547E,0x66,
0x547F,0x65,
0x5480,0x8F,
0x5481,0x88,
0x5482,0x71,
0x5483,0x7F,
0x5484,0x7F,
0x5485,0x7C,
0x5486,0x80,
0x5487,0x80,
0x5488,0x8D,
0x5489,0x89,
0x548A,0x75,
0x548B,0x76,
0x548C,0x85,
0x548D,0x98,
0x548E,0x9B,
0x548F,0x87,
0x5490,0x83,
0x5491,0x8C,
0x5492,0x85,
0x5493,0x99,
0x5494,0x99,
0x5495,0x85,
0x5496,0x86,
0x5497,0x80,
0x5498,0x80,
0x5499,0x81,
0x549A,0x84,
0x549B,0x98,
0x549C,0x9F,
0x549D,0x9F,
0x549E,0x8C,
0x549F,0x83,
0x54A0,0x81,
0x54A1,0x9B,
0x54A2,0x9E,
0x54A3,0x9E,
0x54A4,0x77,
0x54A5,0x88,
0x54A6,0x89,
0x54A7,0x82,
0x54A8,0x87,
0x54A9,0x85,
0x54AA,0x7D,
0x54AB,0x72,
0x54AC,0x7F,
0x54AD,0x71,
0x54AE,0x8E,
0x54AF,0x83,
0x54B0,0x78,
0x54B1,0x65,
0x54B2,0x62,
0x54B3,0x67,
0x54B4,0x72,
0x54B5,0x8A,
0x54B6,0x64,
0x54B7,0x60,
0x54B8,0x6E,
0x54B9,0x6C,
0x54BA,0x7A,
0x54BB,0x72,
0x54BC,0x9A,
0x54BD,0x9F,
0x54BE,0x9F,
0x54BF,0x9B,
0x54C0,0x87,
0x54C1,0x86,
0x54C2,0x9B,
0x54C3,0x9E,
0x54C4,0x99,
0x54C5,0x84,
0x54C6,0x81,
0x54C7,0x82,
0x54C8,0x87,
0x54C9,0x84,
0x54CA,0x80,
0x54CB,0x8F,
0x54CC,0x88,
0x54CD,0x75,
0x54CE,0x82,
0x54CF,0x8F,
0x54D0,0x74,
0x54D1,0x72,
0x54D2,0x72,
0x54D3,0x73,
0x54D4,0x76,
0x54D5,0x72,
0x54D6,0x65,
0x54D7,0x60,
0x54D8,0x64,
0x54D9,0x7B,
0x54DA,0x7F,
0x54DB,0x7A,
0x54DC,0x62,
0x54DD,0x6E,
0x54DE,0x62,
0x54DF,0x64,
0x5500,0x82,
0x5501,0x89,
0x5502,0x88,
0x5503,0x8A,
0x5504,0x76,
0x5505,0x7C,
0x5506,0x80,
0x5507,0x8D,
0x5508,0x74,
0x5509,0x71,
0x550A,0x77,
0x550B,0x7C,
0x550C,0x9E,
0x550D,0x84,
0x550E,0x88,
0x550F,0x74,
0x5510,0x77,
0x5511,0x73,
0x5512,0x9E,
0x5513,0x9A,
0x5514,0x8C,
0x5515,0x89,
0x5516,0x74,
0x5517,0x72,
0x5518,0x86,
0x5519,0x8D,
0x551A,0x75,
0x551B,0x77,
0x551C,0x71,
0x551D,0x73,
0x551E,0x83,
0x551F,0x89,
0x5520,0x73,
0x5521,0x7C,
0x5522,0x72,
0x5523,0x71,
0x5524,0x7C,
0x5525,0x7C,
0x5526,0x64,
0x5527,0x7B,
0x5528,0x70,
0x5529,0x8F,
0x552A,0x71,
0x552B,0x70,
0x552C,0x73,
0x552D,0x76,
0x552E,0x8A,
0x552F,0x81,
0x5530,0x7F,
0x5531,0x72,
0x5532,0x76,
0x5533,0x8A,
0x5534,0x8D,
0x5535,0x87,
0x5536,0x7E,
0x5537,0x7F,
0x5538,0x72,
0x5539,0x71,
0x553A,0x8C,
0x553B,0x86,
0x553C,0x7D,
0x553D,0x71,
0x553E,0x76,
0x553F,0x74,
0x5540,0x8F,
0x5541,0x80,
0x5542,0x7D,
0x5543,0x77,
0x5544,0x8B,
0x5545,0x8E,
0x5546,0x8C,
0x5547,0x8E,
0x5548,0x8F,
0x5549,0x8F,
0x554A,0x9E,
0x554B,0x98,
0x554C,0x80,
0x554D,0x8E,
0x554E,0x72,
0x554F,0x70,
0x5550,0x8D,
0x5551,0x8C,
0x5552,0x8B,
0x5553,0x71,
0x5554,0x7C,
0x5555,0x7F,
0x5556,0x75,
0x5557,0x74,
0x5558,0x70,
0x5559,0x72,
0x555A,0x72,
0x555B,0x70,
0x555C,0x8E,
0x555D,0x8E,
0x555E,0x77,
0x555F,0x70,
0x5560,0x77,
0x5561,0x74,
0x5562,0x8D,
0x5563,0x8F,
0x5564,0x8B,
0x5565,0x77,
0x5566,0x89,
0x5567,0x8A,
0x5568,0x8F,
0x5569,0x89,
0x556A,0x8B,
0x556B,0x8A,
0x556C,0x67,
0x556D,0x7F,
0x556E,0x76,
0x556F,0x75,
0x5570,0x88,
0x5571,0x8A,
0x5572,0x60,
0x5573,0x7F,
0x5574,0x70,
0x5575,0x74,
0x5576,0x8E,
0x5577,0x8A,
0x5578,0x68,
0x5579,0x78,
0x557A,0x71,
0x557B,0x8B,
0x557C,0x81,
0x557D,0x82,
0x557E,0x6C,
0x557F,0x79,
0x5580,0x8A,
0x5581,0x8F,
0x5582,0x87,
0x5583,0x82,
0x5584,0x63,
0x5585,0x65,
0x5586,0x73,
0x5587,0x77,
0x5588,0x8E,
0x5589,0x8B,
0x558A,0x6C,
0x558B,0x61,
0x558C,0x7B,
0x558D,0x7E,
0x558E,0x76,
0x558F,0x71,
0x5590,0x85,
0x5591,0x8B,
0x5592,0x78,
0x5593,0x7A,
0x5594,0x7D,
0x5595,0x77,
0x5596,0x93,
0x5597,0x82,
0x5598,0x76,
0x5599,0x72,
0x559A,0x70,
0x559B,0x89,
0x559C,0xAE,
0x559D,0x98,
0x559E,0x88,
0x559F,0x71,
0x55A0,0x72,
0x55A1,0x77,
0x55A2,0x94,
0x55A3,0x9A,
0x55A4,0x76,
0x55A5,0x7D,
0x55A6,0x7F,
0x55A7,0x70,
0x55A8,0x93,
0x55A9,0x9A,
0x55AA,0x8A,
0x55AB,0x71,
0x55AC,0x70,
0x55AD,0x74,
0x55AE,0x90,
0x55AF,0x9E,
0x55B0,0x82,
0x55B1,0x8E,
0x55B2,0x8B,
0x55B3,0x88,
0x55B4,0x70,
0x55B5,0x8C,
0x55B6,0x9B,
0x55B7,0x99,
0x55B8,0x8F,
0x55B9,0x75,
0x55BA,0x7B,
0x55BB,0x77,
0x55BC,0x8C,
0x55BD,0x82,
0x55BE,0x76,
0x55BF,0x7F,
0x55C0,0x61,
0x55C1,0x7C,
0x55C2,0x75,
0x55C3,0x8A,
0x55C4,0x72,
0x55C5,0x7C,
0x55C6,0x7B,
0x55C7,0x70,
0x55C8,0x8F,
0x55C9,0x8C,
0x55CA,0x77,
0x55CB,0x70,
0x55CC,0x7C,
0x55CD,0x77,
0x55CE,0x8C,
0x55CF,0x82,
0x55D0,0x8B,
0x55D1,0x77,
0x55D2,0x7F,
0x55D3,0x70,
0x55D4,0x88,
0x55D5,0x8F,
0x55D6,0x8B,
0x55D7,0x74,
0x55D8,0x8C,
0x55D9,0x83,
0x55DA,0x75,
0x55DB,0x74,
0x55DC,0x88,
0x55DD,0x75,
0x55DE,0x9A,
0x55DF,0x84,
0x55E0,0x8E,
0x55E1,0x88,
0x55E2,0x8B,
0x55E3,0x74,
0x55E4,0x9D,
0x55E5,0x9C,
0x55E6,0x80,
0x55E7,0x82,
0x55E8,0x88,
0x55E9,0x76,
0x55EA,0x9C,
0x55EB,0x9B,
0x55EC,0x8E,
0x55ED,0x75,
0x55EE,0x75,
0x55EF,0x73,
0x55F0,0x83,
0x55F1,0x8F,
0x55F2,0x72,
0x55F3,0x7E,
0x55F4,0x70,
0x55F5,0x72,
0x55F6,0x8E,
0x55F7,0x75,
0x55F8,0x72,
0x55F9,0x7C,
0x55FA,0x73,
0x55FB,0x73,
0x55FC,0x74,
0x55FD,0x73,
0x55FE,0x74,
0x55FF,0x75,
0x5600,0x74,
0x5601,0x88,
0x5602,0x7C,
0x5603,0x7E,
0x5604,0x73,
0x5605,0x76,
0x5606,0x8B,
0x5607,0x8F,
0x5608,0x7A,
0x5609,0x7A,
0x560A,0x7F,
0x560B,0x72,
0x560C,0x8B,
0x560D,0x83,
0x560E,0x7E,
0x560F,0x7E,
0x5610,0x72,
0x5611,0x77,
0x5612,0x89,
0x5613,0x86,
0x5614,0x71,
0x5615,0x70,
0x5616,0x72,
0x5617,0x76,
0x5618,0x75,
0x5619,0x82,
0x561A,0x71,
0x561B,0x72,
0x561C,0x7A,
0x561D,0x7B,
0x561E,0x7D,
0x561F,0x8A,
0x5620,0x7D,
0x5621,0x74,
0x5622,0x89,
0x5623,0x8B,
0x5624,0x75,
0x5625,0x76,
0x5626,0x72,
0x5627,0x77,
0x5628,0x8F,
0x5629,0x89,
0x562A,0x74,
0x562B,0x71,
0x562C,0x7D,
0x562D,0x70,
0x562E,0x89,
0x562F,0x89,
0x5630,0x77,
0x5631,0x72,
0x5632,0x7E,
0x5633,0x72,
0x5634,0x8B,
0x5635,0x8B,
0x5636,0x76,
0x5637,0x7C,
0x5638,0x7F,
0x5639,0x77,
0x563A,0x86,
0x563B,0x86,
0x563C,0x8E,
0x563D,0x76,
0x563E,0x76,
0x563F,0x8D,
0x5640,0x93,
0x5641,0x93,
0x5642,0x85,
0x5643,0x8E,
0x5644,0x62,
0x5645,0x78,
0x5646,0x7F,
0x5647,0x7C,
0x5648,0x74,
0x5649,0x77,
0x564A,0x63,
0x564B,0x79,
0x564C,0x71,
0x564D,0x74,
0x564E,0x8F,
0x564F,0x88,
0x5650,0x6C,
0x5651,0x7F,
0x5652,0x88,
0x5653,0x8D,
0x5654,0x85,
0x5655,0x80,
0x5656,0x6A,
0x5657,0x7A,
0x5658,0x72,
0x5659,0x77,
0x565A,0x81,
0x565B,0x8F,
0x565C,0x68,
0x565D,0x64,
0x565E,0x65,
0x565F,0x79,
0x5660,0x77,
0x5661,0x70,
0x5662,0x54,
0x5663,0x66,
0x5664,0x78,
0x5665,0x7F,
0x5666,0x7D,
0x5667,0x7E,
0x5668,0x90,
0x5669,0x84,
0x566A,0x8D,
0x566B,0x8F,
0x566C,0x75,
0x566D,0x8B,
0x566E,0x90,
0x566F,0x85,
0x5670,0x8A,
0x5671,0x76,
0x5672,0x70,
0x5673,0x74,
0x5674,0x94,
0x5675,0x87,
0x5676,0x71,
0x5677,0x7F,
0x5678,0x79,
0x5679,0x72,
0x567A,0xAC,
0x567B,0x98,
0x567C,0x88,
0x567D,0x76,
0x567E,0x72,
0x567F,0x75,
0x5680,0x94,
0x5681,0x80,
0x5682,0x75,
0x5683,0x76,
0x5684,0x74,
0x5685,0x8D,
0x5686,0x91,
0x5687,0x89,
0x5688,0x7E,
0x5689,0x78,
0x568A,0x71,
0x568B,0x8C,
0x568C,0x64,
0x568D,0x73,
0x568E,0x75,
0x568F,0x8B,
0x5690,0x75,
0x5691,0x73,
0x5692,0x7A,
0x5693,0x70,
0x5694,0x89,
0x5695,0x8F,
0x5696,0x74,
0x5697,0x73,
0x5698,0x67,
0x5699,0x73,
0x569A,0x8E,
0x569B,0x8F,
0x569C,0x76,
0x569D,0x72,
0x569E,0x62,
0x569F,0x7C,
0x56A0,0x8A,
0x56A1,0x8B,
0x56A2,0x72,
0x56A3,0x79,
0x56A4,0x65,
0x56A5,0x8A,
0x56A6,0x81,
0x56A7,0x86,
0x56A8,0x74,
0x56A9,0x7E,
0x56AA,0x7E,
0x56AB,0x81,
0x56AC,0x93,
0x56AD,0x90,
0x56AE,0x83,
0x56AF,0x76,
0x56B0,0x98,
0x56B1,0x82,
0x56B2,0x86,
0x56B3,0x9C,
0x56B4,0x98,
0x56B5,0x85,
0x56B6,0x85,
0x56B7,0x8E,
0x56B8,0x8D,
0x56B9,0x9A,
0x56BA,0x84,
0x56BB,0x81,
0x56BC,0x84,
0x56BD,0x76,
0x56BE,0x73,
0x56BF,0x8E,
0x56C0,0x8C,
0x56C1,0x8A,
0x56C2,0x81,
0x56C3,0x70,
0x56C4,0x7B,
0x56C5,0x74,
0x56C6,0x77,
0x56C7,0x76,
0x56C8,0x88,
0x56C9,0x72,
0x56CA,0x78,
0x56CB,0x76,
0x56CC,0x71,
0x56CD,0x74,
0x56CE,0x8A,
0x56CF,0x7C,
0x56D0,0x7B,
0x56D1,0x71,
0x56D2,0x8A,
0x56D3,0x8A,
0x56D4,0x6D,
0x56D5,0x7D,
0x56D6,0x76,
0x56D7,0x78,
0x56D8,0x78,
0x56D9,0x7E,
0x56DA,0x6F,
0x56DB,0x7F,
0x56DC,0x77,
0x56DD,0x7C,
0x56DE,0x79,
0x56DF,0x7F,
0x56E0,0x6B,
0x56E1,0x7F,
0x56E2,0x8F,
0x56E3,0x77,
0x56E4,0x70,
0x56E5,0x75,
0x56E6,0x68,
0x56E7,0x7D,
0x56E8,0x84,
0x56E9,0x89,
0x56EA,0x89,
0x56EB,0x89,
0x56EC,0x64,
0x56ED,0x77,
0x56EE,0x98,
0x56EF,0x80,
0x56F0,0x8D,
0x56F1,0x89,
0x56F2,0x7F,
0x56F3,0x8F,
0x56F4,0x9D,
0x56F5,0x85,
0x56F6,0x8C,
0x56F7,0x8E,
0x56F8,0x9B,
0x56F9,0x75,
0x56FA,0x78,
0x56FB,0x7C,
0x56FC,0x76,
0x56FD,0x75,
0x56FE,0x9C,
0x56FF,0x8C,
0x5700,0x7C,
0x5701,0x70,
0x5702,0x8B,
0x5703,0x88,
0x5704,0x91,
0x5705,0x87,
0x5706,0x71,
0x5707,0x74,
0x5708,0x89,
0x5709,0x8A,
0x570A,0x97,
0x570B,0x85,
0x570C,0x73,
0x570D,0x77,
0x570E,0x75,
0x570F,0x74,
0x5710,0x9D,
0x5711,0x80,
0x5712,0x7E,
0x5713,0x72,
0x5714,0x71,
0x5715,0x77,
0x5716,0x9B,
0x5717,0x88,
0x5718,0x65,
0x5719,0x79,
0x571A,0x73,
0x571B,0x71,
0x571C,0x9B,
0x571D,0x9A,
0x571E,0x99,
0x571F,0x81,
0x5720,0x8B,
0x5721,0x88,
0x5722,0x88,
0x5723,0x82,
0x5724,0x85,
0x5725,0x8F,
0x5726,0x70,
0x5727,0x75,
0x5728,0x7C,
0x5729,0x76,
0x572A,0x88,
0x572B,0x7C,
0x572C,0x64,
0x572D,0x70,
0x572E,0x78,
0x572F,0x7C,
0x5730,0x77,
0x5731,0x65,
0x5732,0x62,
0x5733,0x73,
0x5734,0x7E,
0x5735,0x7C,
0x5736,0x73,
0x5737,0x7A,
0x5738,0x66,
0x5739,0x7F,
0x573A,0x73,
0x573B,0x73,
0x573C,0x7D,
0x573D,0x7A,
0x573E,0x64,
0x573F,0x7B,
0x5740,0x67,
0x5741,0x78,
0x5742,0x78,
0x5743,0x71,
0x5744,0x77,
0x5745,0x7D,
0x5746,0x73,
0x5747,0x7D,
0x5748,0x7F,
0x5749,0x77,
0x574A,0x75,
0x574B,0x7C,
0x574C,0x8E,
0x574D,0x8A,
0x574E,0x77,
0x574F,0x8F,
0x5750,0x88,
0x5751,0x7F,
0x5752,0x82,
0x5753,0x8D,
0x5754,0x88,
0x5755,0x87,
0x5756,0x82,
0x5757,0x7F,
0x5758,0x82,
0x5759,0x80,
0x575A,0x83,
0x575B,0x98,
0x575C,0x81,
0x575D,0x77,
0x575E,0x8F,
0x575F,0x82,
0x5760,0x85,
0x5761,0x9D,
0x5762,0x9A,
0x5763,0x82,
0x5764,0x8C,
0x5765,0x77,
0x5766,0x72,
0x5767,0x7E,
0x5768,0x76,
0x5769,0x8D,
0x576A,0x8F,
0x576B,0x8A,
0x576C,0x76,
0x576D,0x72,
0x576E,0x8B,
0x576F,0x87,
0x5770,0x89,
0x5771,0x88,
0x5772,0x8A,
0x5773,0x76,
0x5774,0x82,
0x5775,0x98,
0x5776,0x8A,
0x5777,0x75,
0x5778,0x75,
0x5779,0x71,
0x577A,0x8D,
0x577B,0x99,
0x577C,0x8A,
0x577D,0x76,
0x577E,0x70,
0x577F,0x7C,
0x5780,0x8B,
0x5781,0x84,
0x5782,0x8A,
0x5783,0x76,
0x5784,0x7F,
0x5785,0x7B,
0x5786,0x71,
0x5787,0x8E,
0x5788,0x77,
0x5789,0x7C,
0x578A,0x64,
0x578B,0x7C,
0x578C,0x76,
0x578D,0x8F,
0x578E,0x8E,
0x578F,0x7C,
0x5790,0x7B,
0x5791,0x72,
0x5792,0x77,
0x5793,0x8B,
0x5794,0x87,
0x5795,0x76,
0x5796,0x7F,
0x5797,0x74,
0x5798,0x75,
0x5799,0x77,
0x579A,0x9B,
0x579B,0x8B,
0x579C,0x75,
0x579D,0x82,
0x579E,0x82,
0x579F,0x89,
0x57A0,0x9D,
0x57A1,0x87,
0x57A2,0x9B,
0x57A3,0x90,
0x57A4,0x9C,
0x57A5,0x9A,
0x57A6,0x9C,
0x57A7,0x99,
0x57A8,0x93,
0x57A9,0x95,
0x57AA,0x91,
0x57AB,0x90,
0x57AC,0x71,
0x57AD,0x8C,
0x57AE,0x93,
0x57AF,0x9B,
0x57B0,0x80,
0x57B1,0x75,
0x57B2,0x65,
0x57B3,0x8A,
0x57B4,0x98,
0x57B5,0x86,
0x57B6,0x8C,
0x57B7,0x88,
0x57B8,0x69,
0x57B9,0x7D,
0x57BA,0x80,
0x57BB,0x8E,
0x57BC,0x88,
0x57BD,0x8E,
0x57BE,0x6A,
0x57BF,0x7E,
0x57C0,0x89,
0x57C1,0x76,
0x57C2,0x70,
0x57C3,0x75,
0x57C4,0x68,
0x57C5,0x78,
0x57C6,0x71,
0x57C7,0x79,
0x57C8,0x7B,
0x57C9,0x7E,
0x57CA,0x63,
0x57CB,0x79,
0x57CC,0x72,
0x57CD,0x78,
0x57CE,0x65,
0x57CF,0x67,
0x57D0,0x82,
0x57D1,0x8A,
0x57D2,0x7A,
0x57D3,0x7E,
0x57D4,0x72,
0x57D5,0x73,
0x57D6,0x9B,
0x57D7,0x8D,
0x57D8,0x7F,
0x57D9,0x72,
0x57DA,0x71,
0x57DB,0x73,
0x57DC,0x93,
0x57DD,0x87,
0x57DE,0x73,
0x57DF,0x71,
0x57E0,0x77,
0x57E1,0x73,
0x57E2,0x9C,
0x57E3,0x80,
0x57E4,0x72,
0x57E5,0x71,
0x57E6,0x75,
0x57E7,0x71,
0x57E8,0x84,
0x57E9,0x75,
0x57EA,0x65,
0x57EB,0x79,
0x57EC,0x71,
0x57ED,0x76,
0x57EE,0x8E,
0x57EF,0x7D,
0x57F0,0x63,
0x57F1,0x67,
0x57F2,0x7F,
0x57F3,0x71,
0x57F4,0x7F,
0x57F5,0x72,
0x57F6,0x72,
0x57F7,0x64,
0x57F8,0x66,
0x57F9,0x7A,
0x57FA,0x7C,
0x57FB,0x72,
0x57FC,0x71,
0x57FD,0x78,
0x57FE,0x66,
0x57FF,0x72,
0x5800,0x7E,
0x5801,0x73,
0x5802,0x8A,
0x5803,0x79,
0x5804,0x61,
0x5805,0x70,
0x5806,0x70,
0x5807,0x8E,
0x5808,0x80,
0x5809,0x74,
0x580A,0x7C,
0x580B,0x8A,
0x580C,0x81,
0x580D,0x9E,
0x580E,0x91,
0x580F,0x98,
0x5810,0x8F,
0x5811,0x81,
0x5812,0x9D,
0x5813,0x96,
0x5814,0xA8,
0x5815,0x93,
0x5816,0x83,
0x5817,0x81,
0x5818,0x81,
0x5819,0x81,
0x581A,0x87,
0x581B,0x9C,
0x581C,0x9B,
0x581D,0x8D,
0x581E,0x82,
0x581F,0x8D,
0x5820,0x8C,
0x5821,0x84,
0x5822,0x80,
0x5823,0x70,
0x5824,0x83,
0x5825,0x8E,
0x5826,0x75,
0x5827,0x83,
0x5828,0x8C,
0x5829,0x7C,
0x582A,0x89,
0x582B,0x71,
0x582C,0x72,
0x582D,0x8B,
0x582E,0x77,
0x582F,0x7B,
0x5830,0x7D,
0x5831,0x7B,
0x5832,0x65,
0x5833,0x72,
0x5834,0x73,
0x5835,0x64,
0x5836,0x67,
0x5837,0x66,
0x5838,0x66,
0x5839,0x7F,
0x583A,0x71,
0x583B,0x79,
0x583C,0x7F,
0x583D,0x73,
0x583E,0x72,
0x583F,0x79,
0x5840,0x71,
0x5841,0x8B,
0x5842,0x71,
0x5843,0x77,
0x5844,0x77,
0x5845,0x73,
0x5846,0x88,
0x5847,0x86,
0x5848,0x76,
0x5849,0x77,
0x584A,0x75,
0x584B,0x71,
0x584C,0x8C,
0x584D,0x85,
0x584E,0x74,
0x584F,0x75,
0x5850,0x77,
0x5851,0x70,
0x5852,0x8C,
0x5853,0x85,
0x5854,0x76,
0x5855,0x71,
0x5856,0x7C,
0x5857,0x78,
0x5858,0x77,
0x5859,0x83,
0x585A,0x76,
0x585B,0x7D,
0x585C,0x7B,
0x585D,0x67,
0x585E,0x7E,
0x585F,0x8A,
};

static kal_uint16 addr_data_pair_preview[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};
static kal_uint16 addr_data_pair_capture[] = {
0x0303,0x06,
0x0304,0x01,
0x0305,0x38,
0x0344,0x01,
0x0345,0xab,
0x034a,0x0a,
0x034b,0x00,
0x3501,0x13,
0x3502,0x64,
0x3603,0x0b,
0x3608,0x63,
0x360d,0x61,
0x3622,0x55,
0x3633,0x03,
0x3635,0x0c,
0x3636,0x0c,
0x3639,0xcc,
0x363a,0xcc,
0x366b,0x02,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x67,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x19,
0x3809,0x80,
0x380a,0x13,
0x380b,0x20,
0x380c,0x05,
0x380d,0x28,
0x380e,0x14,
0x380f,0x00,
0x3811,0x12,
0x3812,0x00,
0x3813,0x11,
0x3814,0x11,
0x3815,0x11,
0x3820,0x04,
0x3821,0x00,
0x3822,0x00,
0x4012,0x7d,
0x4015,0x04,
0x4016,0x1b,
0x4018,0x07,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x0b,
0x5000,0xc9,
0x5001,0x42,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x00,
0x5014,0x00,
0x5015,0x06,
0x5035,0x10,
0x5037,0x10,
0x5038,0x19,
0x5039,0x80,
0x503a,0x13,
0x503b,0x20,
0x5185,0x0c,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0x05,
0x5208,0x41,
0x5380,0x0f,
0x5381,0x00,
0x5386,0x19,
0x5387,0xa0,
0x5388,0x13,
0x5389,0x40,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,

};
static kal_uint16 addr_data_pair_normal_video[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc1,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x07,
0x380b,0x30,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x01,
0x3813,0x39,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};



static kal_uint16 addr_data_pair_hs_video[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};
static kal_uint16 addr_data_pair_slim_video[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,


};

static kal_uint16 addr_data_pair_custom1[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};

static kal_uint16 addr_data_pair_custom2[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};

static kal_uint16 addr_data_pair_custom3[] = {
0x0303,0x06,
0x0304,0x01,
0x0305,0xc8,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0c,
0x3502,0x8d,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x01,
0x380d,0xc2,
0x380e,0x0e,
0x380f,0x1b,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x07,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,
};
static kal_uint16 addr_data_pair_custom4[] = {
0x0303,0x06,
0x0304,0x01,
0x0305,0x28,
0x0344,0x01,
0x0345,0xbb,
0x034a,0x05,
0x034b,0x00,
0x3501,0x15,
0x3502,0x0e,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x06,
0x3809,0x60,
0x380a,0x04,
0x380b,0xc8,
0x380c,0x02,
0x380d,0x2b,
0x380e,0x16,
0x380f,0xde,
0x3811,0x06,
0x3812,0x00,
0x3813,0x05,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x11,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x0c,
0x5000,0xa9,
0x5001,0x22,
0x5002,0x11,
0x5003,0x7a,
0x5005,0x00,
0x5014,0x00,
0x5015,0x06,
0x5035,0x10,
0x5037,0x10,
0x5038,0x19,
0x5039,0x80,
0x503a,0x13,
0x503b,0x20,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc7,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x10,
0x5c71,0x10,
0x5200,0x6f,
};

static kal_uint16 addr_data_pair_custom5[] = {
0x0303,0x06,
0x0304,0x00,
0x0305,0xc0,
0x0344,0x01,
0x0345,0xb1,
0x034a,0x06,
0x034b,0x00,
0x3501,0x0b,
0x3502,0xb0,
0x3603,0x0b,
0x3608,0x4a,
0x360d,0x4a,
0x3622,0x66,
0x3633,0x06,
0x3635,0x2c,
0x3636,0x2c,
0x3639,0x44,
0x363a,0x33,
0x366b,0x00,
0x370b,0xb0,
0x3712,0x00,
0x3714,0x61,
0x3800,0x00,
0x3801,0x00,
0x3802,0x00,
0x3803,0x00,
0x3804,0x19,
0x3805,0x9f,
0x3806,0x13,
0x3807,0x3f,
0x3808,0x0c,
0x3809,0xc0,
0x380a,0x09,
0x380b,0x90,
0x380c,0x04,
0x380d,0x38,
0x380e,0x0b,
0x380f,0xc0,
0x3811,0x0a,
0x3812,0x00,
0x3813,0x09,
0x3814,0x22,
0x3815,0x22,
0x3820,0x05,
0x3821,0x09,
0x3822,0x00,
0x4012,0x0d,
0x4015,0x02,
0x4016,0x0d,
0x4018,0x03,
0x401e,0x01,
0x401f,0x0c,
0x4837,0x13,
0x5000,0x89,
0x5001,0x02,
0x5002,0x01,
0x5003,0x7a,
0x5005,0x08,
0x5014,0x30,
0x5015,0x06,
0x5035,0x08,
0x5037,0x08,
0x5038,0x0c,
0x5039,0xc0,
0x503a,0x09,
0x503b,0x90,
0x5185,0x0b,
0x518c,0x01,
0x518d,0x01,
0x518e,0x01,
0x518f,0x01,
0x5207,0xff,
0x5208,0xc1,
0x5380,0x0c,
0x5381,0x06,
0x5386,0x14,
0x5387,0x60,
0x5388,0x0f,
0x5389,0xc8,
0x5880,0xc5,
0x5884,0x18,
0x5885,0x08,
0x5886,0x08,
0x5887,0x18,
0x5889,0x05,
0x588a,0x00,
0x58c0,0x10,
0x58c2,0x0e,
0x58c3,0x0c,
0x58c4,0x04,
0x58c5,0x01,
0x58c6,0xf7,
0x58c8,0x6f,
0x58ca,0x1d,
0x58cb,0x01,
0x58cc,0xfd,
0x5c6f,0x02,
0x5c71,0x00,
0x5200,0x6f,

};



static void sensor_init(void)
{
	//Global setting 
	int i;
    PK_DBG("start");
	table_write_cmos_sensor(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
	if(0x01==crosstalk_main_ov32alq[0]){
	  for(i = 0; i<288; i++){
       write_cmos_sensor_16_8(0x53c0 + i, crosstalk_main_ov32alq[i+1]); 
      }
	}
    PK_DBG("end \n");				  
}	/*	sensor_init  */


static void preview_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	PK_DBG("end \n");
}	/*	preview_setting  */

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps)
{
	

    PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	

	PK_DBG("end \n");
}


static void normal_video_setting(kal_uint16 currefps)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void hs_video_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void slim_video_setting(void)
{
    PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
	PK_DBG("end \n");

}

static void custom1_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom2_setting(void)
{
PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	PK_DBG("end \n");


}

static void custom3_setting(void)
{
PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	PK_DBG("end \n");


}

static void custom4_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	PK_DBG("end \n");

}
static void custom5_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom5,
		   sizeof(addr_data_pair_custom5) / sizeof(kal_uint16));
	PK_DBG("end \n");

}

static kal_uint32 return_sensor_id(void)
{
	
    return ((read_cmos_sensor_16_8(0x300a) << 8) | ((read_cmos_sensor_16_8(0x300b))));
}

//vivo lishuo add MTK patch for [H202012312161] 20210218 begin
static kal_uint32 test_i2c_is_timeout1()
{
    int ret,temp;
    kal_uint16 get_byte;
    char pu_send_cmd[1] = { 0xf0 };
	struct IMGSENSOR_I2C_CFG *pgi2c_cfg_legacy = imgsensor_i2c_get_device();
	
	if(pgi2c_cfg_legacy->pinst->pi2c_client == NULL){
		return 1;
	}	
	PK_DBG("[test_i2c_is_timeout1]:[%s ] E \n",__FUNCTION__);
	
//pr_info(PFX "[%s ] E \n",__FUNCTION__);
    temp = pgi2c_cfg_legacy->pinst->pi2c_client->adapter->timeout;
    pgi2c_cfg_legacy->pinst->pi2c_client->adapter->timeout = HZ/20;
    ret = iReadRegI2C(pu_send_cmd, 1, (u8 *) &get_byte, 1,
            imgsensor.i2c_write_id);
    pgi2c_cfg_legacy->pinst->pi2c_client->adapter->timeout = temp;    
	//pr_err(PFX "[%s ] ret =%d \n",__FUNCTION__,ret);
	PK_DBG("[test_i2c_is_timeout1]:[%s ] ret =%d \n",__FUNCTION__,ret);
    return (ret==-2)?1:0;
}
//vivo lishuo add MTK patch for [H202012312161] 20210218 end

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
#if 0
	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	PK_DBG(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 4){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
#endif
//vivo lishuo add MTK patch for [H202012312161] 20210218 begin
    if (test_i2c_is_timeout1()) {    
	 //pr_err(PFX "[%s ] iic  timeout \n",__FUNCTION__);	
	 PK_DBG("[get_imgsensor_id]:[%s ] iic  timeout \n",__FUNCTION__);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
//vivo lishuo add MTK patch for [H202012312161] 20210218 end

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			PK_DBG("read out sensor id 0x%x \n",*sensor_id);
			if ( *sensor_id == imgsensor_info.sensor_id) {
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				/*vivo lxd add for CameraEM otp errorcode*/
				pr_debug("start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = ov32alq_otp_read();
				pr_debug("read eeprom ---vivo_otp_read_when_power_on = %d,OV32ALQ_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, OV32ALQ_OTP_ERROR_CODE);
				/*vivo lxd add end*/
				return ERROR_NONE;
			}
			PK_DBG("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	PK_DBG("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
	    		sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			PK_DBG("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x1028;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
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
	PK_DBG("E\n");

	return ERROR_NONE;
}				/*      close  */


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
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      preview   */

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
	PK_DBG("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	
	capture_setting(imgsensor.current_fps);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,      
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)                       
{                                                                                
	PK_DBG("E\n");                                                                
                                                                                 
	spin_lock(&imgsensor_drv_lock);                                                
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;                                
	imgsensor.pclk = imgsensor_info.custom3.pclk;                                  
	imgsensor.line_length = imgsensor_info.custom3.linelength;                     
	imgsensor.frame_length = imgsensor_info.custom3.framelength;                   
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;               
	imgsensor.autoflicker_en = KAL_FALSE;                                          
	spin_unlock(&imgsensor_drv_lock);                                              
                                                                                 
	custom3_setting();                                    
	set_mirror_flip(imgsensor.mirror);                                             
                                                                                 
	return ERROR_NONE;                                                             
}	/*	custom3   */                                                               

 
 static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,	  
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)						
 {																				  
	 PK_DBG("E\n"); 															   
																				  
	 spin_lock(&imgsensor_drv_lock);												
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;								
	 imgsensor.pclk = imgsensor_info.custom4.pclk;									
	 imgsensor.line_length = imgsensor_info.custom4.linelength; 					
	 imgsensor.frame_length = imgsensor_info.custom4.framelength;					
	 imgsensor.min_frame_length = imgsensor_info.custom4.framelength;				
	 imgsensor.autoflicker_en = KAL_FALSE;											
	 spin_unlock(&imgsensor_drv_lock);												
																				  
	 custom4_setting();									
	 set_mirror_flip(imgsensor.mirror); 											
																				  
	 return ERROR_NONE; 															
 }	 /*  custom4   */
 
  static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,	  
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)						
 {																				  
	 PK_DBG("E\n"); 															   
																				  
	 spin_lock(&imgsensor_drv_lock);												
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;								
	 imgsensor.pclk = imgsensor_info.custom5.pclk;									
	 imgsensor.line_length = imgsensor_info.custom5.linelength; 					
	 imgsensor.frame_length = imgsensor_info.custom5.framelength;					
	 imgsensor.min_frame_length = imgsensor_info.custom5.framelength;				
	 imgsensor.autoflicker_en = KAL_FALSE;											
	 spin_unlock(&imgsensor_drv_lock);												
																				  
	 custom5_setting();									
	 set_mirror_flip(imgsensor.mirror); 											
																				  
	 return ERROR_NONE; 															
 }	 /*  custom  */

 																
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	PK_DBG("%s E\n", __func__);
	
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;
	
	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;
	
	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*PK_DBG("get_info -> scenario_id = %d\n", scenario_id);*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

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

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* change pdaf support mode to pdaf VC mode */
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

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
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

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

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

	        break;
	    case MSDK_SCENARIO_ID_CUSTOM2:
	        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx; 
	        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc; 

	        break;   
	        
	     case MSDK_SCENARIO_ID_CUSTOM3:                                                                                    
	         sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;                                                 
	         sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;                                                 
	                                                                                                                        
	         sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;      
	                                                                                                                        
	         break;                                                                                                         
	 case MSDK_SCENARIO_ID_CUSTOM4: 																				   
		 sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 												
		 sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty; 												
																														
		 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;		
																														
		 break; 
	case MSDK_SCENARIO_ID_CUSTOM5: 																				   
		 sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 												
		 sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty; 												
																														
		 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;		
																														
		 break; 																														
	        
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
	        Custom1(image_window, sensor_config_data);
	        break;
	    case MSDK_SCENARIO_ID_CUSTOM2:
	        Custom2(image_window, sensor_config_data);
	        break;  
	    case MSDK_SCENARIO_ID_CUSTOM3:                     
	        Custom3(image_window, sensor_config_data);      
	        break;  
	case MSDK_SCENARIO_ID_CUSTOM4:					   
		Custom4(image_window, sensor_config_data);		
		break;	
	case MSDK_SCENARIO_ID_CUSTOM5:					   
		Custom5(image_window, sensor_config_data);		
		break;			
	default:
		PK_DBG("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //PK_DBG("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
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

static kal_uint32 set_auto_flicker_mode(
	kal_bool enable, UINT16 framerate)
{
	PK_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	PK_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
		    / framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
	    (frame_length > imgsensor_info.normal_video.framelength)
	  ? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.slim_video.framelength)
		? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;  
	case MSDK_SCENARIO_ID_CUSTOM3:                                                                                                              
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;                                          	
		spin_lock(&imgsensor_drv_lock);                                                                                                           	
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;     	
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;                                                       	
		imgsensor.min_frame_length = imgsensor.frame_length;                                                                                      	
		spin_unlock(&imgsensor_drv_lock);                                                                                                         	
		if (imgsensor.frame_length > imgsensor.shutter)                                                                                           	
			set_dummy();                                                                                                                            	
		break;   
	
	case MSDK_SCENARIO_ID_CUSTOM4:                                                                                                              
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;                                          	
		spin_lock(&imgsensor_drv_lock);                                                                                                           	
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;     	
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;                                                       	
		imgsensor.min_frame_length = imgsensor.frame_length;                                                                                      	
		spin_unlock(&imgsensor_drv_lock);                                                                                                         	
		if (imgsensor.frame_length > imgsensor.shutter)                                                                                           	
			set_dummy();                                                                                                                            	
		break;  
	case MSDK_SCENARIO_ID_CUSTOM5:                                                                                                              
		frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;                                          	
		spin_lock(&imgsensor_drv_lock);                                                                                                           	
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;     	
		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;                                                       	
		imgsensor.min_frame_length = imgsensor.frame_length;                                                                                      	
		spin_unlock(&imgsensor_drv_lock);                                                                                                         	
		if (imgsensor.frame_length > imgsensor.shutter)                                                                                           	
			set_dummy();                                                                                                                            	
		break;                                                                                                                                          	
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		PK_DBG("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*PK_DBG("scenario_id = %d\n", scenario_id);*/

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

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	PK_DBG("enable: %d\n", enable);

	if (enable) {
/* 0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9 */
		write_cmos_sensor_16_8(0x50c1, 0x01);
	} else {
		write_cmos_sensor_16_8(0x50c1, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    	struct SENSOR_VC_INFO_STRUCT *pvcinfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*PK_DBG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
		
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE: 	/*ITS test_sensor_fusion */
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2310000;
  		break;
		
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom5.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ imgsensor_info.custom4.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom5.framelength << 16)
				+ imgsensor_info.custom5.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
		PK_DBG(
			"feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
#endif
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_16_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
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
		set_auto_flicker_mode((BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		PK_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:	
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
			*feature_return_para_32 = 23;	/*BINNING_AVERAGED 2sum2avg*/
			break;
		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*feature_return_para_32 = 23; /*4sum*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/ 
			break;
		}
		PK_DBG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* PK_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		    case MSDK_SCENARIO_ID_CUSTOM1:
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		      break;
		    case MSDK_SCENARIO_ID_CUSTOM2:
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		      break; 
		    case MSDK_SCENARIO_ID_CUSTOM3:                                                                           
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));       
		     break; 
			case MSDK_SCENARIO_ID_CUSTOM4:																			 
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT)); 	  
			 break; 
			case MSDK_SCENARIO_ID_CUSTOM5:																			 
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[9],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT)); 	  
			 break; 																												  
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			    break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			    break;
            case MSDK_SCENARIO_ID_CUSTOM3:
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM1:
		
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		PK_DBG("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* video & capture use same setting*/
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
/*
    case SENSOR_FEATURE_SET_PDAF:
        PK_DBG("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode= *feature_data_16;
        break;
    */
    case SENSOR_FEATURE_GET_VC_INFO:
        PK_DBG("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		    break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM3:
	    	memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		    memcpy((void *)pvcinfo, (void *) &SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		default:	   
		    break;
		}
		break;  
		
	#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    PK_DBG("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  OV32ALQ_OTP_ERROR_CODE:%d \n", *feature_data,OV32ALQ_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			PK_DBG("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = OV32ALQ_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_main_ov32alq, sizeof(MUINT32)*13); 
				memcpy( feature_data+10, material_inf_main_ov32alq, sizeof(MUINT32)*4); 
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						PK_DBG("sn_inf_main_ov32alq[%d]= 0x%x\n", i, sn_inf_main_ov32alq[i]);
						}
				#endif
			}
				break;
			}
			break;
	#endif

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	PK_DBG("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		PK_DBG("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
		*feature_return_para_32 =  imgsensor.current_ae_effective_frame;
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
		
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom2.pclk /
			(imgsensor_info.custom2.linelength - 80))*
			imgsensor_info.custom2.grabwindow_width;
			break;
			
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom3.pclk /
			(imgsensor_info.custom3.linelength - 80))*
			imgsensor_info.custom3.grabwindow_width;
			break;
	
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom4.pclk /
			(imgsensor_info.custom4.linelength - 80))*
			imgsensor_info.custom4.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom5.pclk /
			(imgsensor_info.custom5.linelength - 80))*
			imgsensor_info.custom5.grabwindow_width;
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
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom3.mipi_pixel_rate;
			break;
		
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom5.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}
#include "../imgsensor_hop.c"
static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
	hop,
  	do_hop,
};

UINT32 OV32ALQ_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
