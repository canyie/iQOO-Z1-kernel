/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas2560-calib.h
**
** Description:
**     header file for tas2560-calib.c
**
** =============================================================================
*/

#ifndef _TAS2560_CALIB_H
#define _TAS2560_CALIB_H
#if 0 //legen
#define TAS_GET_PARAM		1
#define TAS_SET_PARAM		0
#define TAS_PAYLOAD_SIZE	14
#define TAS_RX_PORT_ID		0x1004 /* TERT MI2S RX */
#define TAS_TX_PORT_ID		0x1005 /* TERT MI2S TX */
#define SLAVE1		0x98
#define SLAVE2		0x9A
#define SLAVE3		0x9C
#define SLAVE4		0x9E
#define SMARTAMP_STATUS_NORMAL 0
#define SMARTAMP_STATUS_BYPASS 1
#define SMARTAMP_STATUS_MUTE   2
#endif
#define AFE_PARAM_ID_ENABLE	0x00010203

int tas_calib_init(void);
void tas_calib_exit(void);

#endif /* _TAS2560_CALIB_H */
