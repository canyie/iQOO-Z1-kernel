#ifndef _SMART_AMP_H
#define _SMART_AMP_H

#include <linux/types.h>
#include <linux/delay.h>

#define SMART_AMP
#define MAX_DSP_PARAM_INDEX		881//600
#define MAX_CHANNELS	2

#define CALIB_FAILED 0xCACACACA

#define AP_2_DSP_SEND_PARAM     2  /* wangkai add */
#define AP_2_DSP_GET_PARAM		1
#define AP_2_DSP_SET_PARAM		0
#define AP_2_DSP_PAYLOAD_SIZE	14
#define AP_2_DSP_RX_PORT_ID		0x1004 /* TERT MI2S RX */
#define AP_2_DSP_TX_PORT_ID		0x1005 /* TERT MI2S TX */
#define SLAVE1		0x98
#define SLAVE2		0x9A
#define SLAVE3		0x9C
#define SLAVE4		0x9E
#define SMARTAMP_STATUS_NORMAL 0
#define SMARTAMP_STATUS_BYPASS 1
#define SMARTAMP_STATUS_MUTE   2

#define AFE_SA_SEND_ALL        3808
#define AFE_SA_GET_F0          3810
#define AFE_SA_GET_Q           3811
#define AFE_SA_GET_TV          3812
#define AFE_SA_GET_RE          3813
#define AFE_SA_CALIB_INIT      3814
#define AFE_SA_CALIB_DEINIT    3815
#define AFE_SA_SET_RE          3816
#define AFE_SA_F0_TEST_INIT    3817
#define AFE_SA_F0_TEST_DEINIT  3818
#define AFE_SA_SET_PROFILE     3819
#define AFE_SA_SET_STATUS      3820
#define AFE_SA_BYPASS		   3821

#define AFE_SA_IS_SPL_IDX(X)	((((X) >= 3810) && ((X) < 3899)) ? 1 : 0)

int afe_smartamp_algo_ctrl_v1(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length);

#endif
