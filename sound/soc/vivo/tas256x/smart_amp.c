#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include "../../mediatek/common/mtk-sp-spk-amp.h"
#include "smart_amp.h"
#include "smartpa_debug_common_v1.h"

/* Mutex to serialize DSP read/write commands*/
static struct mutex routing_lock;

/*
    {TI_SMART_PA, (14<<15), (2<<15), (14<<15)}
    {NXP_SMART_PA, 747111, 109118, 747111}
    {AW_SMART_PA, (18<<15), 120161, (18<<15)}
*/

struct mtk_apr {
	uint32_t param_id;
	uint32_t data[AP_2_DSP_PAYLOAD_SIZE];
	struct CALIBRATION_RX_ calib_param;
};

#if defined(CONFIG_SND_SOC_MTK_AUDIO_DSP)
static int afe_smartamp_get_set_v1(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int32_t ret = 0;
	int32_t rd_length = 0;
	struct mtk_apr apr_buff;
	int32_t i = 0;

	pr_err("[SmartPA:%s] get_set %d param_id %d length %d",
		__func__, get_set, param_id, length);
	if (length > AP_2_DSP_PAYLOAD_SIZE*4) {
		pr_err("[SmartPA:%s] Out of bound length %d", length);
		return -1;
	}

	switch (get_set) {
	case AP_2_DSP_SET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_err("[SmartPA:%s] AP_2_DSP_SET_PARAM param_id %d", __func__, param_id);
			memcpy(apr_buff.data, data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, length+sizeof(param_id)); 
		}
		break;
	case AP_2_DSP_GET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_GET_PARAM param_id %d", __func__, param_id);
			memset(apr_buff.data, 0, length);
			//update param_id firstly, since param_id can not be sent by get_buf
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(param_id)); 
			if (ret == 0) {
				ret = mtk_spk_recv_ipi_buf_from_dsp((void *)&apr_buff, length+sizeof(param_id), &rd_length);
				pr_err("[SmartPA:%s] legen-AP_2_DSP_GET rd_length %d, %lld", __func__, rd_length, (unsigned long)(rd_length-sizeof(param_id)));
				if ((ret == 0) && (rd_length <= AP_2_DSP_PAYLOAD_SIZE*4+sizeof(param_id)) && (rd_length >= sizeof(param_id))) {
					memcpy(data_buff, apr_buff.data, rd_length-sizeof(param_id));
				}
			}

			//For Debug
			for (i = 0; i < length/4; i++)
				pr_err("[SmartPA:%s] apr_buff.data[%d] = 0x%0x", __func__, i, apr_buff.data[i]);

			break;
		}
	case AP_2_DSP_SEND_PARAM: /* wangkai add, to pass cali info (struct CALIBRATION_RX_) to DSP */
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_SEND_PARAM param_id %d", __func__, param_id);
			memcpy((void *)&(apr_buff.calib_param), data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(struct mtk_apr));
			if (ret < 0) {
				pr_err("%s: mtk_spk_send_ipi_buf_to_dsp, param send error!\n", __func__);
			} else {
				pr_info("%s: mtk_spk_send_ipi_buf_to_dsp, param send successful!\n", __func__);
			}
		}
		break;
	default:
		{
			break;
		}
	}

	return ret;
}

int afe_smartamp_algo_ctrl_v1(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int ret = 0;
	mutex_lock(&routing_lock);
	ret = afe_smartamp_get_set_v1(data_buff, param_id,
		get_set, length);
	mutex_unlock(&routing_lock);
	return ret;
}
#else
static int afe_smartamp_get_set_v1(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
    return 0;
}
#endif