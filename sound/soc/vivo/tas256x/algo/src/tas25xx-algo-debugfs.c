#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "../../dsp/tas_smart_amp_v2.h"
#include "../../codec/inc/tas256x-regmap.h"
#include "../../smart_amp.h"
#include "../../smartpa_debug_common_v1.h"

#define CALIB_FAILED	0xCACACACA
#define NO_CALIB		0xAA 		//170
#define RDC_MIN_L		(2621440)	//(3145728)  //3145728/2^19 = 6 ohm
#define RDC_MAX_L		(5242880)  	//5242880/2^19 = 10 ohm
#define RDC_MIN_R		(2621440)	//(3145728)  //3145728/2^19 = 6 ohm
#define RDC_MAX_R		(5242880)  	//5242880/2^19 = 10 ohm

/* user impedance: (detect_val/2^19)*/
#define TRANSF_IMPED_TO_USER_I(X) \
		(((X * 100) >> 19) / 100)
#define TRANSF_IMPED_TO_USER_M(X) \
		(((X * 100) >> 19) % 100)

#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"
#define MAX_CONTROL_NAME        48
#define STR_SZ_TAS 512

typedef struct {
	int mn_channels;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbg_dir;
#endif
	int mi2s_rx_port_id;
	int mi2s_tx_port_id;

	uint32_t calibRe[MAX_CHANNELS];
	uint32_t imped_min[MAX_CHANNELS]; // << 19
	uint32_t imped_max[MAX_CHANNELS]; // << 19
	uint32_t fres_min[MAX_CHANNELS]; // orig
	uint32_t fres_max[MAX_CHANNELS]; // orig
	uint32_t Qt[MAX_CHANNELS];
	struct i2c_client *i2c_client;
	int debugfs_init_done;
} smartpa_algo_data_t;

static smartpa_algo_data_t	*s_smartpa_algo;

/* wangkai add */
// if false, then get param info from file; else directly get param from priv
// everytime reboot or after calibration finish, this value becomes false
bool update_cali_flag;
// priv
struct smartpa_para smartpa;
/*    end      */


/* Interfaces required for customer*/
int smartpa_init_dbg(char *buffer, int size);
int smartpa_read_freq_dbg(char *buffer, int size);
void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
void smartpa_get_client(struct i2c_client **client, unsigned char addr);
int smartpa_check_calib_dbg(void);
/* Interfaces required for customer -- end*/

/* Prototypes */
static int smartpa_calib_get(uint32_t *calib_value);
static int alloc_memory_for_smartpa_algo_client(void);

static void smartpa_set_re(uint32_t *calibRe)
{
	int nSize = sizeof(uint32_t);
	int ret;
	uint8_t iter = 0;
	uint32_t paramid = 0;
	
	if (!s_smartpa_algo || !calibRe) {
		pr_err("[SmartPA-%d]: s_smartpa_algo or calibRe is NULL\n", __LINE__);
		return;
	}
	//if((calibRe != NO_CALIB) && (calibRe != CALIB_FAILED)) {
	for (iter = 0; iter < s_smartpa_algo->mn_channels; iter++) {
		if (calibRe[iter] != 0) {
			pr_info("[SmartPA-%d]: smartamp : Payload : %d", __LINE__, calibRe[iter]);
			paramid = ((iter+1) << 24) | (nSize << 16) | TAS_SA_SET_RE;
			#ifdef CONFIG_TAS256x_FOR_MTK
			ret = afe_smartamp_algo_ctrl((uint8_t *)&calibRe[iter], paramid, TAS_SET_PARAM, nSize);
			#else
			ret = tas25xx_smartamp_algo_ctrl ((uint8_t *)&calibRe[iter], paramid,
									TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
			#endif
			pr_err("[SmartPA-%d]: set Re[%d]: %d, ret=%d", __LINE__, iter, calibRe[iter], ret);
		} else
			pr_err("[SmartPA-%d]: Cannot set Re for calib status wrong", __LINE__);
	}
}

static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return false;
	}
	if (rdc > s_smartpa_algo->imped_min[iter] && rdc < s_smartpa_algo->imped_max[iter]) {
		return true;
	}

	pr_info("[SmartPA-%d]rdc check: rdc=%d invalid, [%d, %d] \n", __LINE__, rdc, s_smartpa_algo->imped_min[iter], s_smartpa_algo->imped_max[iter]);	
	return false;
}


static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	
	pr_info("[SmartPA-%d] smartpa_check_re enter.\n", __LINE__);
	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return false;
	}
	channels = s_smartpa_algo->mn_channels;
	
	for (iter = 0; iter < channels; iter++) {
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) || (s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d ok.\n",
					__LINE__, iter, s_smartpa_algo->calibRe[iter]);
			rc += 1;
		}
	}
	if (rc == channels)
		return true;
	
	rc = smartpa_calib_get(impedance);
	if (rc == 0) {
		pr_info("[SmartPA-%d] smartpa_check_re get re failed.\n", __LINE__);
		return false;
	}
	
	rc = 0;
	for (iter = 0; iter < channels; iter++) {
		s_smartpa_algo->calibRe[iter] = impedance[iter];
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) || (s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d success.\n", __LINE__, iter, impedance[iter]);
			rc += 1;
		} else {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d failed.\n", __LINE__, iter, impedance[iter]);
		}
	}
	
	if (rc == channels)
		return true;
	
	return false;
}

static int smartpa_calib_get(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	int channels = 1;
	
	if (!s_smartpa_algo || !calib_value) {
		pr_err("[SmartPA-%d]: s_smartpa_algo or calib_value is NULL\n", __LINE__);
		return false;
	}
	channels =  s_smartpa_algo->mn_channels;
	
	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*channels, &pos);
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %d  \n", __LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %d  \n", __LINE__, calib_value[1]);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]calibrate: No found\n", __LINE__);
		found = 0;
	}

	set_fs(old_fs);

	return found;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t smartpa_dbgfs_calibrate_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int ret = 0;
	uint32_t data = 0;
	uint8_t nSize = sizeof(uint32_t);
	uint32_t paramid = 0;
	uint32_t calib_re[MAX_CHANNELS];
	char *str;
	int ret_count = 0;
	uint8_t iter = 0, channels = 1;

	pr_info("%s\n", __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = s_smartpa_algo->mn_channels;
	
	//calib init
	for (iter = 0; iter < channels; iter++)	{
		data = 1;//Value is ignored
		paramid = ((TAS_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM,
						nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto end;
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: calib init\n", __LINE__);

	msleep(2*1000);

	//get Re
	for (iter = 0; iter < channels; iter++)	{
		paramid = ((TAS_SA_GET_RE)|((iter+1)<<24)|(1<<16));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4);
		#else
		ret = tas25xx_smartamp_algo_ctrl ((u8 *)&data, paramid,
				TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto deinit;

		calib_re[iter] = data;
		pr_info("[SmartPa-%d]debugfs: calib_re-%d 0x%x\n", __LINE__, (int)iter, (int)calib_re[iter]);
		
		if ((calib_re[iter] < s_smartpa_algo->imped_min[iter])
			|| (calib_re[iter] > s_smartpa_algo->imped_max[iter]))
			calib_re[iter] = CALIB_FAILED;

		s_smartpa_algo->calibRe[iter] = calib_re[iter];
		pr_info("[SmartPA-%d]debugfs calib_re[%d] is %d\n", __LINE__, iter, calib_re[iter]);
	}
	
	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc \n", __LINE__);
		ret = -ENOMEM;
		goto deinit;
	}
	ret = 0;
	if (channels == 2) {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x; ", calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %02d.%02d; ", 
				TRANSF_IMPED_TO_USER_I(calib_re[0]), TRANSF_IMPED_TO_USER_M(calib_re[0]));
		if (calib_re[1] == CALIB_FAILED)
			ret += scnprintf(str+ret, STR_SZ_TAS-ret, "Channel[1] = 0x%x;\n", calib_re[1]);
		else
			ret += scnprintf(str+ret, STR_SZ_TAS-ret, "Channel[1] = %02d.%02d;\n",
				TRANSF_IMPED_TO_USER_I(calib_re[1]), TRANSF_IMPED_TO_USER_M(calib_re[1]));	
	} else {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x;\n", calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %02d.%02d;\n", 
				TRANSF_IMPED_TO_USER_I(calib_re[0]), TRANSF_IMPED_TO_USER_M(calib_re[0]));	
	}
	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	pr_info("[SmartPA-%d]debugfs count %d, ret_count %d, ret %d\n",
			__LINE__, (int)count, (int)ret_count, (int)ret);
	kfree(str);

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM,
										nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: decalib init\n", __LINE__);

end:
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: end\n", __LINE__);
	
	if (ret < 0) {
		return ret;
	} else {
		return ret_count;
	}
}

static const struct file_operations smartpa_dbgfs_calibrate_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_calibrate_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_impedance_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_impedance_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_impedance_read,
	.llseek = default_llseek,
};

static ssize_t smartpa_dbgfs_f0_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS];
	uint32_t F0[MAX_CHANNELS], Q[MAX_CHANNELS];
	int ret = 0, ret_count = 0;
	uint32_t data;
	mm_segment_t fs;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	char *str;
	struct file *fp = NULL;
	loff_t pos;
	uint8_t iter = 0, channels = 1;
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = s_smartpa_algo->mn_channels;
	
	//Load Calib
	if (smartpa_check_re()) {
		calibRe[0] = s_smartpa_algo->calibRe[0];
		if (channels == 2)
			calibRe[1] = s_smartpa_algo->calibRe[1];
		smartpa_set_re(calibRe);
	}
	
	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM,
					nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto end;
	}
	//wait 5s
	msleep(5000);
	//read F0
	for (iter = 0; iter < channels; iter++)	{
		data = 0;//Resets data to 0
		paramid = (TAS_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM, /*length **/ 4);
		#else
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM
				, /*length **/ 4, AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto end;
		F0[iter] = data;
		pr_err("[SmartPA-%d]: F0[%d] is %d\n", __LINE__, iter, F0[iter]);
	}

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc \n", __LINE__);
		ret = -ENOMEM;
		goto deinit;
	}
	ret = 0;
	if (channels == 2) {
		ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %d; Channel[1] = %d;\n", (F0[0] >> 19), (F0[1] >> 19));
	} else
		ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %d;\n", (F0[0] >> 19));
	
	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	kfree(str);

	//read Q
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Reset data to 0
		paramid = (TAS_SA_GET_Q | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM, /*length **/ 4);
		#else
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM,
				/*length **/ 4, AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto end;
		Q[iter] = data;
		pr_err("[SmartPA-%d]: Q[%d] is %d\n", __LINE__, iter, Q[iter]);
	}
	
	//write to file
	fp = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0644);
	if (fp > 0) {
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		nSize = vfs_write(fp, (char *)&F0[0], sizeof(uint32_t), &pos);
		nSize = vfs_write(fp, (char *)&Q[0], sizeof(uint32_t), &pos);
		pr_info("[SmartPA-%d] write to file channel[0], F0 = %d, Q = %d\n", __LINE__, F0[0], Q[0]);
		if (channels == 2) {
			nSize = vfs_write(fp, (char *)&F0[1], sizeof(uint32_t), &pos);
			nSize = vfs_write(fp, (char *)&Q[1], sizeof(uint32_t), &pos);
			pr_info("[SmartPA-%d] write to file channel[1], F0 = %d, Q = %d\n", __LINE__, F0[1], Q[1]);
		}
		filp_close(fp, NULL);
		set_fs(fs);
	}

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
		if (ret < 0)
			goto end;	
	}	
end:
	if (ret < 0) {
		return ret;
	} else {
		return ret_count;
	}
}

static const struct file_operations smartpa_dbgfs_f0_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_f0_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_temperature_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_temperature_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_temperature_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_QFactor_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_QFactor_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_QFactor_read,
	.llseek = default_llseek,
};

static void smartpa_debug_init(smartpa_algo_data_t *algo)
{
	char name[60];

	struct i2c_client *i2c = algo->i2c_client;
	if (i2c == NULL) {
		scnprintf(name, MAX_CONTROL_NAME, "audio-tismartpa");	
	} else {
		scnprintf(name, MAX_CONTROL_NAME, "audio-%s", i2c->name);
	}
	
	algo->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("calibrate", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_calibrate_fops);
	debugfs_create_file("impedance", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_impedance_fops);
	debugfs_create_file("f0detect", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_f0_fops);
	debugfs_create_file("QFactor", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_QFactor_fops);
	debugfs_create_file("temperature", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_temperature_fops);
	debugfs_create_file("i2c", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_i2c_fops);
	debugfs_create_file("reg", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_register_fops);

}

static void smartpa_debug_remove(smartpa_algo_data_t *algo)
{
	if (algo->dbg_dir)
		debugfs_remove_recursive(algo->dbg_dir);
}

#endif //CONFIG_DEBUG_FS

static int smartpa_calib_save(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	uint8_t channels  = 1;
	
	if (!s_smartpa_algo || !calib_value) {
		pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n", __LINE__);
		return -1;
	}
	channels = s_smartpa_algo->mn_channels;
	

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[0]=%d \n", __LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[1]=%d \n", __LINE__, calib_value[1]);
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*channels, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]smartpa_calib_save: %s open failed! \n", __LINE__, CALIBRATE_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}

int smartpa_init_dbg(char *buffer, int size)
{
	uint32_t calib_re[MAX_CHANNELS] = {0};
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	bool done[MAX_CHANNELS] = {false};
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return -1;
	}
	channels = s_smartpa_algo->mn_channels;
	if (channels == 1)
		done[1] = true;
	
	if (1) { //s_smartpa_algo->mb_power_up) {
		//calib init
		for (iter = 0; iter < channels; iter++) {
			data = 1;//Value is ignored
			paramid = ((TAS_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
			#ifdef CONFIG_TAS256x_FOR_MTK
			ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
			#else
			ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
							TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
			#endif
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg:set calib_data error.\n", __LINE__);
				ret = -ENOMEM;
			}
		}
		pr_info("[SmartPA-%d]init_dbg: calib init\n", __LINE__);

		msleep(3 * 1000);

		//get Re
		for (iter = 0; iter < channels; iter++) {
			data = 0;//Reset data to 0
			paramid = ((TAS_SA_GET_RE)|((iter+1)<<24)|(1<<16));
			#ifdef CONFIG_TAS256x_FOR_MTK
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4);
			#else
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4, AFE_SMARTAMP_MODULE_RX);
			#endif
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
			} else {
				calib_re[iter] = data;

				if ((calib_re[iter] < s_smartpa_algo->imped_min[iter]) || (calib_re[iter] > s_smartpa_algo->imped_max[iter]))
					done[iter] = false;
				else
					done[iter] = true;
				pr_info("[SmartPA-%d]init_dbg: calib_re is %d, valid range (%d %d)\n",
						__LINE__, calib_re[iter], s_smartpa_algo->imped_min[iter], s_smartpa_algo->imped_max[iter]);
			}
		}
	} else {
		done[0] = false;
		if (channels == 2)
			done[1] = false;
		ret = -EINVAL;
		pr_info("[SmartPA-%d]dbg init: failed to calibrate %d\n", __LINE__, ret);
	}

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (channels == 1) ? "Mono" : "Stereo"); 
	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer + n, size - n,
			"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n", iter,
			TRANSF_IMPED_TO_USER_I(calib_re[iter]), TRANSF_IMPED_TO_USER_M(calib_re[iter]),
			TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_min[iter]), TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_min[iter]),
			TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_max[iter]), TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_max[iter]));
		pr_info("[SmartPA-%d]init_dbg: calibRe[%d] %d\n", __LINE__, iter, calib_re[iter]);
		if (!done[iter]) {
			calib_re[iter] = CALIB_FAILED;
		}
	}
	n += scnprintf(buffer + n, size - n, "\n Calibrate result: %s\n", (done[0] && done[1]) ? "OKAY(impedance ok)." : "ERROR!");
	buffer[n] = 0;

	pr_info("[SmartPA-%d]init_dbg: write to file\n", __LINE__);

	s_smartpa_algo->calibRe[0] = calib_re[0];
	s_smartpa_algo->calibRe[1] = calib_re[1];
	pr_info("[SmartPA-%d]init_dbg: update Re value\n", __LINE__);
	smartpa_calib_save(calib_re);

//deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid  = ((TAS_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
		pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
	}
//end:
	pr_info("[SmartPA-%d]init_dbg: end\n", __LINE__);

	if (done[0] && done[1])
		return 0;
	else
		return -1;
}


/* =============================================================================== */
/*                                     wangkai add                                 */
/* =============================================================================== */
int smartpa_calib_save_v1(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	int i = 0;
	if (!calib_value) {
		pr_err("[SmartPA]: param is NULL\n");
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (IS_ERR_OR_NULL(pfile)) {
		pr_err("[SmartPA] smartpa_calib_save: %s open or create failed! \n", CALIBRATE_FILE);
		ret = -1;
	} else {
		for (i = 0; i < CHANNEL_NUMS; i++) {
			pr_info("[SmartPA] smartpa_calib_save: save calib_value[%d]=%u \n", i, calib_value[i]);
		}
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNEL_NUMS, &pos);
		filp_close(pfile, NULL);
	}
	set_fs(old_fs);
	return ret;
}

int smartpa_calib_get_v1(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	int i = 0;

	if (!calib_value) {
		pr_err("[SmartPA] there is no space to store file info\n");
		return -1;
	}

	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(pfile)) {
		pr_err("[SmartPA] calibrate: %s not found\n", CALIBRATE_FILE);
		found = 0;
	} else {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNEL_NUMS, &pos);
		for (i = 0; i < CHANNEL_NUMS; i++) {
			pr_info("[SmartPA] calibrate:get calib_value[%d] = %u  \n", i, calib_value[i]);
		}
		filp_close(pfile, NULL);
	}
	set_fs(old_fs);
	return found;
}

void parse_calib_re_v1(uint32_t *calib_re, int n)
{
	struct smartpa_para *priv = &smartpa;
	uint32_t re_15 = calib_re[n];

	if (!priv || !calib_re || n < 0 || n >= CHANNEL_NUMS) {
		pr_err("[SmartPA] calibrate: param is invalid\n");
	}

	if (re_15 == 0 || (re_15 >= priv->imped_min && re_15 <= priv->imped_max)) {
		priv->Calibration_Success = 1;
		priv->Re_Q15 = re_15;
	} else {
		priv->Calibration_Success = 0;
		priv->Re_Q15 = CALIB_FAILED;
	}
}

void vivo_adsp_send_params_v1(void)
{
	struct smartpa_para *priv = &smartpa;
	uint32_t paramid = 0;
	struct CALIBRATION_RX_ calInfo;
	uint32_t calib_re[CHANNEL_NUMS] = {0};
	int iter = 0;
	int ret = 0;

	if (update_cali_flag == false)
		smartpa_calib_get_v1(calib_re);

	if (priv == NULL) {
		pr_err("[SmartPA] smartPA's smartpa_para is invalid\n");
	} else {
		if (update_cali_flag == false) {
			parse_calib_re_v1(calib_re, iter);
		}

		pr_info("[smartPA] %s: pass cali parameter to DSP\n", __func__);
		pr_info("[smartPA]========================================\n");
		pr_info("[smartPA-%d] calInfo.Re_Q15 = %u\n", iter, priv->Re_Q15);
		pr_info("[smartPA-%d] calInfo.f0_Q15 = %u\n", iter, priv->f0_Q15);
		pr_info("[smartPA-%d] calInfo.Qts_Q15 = %u\n", iter, priv->Qts_Q15);
		pr_info("[smartPA-%d] calInfo.Calibration_Success = %u\n", iter, priv->Calibration_Success);
		pr_info("[smartPA-%d] calInfo.PA_ID = %u\n", iter, priv->PA_ID);
		pr_info("[smartPA-%d] calInfo.V_MAX_Q15 = %u\n", iter, priv->vmax);
		pr_info("[smartPA-%d] calInfo.I_MAX_Q15 = %u\n", iter, priv->imax);
		pr_info("[smartPA-%d] calInfo.V_OUT_MAX_Q15 = %u\n", iter, priv->voutmax);
		pr_info("[smartPA]========================================\n");

		calInfo.Calibration_Success = priv->Calibration_Success;
		calInfo.f0_Q15 = priv->f0_Q15;
		calInfo.I_MAX_Q15 = priv->imax;
		calInfo.PA_ID = priv->PA_ID;
		calInfo.Qts_Q15 = priv->Qts_Q15;
		calInfo.Re_Q15 = priv->Re_Q15;
		calInfo.V_MAX_Q15 = priv->vmax;
		calInfo.V_OUT_MAX_Q15 = priv->voutmax;

		paramid = ((AFE_SA_SEND_ALL)|((iter+1)<<24)|((iter+1)<<16));
		ret = afe_smartamp_algo_ctrl_v1((void *)&calInfo, paramid, AP_2_DSP_SEND_PARAM, sizeof(struct CALIBRATION_RX_));
		if (ret < 0) {
			pr_err("[smartPA-%s-%d]: afe_smartamp_algo_ctrl dsp param send error!\n", __func__, iter);
		} else {
			pr_info("[smartPA-%s-%d]: afe_smartamp_algo_ctrl dsp param send successful!\n", __func__, iter);
		}
	}

	update_cali_flag == true;
}


int smartpa_init_dbg_v1(char *buffer, int size)
{
	int  n = 0;
	struct smartpa_para *priv = &smartpa;
	int iter = 0;
	uint32_t paramid = 0;
	uint32_t data = 0;
	uint32_t flag = 0;
	bool done[CHANNEL_NUMS] = {false};
	int ret = 0;
	int nSize = sizeof(uint32_t);
	uint32_t calib_re[CHANNEL_NUMS] = {0};

	pr_info("[SmartPA]: enter======> %s\n", __func__);

	if (priv == NULL) {
		pr_err("[SmartPA-%d] smartPA's private pointer is invalid\n", iter);
		done[iter] = false;
	} else {
		data = 1;
		paramid = ((AFE_SA_CALIB_INIT)|((iter+1)<<24)|((iter+1)<<16)); // 16846566
		pr_info("[SmartPA] now startup cali for smartPA-%d\n", iter);
		ret = afe_smartamp_algo_ctrl_v1((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
		if (ret < 0) {
			done[iter] = false;
			pr_err("[SmartPA] %s: startup cali for smartPA-%d error\n", __func__, iter);
		} else {
			pr_info("[SmartPA] %s: startup cali for smartPA-%d successful\n", __func__, iter);
			done[iter] = true;
		}
	}

	msleep(3000);

	if (done[iter] == false) {
		priv->Calibration_Success = 1;
		priv->Re_Q15 = 0;
	} else {
		data = 0;
		paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|((iter+1)<<16)); //16846565
		pr_info("[SmartPA] now get calibrate result for smartPA-%d\n", iter);
		ret = afe_smartamp_algo_ctrl_v1((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, 4);
		if (ret < 0) {
			pr_err("[SmartPA] get calibrate result for smartPA-%d error\n", iter);
			done[iter] = false;
			priv->Calibration_Success = 0;
			priv->Re_Q15 = CALIB_FAILED;
		} else {
			if (data < priv->imped_min || data > priv->imped_max) {
				pr_err("[SmartPA] %s: calibrate result for smartPA-%d is invalid\n", __func__, iter);
				done[iter] = false;
				priv->Calibration_Success = 0;
				priv->Re_Q15 = CALIB_FAILED;
			} else {
				pr_info("[SmartPA] %s: get calibrate result for smartPA-%d successful, value is %u\n", __func__, iter, data);
				priv->Calibration_Success = 1;
				priv->Re_Q15 = data;
			}
		}
	}

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (CHANNEL_NUMS == 1) ? "Mono" : "Stereo");

	if (priv->Re_Q15 != CALIB_FAILED) {
		n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %d.%02d ohm, ", iter,
			TRANSF_IMPED_TO_USER_I_V1(priv->Re_Q15), TRANSF_IMPED_TO_USER_M_V1(priv->Re_Q15));
	} else {
		n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %#X ohm, ", iter, CALIB_FAILED);
	}

	n += scnprintf(buffer + n, size - n, ", valid range(%d.%02d ~ %d.%02d ohm).\n",
		TRANSF_IMPED_TO_USER_I_V1(priv->imped_min), TRANSF_IMPED_TO_USER_M_V1(priv->imped_min),
		TRANSF_IMPED_TO_USER_I_V1(priv->imped_max), TRANSF_IMPED_TO_USER_M_V1(priv->imped_max));

	if (done[iter] == true)
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "OKAY(impedance ok).\n");
	else
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "ERROR(impedance error).\n");

	buffer[n] = 0;

	data = priv->Re_Q15;
	paramid = 17043176; //send Re
	pr_info("[SmartPA] now start send Re for smartPA-%d!\n", iter);
	ret = afe_smartamp_algo_ctrl_v1((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		pr_err("[SmartPA] send Re for smartPA-%d error!\n", iter);
	} else {
		pr_info("[SmartPA] send Re for smartPA-%d successful!\n", iter);
	}

	flag = priv->Calibration_Success;
	paramid = ((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|((iter+1)<<16)); //16846567
	pr_info("[SmartPA] now start send calibration_Success flag for smartPA-%d\n", iter);
	ret = afe_smartamp_algo_ctrl_v1((uint8_t *)&flag, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		pr_err("[SmartPA] send calibration_Success flag for smartPA-%d error\n", iter);
	} else {
		pr_info("[SmartPA] send calibration_Success flag for smartPA-%d successful\n", iter);
	}

	calib_re[iter] = priv->Re_Q15;

	ret = smartpa_calib_save_v1(calib_re);
	update_cali_flag = false;

	if (ret < 0) {
		pr_err("[SmartPA] store calibration result error\n");
	} else {
		pr_info("[SmartPA] store calibration result successful\n");
	}
	pr_info("[SmartPA]: leave<====== %s\n", __func__);

	if (done[iter] == false)
		return -1;

	return 0;
}

static int smartpa_freq_save_v1(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA]freq: save count=%d \n", __func__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __func__, FREQ_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}


int smartpa_read_freq_dbg_v1(char *buffer, int size)
{
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	int nSize = sizeof(uint32_t);
	int iter = 0;
	bool done[CHANNEL_NUMS] = {false};
	struct smartpa_para *priv = &smartpa;
	int i[CHANNEL_NUMS] = {0}, j[CHANNEL_NUMS] = {0};
	uint32_t calib_re[CHANNEL_NUMS] = {0};

	pr_info("[SmartPA]: enter======> %s\n", __func__);

	ret = smartpa_calib_get_v1(calib_re);
	if (ret < 0) {
		pr_err("[SmartPA] get calibration result error\n");
	} else {
		pr_info("[SmartPA] get calibration result successful\n");
	}

	if (priv == NULL) {
		pr_err("[SmartPA-%d] smartPA's private pointer is invalid\n", iter);
		done[iter] = false;
	} else {
		parse_calib_re_v1(calib_re, iter);

		data = 1;
		paramid = ((AFE_SA_F0_TEST_INIT) | (iter << 16) | ((iter+1) << 24));
		pr_info("[SmartPA] now startup freq for smartPA-%d\n", iter);
		ret = afe_smartamp_algo_ctrl_v1((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
		if (ret < 0) {
			pr_err("[SmartPA] startup freq for smartPA-%d error\n", iter);
			done[iter] = false;
		} else {
			pr_info("[SmartPA] startup freq for smartPA-%d successful\n", iter);
			done[iter] = true;
		}
	}

	msleep(5000);

	if (done[iter] == true) {
		n += scnprintf(buffer+n, size-n, "Ch[%d] ", iter);
		n += scnprintf(buffer+n, size-n, "Rdc = %d.%02d\n",
				TRANSF_IMPED_TO_USER_I_V1(priv->Re_Q15),
				TRANSF_IMPED_TO_USER_M_V1(priv->Re_Q15));
		while ((i[iter]++ < 5) && (j[iter] < 3)) {
			data = 0;
			paramid = (AFE_SA_GET_F0 | ((iter+1) << 16) | ((iter+1) << 24));
			pr_info("[SmartPA] now get F0 result for smartPA-%d\n", iter);
			ret = afe_smartamp_algo_ctrl_v1((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, nSize);
			if (ret < 0) {
				pr_err("[SmartPA] get F0 result for smartPA-%d error\n", iter);
			} else {
				pr_info("[SmartPA] get F0 result for smartPA-%d successful\n", iter);
			}

			priv->f0_Q15 = data;
			if (priv->f0_Q15 < priv->fres_min || priv->f0_Q15 > priv->fres_max) {
				pr_err("[SmartPA] F0 result for smartPA-%d is invalid\n", iter);
			} else {
				pr_err("[SmartPA] F0 result for smartPA-%d is OK\n", iter);
			}

			data = 0;
			paramid = (AFE_SA_GET_Q | ((iter+1) << 16) | ((iter+1) << 24));
			pr_info("[SmartPA] now get Q result for smartPA-%d\n", iter);
			ret = afe_smartamp_algo_ctrl_v1((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, nSize);
			if (ret < 0) {
				pr_err("[SmartPA] get Q result for smartPA-%d error\n", iter);
			} else {
				pr_info("[SmartPA] get Q result for smartPA-%d successful\n", iter);
			}

			priv->Qts_Q15 = data;

			if (priv->Qts_Q15 < priv->Qt) {
				pr_err("[SmartPA] Q result for smartPA-%d is invalid\n", iter);
			} else {
				pr_info("[SmartPA] Q result for smartPA-%d is OK\n", iter);
			}

			if (priv->f0_Q15 < priv->fres_min || priv->f0_Q15 > priv->fres_max ||
					priv->Qts_Q15 < priv->Qt)
				j[iter] = 0;
			else
				j[iter]++;

			pr_info("[SmartPA] read freq dbg channel[%d]: f0 = %u Q = %u i = %d j = %d\n", 
						iter, priv->f0_Q15, priv->Qts_Q15, i[iter], j[iter]);

			n += scnprintf(buffer+n, size-n, "f0: %d.%02d Qt = %d.%02d\n",
				TRANSF_IMPED_TO_USER_I_V1(priv->f0_Q15),
				TRANSF_IMPED_TO_USER_M_V1(priv->f0_Q15),
				TRANSF_IMPED_TO_USER_I_V1(priv->Qts_Q15),
				TRANSF_IMPED_TO_USER_M_V1(priv->Qts_Q15));

			msleep(500);
		}

		n += scnprintf(buffer+n, size-n, "f0 (%d~%d)\nQ_Min: %d.%2d \n",
					   TRANSF_IMPED_TO_USER_I_V1(priv->fres_min),
					   TRANSF_IMPED_TO_USER_I_V1(priv->fres_max),
					   TRANSF_IMPED_TO_USER_I_V1(priv->Qt),
					   TRANSF_IMPED_TO_USER_M_V1(priv->Qt));
		if (j[iter] == 3) {
			n += scnprintf(buffer+n, size-n, "PASS\n");
		} else {
			n += scnprintf(buffer+n, size-n, "FAIL\n");
		}
	}

	buffer[n] = 0;
	ret = smartpa_freq_save_v1(buffer, n+1);

	pr_info("[SmartPA]: leave<======%s\n", __func__);
	return 0;
}

static int smartpa_freq_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d \n", __LINE__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __LINE__, FREQ_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS] = {0};
	uint32_t F0[MAX_CHANNELS] = {0}, Q[MAX_CHANNELS] = {0};
	int ret = 0, n = 0, i[MAX_CHANNELS] = {0}, j[MAX_CHANNELS] = {0};
	uint32_t data = 0;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;
	
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return -1;
	}
	channels = s_smartpa_algo->mn_channels;
	
	//Load Calib
	if (smartpa_check_re()) {
		
		for (iter = 0; iter < channels; iter++) {
			calibRe[iter] = s_smartpa_algo->calibRe[iter];
		}
		smartpa_set_re(calibRe);
	}
	
	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
	}
	//wait 5s
	msleep(5000);

	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer+n, size-n, "Ch[%d] ", iter);
		if (calibRe[iter] == CALIB_FAILED)
			n += scnprintf(buffer+n, size-n, "Rdc = %x\n", calibRe[iter]);
		else
			n += scnprintf(buffer+n, size-n, "Rdc = %02d.%02d\n", 
				TRANSF_IMPED_TO_USER_I(calibRe[iter]), TRANSF_IMPED_TO_USER_M(calibRe[iter]));
		while ((i[iter]++ < 5) && (j[iter] < 3)) {
			//read F0
			data = 0;//Reset data to 0
			paramid = (TAS_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
			#ifdef CONFIG_TAS256x_FOR_MTK
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4);
			#else
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE_RX);
			#endif
			F0[iter] = data;
	
			//read Q
			data = 0;//Reset data to 0
			paramid = (TAS_SA_GET_Q | (length << 16) | ((iter+1) << 24));
			#ifdef CONFIG_TAS256x_FOR_MTK
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4);
			#else
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4, AFE_SMARTAMP_MODULE_RX);
			#endif
			Q[iter] = data;

			if (((F0[iter] >> 19) < s_smartpa_algo->fres_min[iter]) || ((F0[iter] >> 19) > s_smartpa_algo->fres_max[iter])
				|| (((Q[iter] * 100) >> 19) < s_smartpa_algo->Qt[iter]))
				j[iter] = 0;
			else
				j[iter]++;

			pr_info("[SmartPA-%d]read freq dbg channel[%d]: f0 = %d Q = %d i = %d j = %d\n",
					__LINE__, iter, F0[iter], Q[iter], i[iter], j[iter]);
			n += scnprintf(buffer+n, size-n, "f0 = %d Qt = %d.%d\n",
				(F0[iter] >> 19), TRANSF_IMPED_TO_USER_I(Q[iter]), TRANSF_IMPED_TO_USER_M(Q[iter]));
			msleep(500);
		}
		n += scnprintf(buffer+n, size-n, "f0 (%d~%d)\nQt_Min: %d.%d \n",
					   s_smartpa_algo->fres_min[iter], s_smartpa_algo->fres_max[iter],
					   s_smartpa_algo->Qt[iter] / 100, s_smartpa_algo->Qt[iter] % 100);
		if (j[iter] == 3)
			n += scnprintf(buffer+n, size-n, "PASS\n");
		else
			n += scnprintf(buffer+n, size-n, "FAIL\n");
	}

	ret = smartpa_freq_save(buffer, n);
	buffer[n] = 0;

//deinit :
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
		#ifdef CONFIG_TAS256x_FOR_MTK
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		#else
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize, AFE_SMARTAMP_MODULE_RX);
		#endif
	}
//end:
	return 0;
}

void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	return ;
}


/* This should be called first */
int smaramp_set_i2c_client(struct i2c_client *i2c)
{
	int ret = alloc_memory_for_smartpa_algo_client();
	if (ret)
		return ret;

	s_smartpa_algo->i2c_client = i2c;
	return 0;
}

void smartpa_get_client(struct i2c_client **client, unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	if (s_smartpa_algo) {
		*client = s_smartpa_algo->i2c_client;
	}
	return ;
}

int smartpa_check_calib_dbg(void)
{
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	int ret = 1;

	if (!s_smartpa_algo)
		return 0;

	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	smartpa_calib_get(impedance);
	channels = s_smartpa_algo->mn_channels;
	for (iter = 0; iter < channels; iter++) {
		ret &= rdc_check_valid(impedance[iter], iter);
	}
	return ret;
}



int tas25xx_parse_algo_dt(struct i2c_client *i2c)
{
	uint32_t temp = 0;
	int ret = 0;

	struct smartpa_para *priv = &smartpa;

	pr_info("[SmartPA] %s : enter ===>\n", __func__);

	if (!s_smartpa_algo || !priv) {
		pr_err("[SmartPA-%d]: s_smartpa_algo or smartpa_para is NULL\n", __LINE__);
		return -ENOMEM;
	}

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-min", &temp);
	s_smartpa_algo->imped_min[0] = (!ret) ? temp : RDC_MIN_L;
	pr_err("[SmartPA-%d]: s_smartpa_algo s_smartpa_algo->imped_min[0] %d\n", __LINE__, s_smartpa_algo->imped_min[0]);

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-max", &temp);
	s_smartpa_algo->imped_max[0] = (!ret) ? temp : RDC_MAX_L;
	pr_err("[SmartPA-%d]: s_smartpa_algo s_smartpa_algo->imped_max[0] %d\n", __LINE__, s_smartpa_algo->imped_max[0]);

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-min", &temp);
	s_smartpa_algo->fres_min[0] = (!ret) ? temp : 500;
	pr_err("[SmartPA-%d]: s_smartpa_algo s_smartpa_algo->fres_min[0] %d\n", __LINE__, s_smartpa_algo->fres_min[0]);

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-max", &temp);
	s_smartpa_algo->fres_max[0] = (!ret) ? temp : 1100;
	pr_err("[SmartPA-%d]: s_smartpa_algo s_smartpa_algo->fres_max[0] %d\n", __LINE__, s_smartpa_algo->fres_max[0]);

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-min", &temp);
	s_smartpa_algo->Qt[0] = (!ret) ? temp : 100;
	pr_err("[SmartPA-%d]: s_smartpa_algo s_smartpa_algo->Qt[0] %d\n", __LINE__, s_smartpa_algo->Qt[0]);

	if (s_smartpa_algo->mn_channels == 2) {
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-min", &temp);
		s_smartpa_algo->imped_min[1] = (!ret) ? temp : RDC_MIN_R;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-max", &temp);
		s_smartpa_algo->imped_max[1] = (!ret) ? temp : RDC_MAX_R;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-min", &temp);
		s_smartpa_algo->fres_min[1] = (!ret) ? temp : 500;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-max", &temp);
		s_smartpa_algo->fres_max[1] = (!ret) ? temp : 1100;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-min", &temp);
		s_smartpa_algo->Qt[1] = (!ret) ? temp : 100;
	}

	priv->imped_min = 163840;
	priv->imped_max = 327680;
	priv->fres_min = 16384000;
	priv->fres_max = 32768000;
	priv->Qt = 32768;
	priv->PA_ID = 62;
	priv->vmax = 458752;
	priv->imax = 65536;
	priv->voutmax = 458752;
	priv->Calibration_Success = 1;
	priv->f0_Q15 = 0;
	priv->Qts_Q15 = 0;
	priv->Re_Q15 = 0;

	return 0;
}


void tas25xx_send_algo_calibration(void)
{
	if (smartpa_check_re ()) {
		smartpa_set_re (s_smartpa_algo->calibRe);
		pr_info("[SmartPA-%d] SetRe[0] called %d(0x%x)", __LINE__
							, s_smartpa_algo->calibRe[0], s_smartpa_algo->calibRe[0]);
		if (s_smartpa_algo->mn_channels == 2)
			pr_info("[SmartPA-%d] SetRe[1] called %d(0x%x)", __LINE__
							, s_smartpa_algo->calibRe[1], s_smartpa_algo->calibRe[1]);
	} else {
		pr_err("[SmartPA-%d] SetRe is not called", __LINE__);
	}
}

static int alloc_memory_for_smartpa_algo_client(void)
{
	if (!s_smartpa_algo) {
		int size = sizeof(smartpa_algo_data_t);
		s_smartpa_algo = kmalloc(size, GFP_KERNEL);
		memset (s_smartpa_algo, 0, sizeof(smartpa_algo_data_t));
	}

	if (!s_smartpa_algo)
		return -ENOMEM;

	return 0;
}

int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_codec *c, int number_of_channels)
{
	int ret = alloc_memory_for_smartpa_algo_client();
	if (ret)
		return ret;

	s_smartpa_algo->mn_channels = number_of_channels;

#ifdef CONFIG_DEBUG_FS
	smartpa_debug_init(s_smartpa_algo);
#endif

	return 0;
}

void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_codec *c)
{
	if (s_smartpa_algo) {
#ifdef CONFIG_DEBUG_FS
		smartpa_debug_remove(s_smartpa_algo);
#endif

		kfree (s_smartpa_algo);
		s_smartpa_algo = NULL;
	}
}
