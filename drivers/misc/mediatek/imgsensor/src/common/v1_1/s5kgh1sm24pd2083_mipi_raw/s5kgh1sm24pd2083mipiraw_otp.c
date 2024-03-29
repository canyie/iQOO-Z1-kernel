#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "s5kgh1sm24pd2083mipiraw_Sensor.h"

#define PFX "SUB[F881]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x34AB
  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
//#define AF_RANGE_GOT  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA2
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x34AA
/*#define VIVO_VENDOR_SUNNY 0x01*/
/*#define VIVO_VENDOR_TRULY 0x02*/
/*#define VIVO_VENDOR_QTECH 0x05*/
#define VIVO_VENDOR_OFILM 0x08
#define VIVO_VENDOR_LENS_ID 0x07
#define VIVO_VENDOR_VCM_ID 0x10
#define VIVO_VENDOR_DRIVERIC_ID 0x02
#define VIVO_VENDOR_PLATFORM_ID 0x03

unsigned char vivo_otp_data_s5kgh1sm24pd2083[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int Fulse_id_addr = 0x0020;
static unsigned const int Fulse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x005f;
static unsigned const int Awb_addr = 0x0060;
static unsigned const int Awb_checksum_addr = 0x009F;
static unsigned const int Lsc_addr = 0x0D20;
static unsigned const int Lsc_checksum_addr = 0x146F;
static unsigned const int PDAF_proc1_addr = 0x1470;
static unsigned const int PDAF_proc1_checksum_addr = 0x1661;
static unsigned const int PDAF_proc2_addr = 0x1662;
static unsigned const int PDAF_proc2_checksum_addr = 0x1A4F;
static unsigned const int XTC_cal_addr = 0x1A9D;
static unsigned const int XTC_cal_checksum_addr = 0x2EE8;
static unsigned const int PDXTC_addr = 0x2EE9;
static unsigned const int PDXTC_checksum_addr = 0x33D1;
static unsigned const int OIS_Process_addr =  0x33D2;
static unsigned const int OIS_Process_checksum_addr =0x3493;
static unsigned const int Af_addr = 0x3494;
static unsigned const int Af_checksum_addr = 0x34AA;

#if 0
static unsigned const int Coefficient_of_light_source_addr = 0x19E0;
static unsigned const int Coefficient_of_light_source_checksum_addr = 0x19ED;
static unsigned const int PDAF_proc2_v3_8_addr = 0x1A01;
static unsigned const int PDAF_proc2_v3_8_checksum_addr = 0x1DFF;
#endif

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_sub_s5kgh1sm24pd2083[13];
extern MUINT32  material_inf_sub_s5kgh1sm24pd2083[4];  

static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
	if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		pr_err("addr > VIVO_MAX_OFFSET 0x%x > 0x%x \n", addr,  VIVO_MAX_OFFSET);
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);
	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		pr_err("iReadRegI2C  failed \n");
		return false;
	}
    return true;
}
//extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN_F881_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	#if 0
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	if (is_atboot == 1) {
		LOG_INF("[zxw++]AT mode skip s5kgh1sm24pd2083_otp_read\n");
		return 1;
	}
	/* guojunzheng add end */
	#endif
	
	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset,  &vivo_otp_data_s5kgh1sm24pd2083[i])) {
			LOG_INF("[zxw++]read_vivo_eeprom 0x%0x %d fail \n", offset,  vivo_otp_data_s5kgh1sm24pd2083[i]);
			return 0;
		}
		/*LOG_INF("[zxw++]read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data_s5kgh1sm24pd2083[i]);*/
		offset++;
	}
	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[zxw++]read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_s5kgh1sm24pd2083[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data_s5kgh1sm24pd2083[ModuleInfo_addr] | vivo_otp_data_s5kgh1sm24pd2083[Awb_addr] | vivo_otp_data_s5kgh1sm24pd2083[Af_addr] | vivo_otp_data_s5kgh1sm24pd2083[Lsc_addr] | vivo_otp_data_s5kgh1sm24pd2083[PD_Proc1_addr] | vivo_otp_data_s5kgh1sm24pd2083[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data_s5kgh1sm24pd2083[ModuleInfo_addr]) && 
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[Fulse_id_addr]) && 
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[SN_addr]) && 
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[Af_addr]) && 
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[Awb_addr]) &&
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[Lsc_addr]) &&
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc1_addr]) &&
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc2_addr])&&
		(0x01 == vivo_otp_data_s5kgh1sm24pd2083[OIS_Process_addr])) {
		
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if ((VIVO_VENDOR_OFILM != vivo_otp_data_s5kgh1sm24pd2083[0x01]) ) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[zxw++]Module ID error!!!    otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_VENDOR_LENS_ID != vivo_otp_data_s5kgh1sm24pd2083[0x08]) || 
				(VIVO_VENDOR_VCM_ID != vivo_otp_data_s5kgh1sm24pd2083[0x09]) || 
				(VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_s5kgh1sm24pd2083[0x0A]) || 
				(VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_s5kgh1sm24pd2083[0x02])) {
				
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[zxw++]: Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		} else if ((0x00 != vivo_otp_data_s5kgh1sm24pd2083[0x0B]) || 
				(0x00 != vivo_otp_data_s5kgh1sm24pd2083[0x0C]) || 
				(0x00 != vivo_otp_data_s5kgh1sm24pd2083[0x0D]) || 
				((0x00 != vivo_otp_data_s5kgh1sm24pd2083[0x0E]) && (0x03 != vivo_otp_data_s5kgh1sm24pd2083[0x0E]) && (0x02 != vivo_otp_data_s5kgh1sm24pd2083[0x0E]))) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[zxw++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", 
				vivo_otp_data_s5kgh1sm24pd2083[0x0B], vivo_otp_data_s5kgh1sm24pd2083[0x0C], vivo_otp_data_s5kgh1sm24pd2083[0x0D], vivo_otp_data_s5kgh1sm24pd2083[0x0E]);
			return 0;
		}
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[ModuleInfo_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
			}
		
		/****material info start****/
			material_inf_sub_s5kgh1sm24pd2083[0] = (MUINT32)vivo_otp_data_s5kgh1sm24pd2083[0x0F];
			material_inf_sub_s5kgh1sm24pd2083[1] = (MUINT32)vivo_otp_data_s5kgh1sm24pd2083[0x10];
			material_inf_sub_s5kgh1sm24pd2083[2] = (MUINT32)vivo_otp_data_s5kgh1sm24pd2083[0x11];
			material_inf_sub_s5kgh1sm24pd2083[3] = (MUINT32)vivo_otp_data_s5kgh1sm24pd2083[0x12];
			LOG_INF("material_inf_sub_s5kgh1sm24pd2083[0] = 0x%x, material_inf_sub_s5kgh1sm24pd2083[1] = 0x%x, material_inf_sub_s5kgh1sm24pd2083[2] = 0x%x,material_inf_sub_s5kgh1sm24pd2083[3] = 0x%x\n",
				material_inf_sub_s5kgh1sm24pd2083[0]  , material_inf_sub_s5kgh1sm24pd2083[1],  material_inf_sub_s5kgh1sm24pd2083[2], material_inf_sub_s5kgh1sm24pd2083[3]);
		/****material info end****/
		
		/****Fulse id_checksum****/
		checksum = 0;
		for (i = Fulse_id_addr+1; i < Fulse_id_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[Fulse_id_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}

		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data_s5kgh1sm24pd2083[0x%x] = 0x%x\n", i, vivo_otp_data_s5kgh1sm24pd2083[i]);*/
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[SN_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_sub_s5kgh1sm24pd2083[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_sub_s5kgh1sm24pd2083[i+1] = (MUINT32)vivo_otp_data_s5kgh1sm24pd2083[i + SN_addr+1];
			/*LOG_INF("sn_inf_sub_s5kgh1sm24pd2083[%d] = 0x%x, vivo_otp_data_s5kgh1sm24pd2083[0x%x] = 0x%x\n", i+1  , sn_inf_sub_s5kgh1sm24pd2083[i+1],  i +SN_addr+1, vivo_otp_data_s5kgh1sm24pd2083[i+SN_addr+1]);*/
		}


		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[Af_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]AF_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}

		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[Awb_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]AWB_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
			}


		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[Lsc_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]LSC_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}

		/****PDAF Proc1_checksum****/
		checksum = 0;
		for (i = PDAF_proc1_addr+1; i < PDAF_proc1_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc1_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[zxw++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF Proc2_checksum****/
		if (0x01 == vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc2_addr]){
			LOG_INF("pdaf use 3.4 tools calibration\n");
			checksum = 0;
			for (i = PDAF_proc2_addr+1; i < PDAF_proc2_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc2_checksum_addr] != checksum) {
				S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[zxw++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
				return 0;
				}
		}

		/****XTC_checksum****/
	//	if (0x01 == vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_addr]){
			LOG_INF("XTC_call \n");
			checksum = 0;
			for (i = XTC_cal_addr+1; i < XTC_cal_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
				checksum = checksum % 0xff+1;
			LOG_INF("[zxw++] vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_addr] =%d vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_checksum_addr] =%d  checksum:%d\n", vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_addr],vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_checksum_addr],checksum);
			if (vivo_otp_data_s5kgh1sm24pd2083[XTC_cal_checksum_addr] != checksum) {
				S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[zxw++]XTC_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
				return 0;
				}
//		}


		/****PDXTC_checksum****/
LOG_INF("[zxw++] vivo_otp_data_s5kgh1sm24pd2083[PDXTC_addr] =%d",vivo_otp_data_s5kgh1sm24pd2083[PDXTC_addr]);
//		if (0x01 == vivo_otp_data_s5kgh1sm24pd2083[PDXTC_addr]){
			LOG_INF("PDXTC \n");
			checksum = 0;
			for (i = PDXTC_addr+1; i < PDXTC_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
				checksum = checksum % 0xff+1;
				LOG_INF("[zxw++] vivo_otp_data_s5kgh1sm24pd2083[PDXTC_addr] =%d checksum=%d",vivo_otp_data_s5kgh1sm24pd2083[PDXTC_checksum_addr],checksum);
			if (vivo_otp_data_s5kgh1sm24pd2083[PDXTC_checksum_addr] != checksum) {
				S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[zxw++]PDXTC_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
				return 0;
				}
//		}

		/****OIS_checksum****/
		if (0x01 == vivo_otp_data_s5kgh1sm24pd2083[OIS_Process_addr]){
			LOG_INF("ois cal \n");
			checksum = 0;
			for (i = OIS_Process_addr+1; i < OIS_Process_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kgh1sm24pd2083[OIS_Process_checksum_addr] != checksum) {
				S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[zxw++]ois_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
				return 0;
				}
		}

#if 0
		if (0x01 == vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc2_v3_8_addr]){
			LOG_INF("pdaf use 3.8 tools calibration\n");
			checksum = 0;
			for (i = PDAF_proc2_v3_8_addr+1; i < PDAF_proc2_v3_8_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kgh1sm24pd2083[PDAF_proc2_v3_8_checksum_addr] != checksum) {
				S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[zxw++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
				return 0;
				}
		}


		/****Coefficient_of_light_source_checksum****/
		checksum = 0;
		for (i = Coefficient_of_light_source_addr+1; i < Coefficient_of_light_source_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgh1sm24pd2083[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgh1sm24pd2083[Coefficient_of_light_source_checksum_addr] != checksum) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Coefficient_of_light_source_checksum  error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
			}
#endif		
		/****check if awb out of range[5000K high  cct]****/
		R_unit = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+2]);
		B_unit = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+4]);
		G_unit = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+6]);

		R_golden = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+8]);
		B_golden = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+10]);
		G_golden = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("lxd_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[zxw++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}
		/****check if awb out of range[3000K low cct]****/
		R_unit_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+17];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+18]);
		B_unit_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+19];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+20]);
		G_unit_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+21];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+22]);

		R_golden_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+23];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+24]);
		B_golden_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+25];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+26]);
		G_golden_low = vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+27];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Awb_addr+28]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
			return 0;
		}

		
		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_s5kgh1sm24pd2083[Af_addr + 9];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Af_addr + 10]);

		AF_mac = vivo_otp_data_s5kgh1sm24pd2083[Af_addr + 11];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_s5kgh1sm24pd2083[Af_addr + 12]);
        
		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
        
		if (diff_inf_macro > 480 || diff_inf_macro < 200) {  /*AF code out of range*/
			S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("[zxw++]AF out of range error!!!   otp_error_code:%d    Module ID = 0x%0x   inf = %d    mac = %d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE, vivo_otp_data_s5kgh1sm24pd2083[0x01], AF_inf, AF_mac);
			return 0;
		}
		#endif

		/*lxd add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("[zxw++]read_S5KGH1SM24PD2083F_EX_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_s5kgh1sm24pd2083[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_s5kgh1sm24pd2083[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_s5kgh1sm24pd2083[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*lxd add for pdaf data end*/
		S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("[zxw++]invalid otp data. error!!!   otp_error_code:%d\n", S5KGH1SM24PD2083F_EX_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
