#ifndef __SMARTPA_DEGUG_COMMON_H__
#define __SMARTPA_DEGUG_COMMON_H__

//used for communicate with CODEC's DSP
typedef struct CALIBRATION_RX_{
	int32_t Re_Q15;
	int32_t f0_Q15;
	int32_t Qts_Q15;
	int32_t Calibration_Success;
	int32_t PA_ID;//hex format
	int32_t V_MAX_Q15;//V sense full scale
	int32_t I_MAX_Q15;//I sense full scale
	int32_t V_OUT_MAX_Q15;//output max voltage
} CALIBRATION_RX_;

//used for internel
struct smartpa_para {
	uint32_t PA_ID;//vendor ID
	uint32_t imped_min; // read from dts
	uint32_t imped_max; // read from dts
	uint32_t fres_min; // read from dts
	uint32_t fres_max; // read from dts
	uint32_t Qt; // read from dts
	uint32_t Re_Q15; //calculate Result
	uint32_t f0_Q15; //calculate Result
	uint32_t Qts_Q15; //calculate Result
	uint32_t Calibration_Success;
	uint32_t vmax; // read from dts
	uint32_t imax; // read from dts
	uint32_t voutmax; // read from dts
};

#define CHANNEL_NUMS 1
#define TRANSF_IMPED_TO_USER_I_V1(X) (((X * 100) >> 15) / 100)
#define TRANSF_IMPED_TO_USER_M_V1(X) (((X * 100) >> 15) % 100)

#endif
