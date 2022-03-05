#ifndef __AW87XXX_MNT_H__
#define __AW87XXX_MNT_H__


/********************************************
*
* vmax of matching about capacity
*
********************************************/
static const char aw87xxx_vmax_cfg_name[] = {"aw87xxx_vmax.bin"};

enum aw87xxx_mnt_flag {
	AW87XXX_MNT_DISABLE = 0,
	AW87XXX_MNT_ENABLE = 1,
};

struct vmax_single_config {
	uint32_t min_thr;
	uint32_t vmax;
};

struct vmax_config {
	int vmax_cfg_num;
	struct vmax_single_config vmax_cfg_total[];
};

struct aw87xxx_mnt {
	uint8_t cfg_update_flag;
	uint8_t update_num;
	uint32_t mnt_flag;
	struct vmax_config *vmax_cfg;
};

/**********************************************************
 * aw87xxx mnt function
***********************************************************/
int aw87xxx_mnt_stop(struct aw87xxx_mnt *mnt);
int aw87xxx_mnt_start(struct aw87xxx_mnt *mnt);
void aw87xxx_mnt_init(struct aw87xxx_mnt *mnt);
void aw87xxx_parse_mnt_dt(struct aw87xxx_mnt *mnt);


#endif

