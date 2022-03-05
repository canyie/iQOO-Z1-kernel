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
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_MTK_FREQ_HOPPING
#include "mtk_freqhopping_drv.h"
#else
#define FH_PLL0 0
#define FH_PLL1 1
#define FH_PLL3 3
#endif

#include <mt-plat/upmu_common.h>
#include "mtk_cpufreq_platform.h"
#include "../../mtk_cpufreq_hybrid.h"
#include "mtk_devinfo.h"

static struct regulator *regulator_proc11;
static struct regulator *regulator_proc12;
static struct regulator *regulator_sram11;
static struct regulator *regulator_sram12;

static unsigned long apmixed_base	= 0x1000c000;
static unsigned long mcucfg_base	= 0x0c530000;
/* static unsigned long topckgen_base	= 0x10000000; */

#define APMIXED_NODE		"mediatek,apmixed"
#define MCUCFG_NODE		"mediatek,mcucfg"
/* #define TOPCKGEN_NODE		"mediatek,topckgen" */

#define ARMPLL_LL_CON1		(apmixed_base + 0x204)	/* ARMPLL1 */
#define ARMPLL_L_CON1		(apmixed_base + 0x214)	/* ARMPLL2 */
#define CCIPLL_CON1		(apmixed_base + 0x294)

#define CKDIV1_LL_CFG		(mcucfg_base + 0x7a0)	/* MP0_PLL_DIVIDER */
#define CKDIV1_L_CFG		(mcucfg_base + 0x7a8)	/* MP1_PLL_DIVIDER */
#define CKDIV1_CCI_CFG		(mcucfg_base + 0x7c0)	/* BUS_PLL_DIVIDER */

struct mt_cpu_dvfs cpu_dvfs[NR_MT_CPU_DVFS] = {
	[MT_CPU_DVFS_LL] = {
		.name		= __stringify(MT_CPU_DVFS_LL),
		.id		= MT_CPU_DVFS_LL,
		.cpu_id		= 0,
		.idx_normal_max_opp = -1,
		.idx_opp_ppm_base = 15,
		.idx_opp_ppm_limit = 0,
		.Vproc_buck_id	= CPU_DVFS_VPROC12,
		.Vsram_buck_id	= CPU_DVFS_VSRAM12,
		.Pll_id		= PLL_LL_CLUSTER,
	},

	[MT_CPU_DVFS_L] = {
		.name		= __stringify(MT_CPU_DVFS_L),
		.id		= MT_CPU_DVFS_L,
		.cpu_id		= 4,
		.idx_normal_max_opp = -1,
		.idx_opp_ppm_base = 15,
		.idx_opp_ppm_limit = 0,
		.Vproc_buck_id	= CPU_DVFS_VPROC11,
		.Vsram_buck_id	= CPU_DVFS_VSRAM11,
		.Pll_id		= PLL_L_CLUSTER,
	},


	[MT_CPU_DVFS_CCI] = {
		.name		= __stringify(MT_CPU_DVFS_CCI),
		.id		= MT_CPU_DVFS_CCI,
		.cpu_id		= 10,
		.idx_normal_max_opp = -1,
		.idx_opp_ppm_base = 15,
		.idx_opp_ppm_limit = 0,
		.Vproc_buck_id	= CPU_DVFS_VPROC12,
		.Vsram_buck_id	= CPU_DVFS_VSRAM12,
		.Pll_id		= PLL_CCI_CLUSTER,
	},
};

static int set_cur_volt_proc_cpu(struct buck_ctrl_t *buck_p,
	unsigned int volt)
{
	unsigned int max_volt = MAX_VPROC_VOLT + 625;

	if (buck_p->buck_id == CPU_DVFS_VPROC12)
		return regulator_set_voltage(regulator_proc12, volt * 10,
		max_volt * 10);
	else
		return regulator_set_voltage(regulator_proc11, volt * 10,
		max_volt * 10);
}

static unsigned int get_cur_volt_proc_cpu(struct buck_ctrl_t *buck_p)
{
	unsigned int rdata;

	if (buck_p->buck_id == CPU_DVFS_VPROC12)
		rdata = regulator_get_voltage(regulator_proc12) / 10;
	else
		rdata = regulator_get_voltage(regulator_proc11) / 10;

	return rdata;
}

static unsigned int mt6358_transfer2pmicval(unsigned int volt)
{
	return ((volt - 50000) + 625 - 1) / 625;
}

static unsigned int mt6358_transfer2volt(unsigned int val)
{
	return val * 625 + 50000;
}

static unsigned int mt6358_vproc_settletime(unsigned int old_volt,
	unsigned int new_volt)
{
	/* UP:10mv/us DOWN:7.5mv/us */
	if (new_volt > old_volt)
		return ((new_volt - old_volt) + UP_SRATE - 1) / UP_SRATE +
		PMIC_CMD_DELAY_TIME;
	else
		return ((old_volt - new_volt) + DOWN_SRATE - 1) / DOWN_SRATE +
		PMIC_CMD_DELAY_TIME;
}

static int set_cur_volt_sram_cpu(struct buck_ctrl_t *buck_p,
	unsigned int volt)
{
/* HW auto-tracking */
#if 0
	unsigned int max_volt = MAX_VSRAM_VOLT + 625;

	if (buck_p->buck_id == CPU_DVFS_VSRAM12)
		return regulator_set_voltage(regulator_sram12, volt * 10,
		max_volt * 10);
	else
		return regulator_set_voltage(regulator_sram11, volt * 10,
		max_volt * 10);
#endif
	return 0;
}

static unsigned int get_cur_volt_sram_cpu(struct buck_ctrl_t *buck_p)
{
	unsigned int rdata;

	if (buck_p->buck_id == CPU_DVFS_VSRAM12)
		rdata = regulator_get_voltage(regulator_sram12) / 10;
	else
		rdata = regulator_get_voltage(regulator_sram11) / 10;

	return rdata;
}

static unsigned int mt6358_vsram_settletime(unsigned int old_volt,
	unsigned int new_volt)
{
	/* UP:10mv/us DOWN:7.5mv/us */
	if (new_volt > old_volt)
		return ((new_volt - old_volt) + UP_SRATE - 1) / UP_SRATE +
		PMIC_CMD_DELAY_TIME;
	else
		return ((old_volt - new_volt) + DOWN_SRATE - 1) / DOWN_SRATE +
		PMIC_CMD_DELAY_TIME;
}

/* upper layer CANNOT use 'set' function in secure path */
static struct buck_ctrl_ops buck_ops_mt6358_vproc = {
	.get_cur_volt		= get_cur_volt_proc_cpu,
	.set_cur_volt		= set_cur_volt_proc_cpu,
	.transfer2pmicval	= mt6358_transfer2pmicval,
	.transfer2volt		= mt6358_transfer2volt,
	.settletime		= mt6358_vproc_settletime,
};

static struct buck_ctrl_ops buck_ops_mt6358_vsram = {
	.get_cur_volt		= get_cur_volt_sram_cpu,
	.set_cur_volt		= set_cur_volt_sram_cpu,
	.transfer2pmicval	= mt6358_transfer2pmicval,
	.transfer2volt		= mt6358_transfer2volt,
	.settletime		= mt6358_vsram_settletime,
};

struct buck_ctrl_t buck_ctrl[NR_MT_BUCK] = {
	[CPU_DVFS_VPROC12] = {
		.name		= __stringify(BUCK_MT6358_VPROC),
		.buck_id	= CPU_DVFS_VPROC12,
		.buck_ops	= &buck_ops_mt6358_vproc,
	},

	[CPU_DVFS_VPROC11] = {
		.name		= __stringify(BUCK_MT6358_VPROC),
		.buck_id	= CPU_DVFS_VPROC11,
		.buck_ops	= &buck_ops_mt6358_vproc,
	},

	[CPU_DVFS_VSRAM12] = {
		.name		= __stringify(BUCK_MT6358_VSRAM),
		.buck_id	= CPU_DVFS_VSRAM12,
		.buck_ops	= &buck_ops_mt6358_vsram,
	},

	[CPU_DVFS_VSRAM11] = {
		.name		= __stringify(BUCK_MT6358_VSRAM),
		.buck_id	= CPU_DVFS_VSRAM11,
		.buck_ops	= &buck_ops_mt6358_vsram,
	},
};

/* PMIC Part */
void prepare_pmic_config(struct mt_cpu_dvfs *p)
{
}

int __attribute__((weak)) sync_dcm_set_mp0_freq(unsigned int mhz)
{
	return 0;
}

int __attribute__((weak)) sync_dcm_set_mp1_freq(unsigned int mhz)
{
	return 0;
}

int __attribute__((weak)) sync_dcm_set_mp2_freq(unsigned int mhz)
{
	return 0;
}

int __attribute__((weak)) sync_dcm_set_cci_freq(unsigned int mhz)
{
	return 0;
}

/* PLL Part */
void prepare_pll_addr(enum mt_cpu_dvfs_pll_id pll_id)
{
	struct pll_ctrl_t *pll_p = id_to_pll_ctrl(pll_id);

	if (pll_p == NULL)
		return;
	pll_p->armpll_addr =
	(unsigned int *)(pll_id == PLL_LL_CLUSTER ? ARMPLL_LL_CON1 :
	pll_id == PLL_L_CLUSTER ? ARMPLL_L_CON1 : CCIPLL_CON1);

	pll_p->armpll_div_addr =
	(unsigned int *)(pll_id == PLL_LL_CLUSTER ? CKDIV1_LL_CFG :
	pll_id == PLL_L_CLUSTER ? CKDIV1_L_CFG : CKDIV1_CCI_CFG);
}

unsigned int _cpu_dds_calc(unsigned int khz)
{
	unsigned int dds;

	dds = ((khz / 1000) << 14) / 26;

	return dds;
}

static void adjust_armpll_dds(struct pll_ctrl_t *pll_p, unsigned int vco,
	unsigned int pos_div)
{
	unsigned int dds;
	unsigned int val;

	dds = _GET_BITS_VAL_(21:0, _cpu_dds_calc(vco));

	val = cpufreq_read(pll_p->armpll_addr) & ~(_BITMASK_(21:0));
	val |= dds;

	cpufreq_write(pll_p->armpll_addr, val | _BIT_(31) /* CHG */);
	udelay(PLL_SETTLE_TIME);
}

static void adjust_posdiv(struct pll_ctrl_t *pll_p, unsigned int pos_div)
{
	unsigned int sel;

	sel = (pos_div == 1 ? 0 :
	       pos_div == 2 ? 1 :
	       pos_div == 4 ? 2 : 0);

	cpufreq_write_mask(pll_p->armpll_addr, 26:24, sel);
	udelay(POS_SETTLE_TIME);
}

static void adjust_clkdiv(struct pll_ctrl_t *pll_p, unsigned int clk_div)
{
	unsigned int sel;

	sel = (clk_div == 1 ? 8 :
	       clk_div == 2 ? 10 :
	       clk_div == 4 ? 11 : 8);

	cpufreq_write_mask(pll_p->armpll_div_addr, 21:17, sel);
}

unsigned char get_posdiv(struct pll_ctrl_t *pll_p)
{
	unsigned char sel, cur_posdiv;

	sel = _GET_BITS_VAL_(26:24, cpufreq_read(pll_p->armpll_addr));
	cur_posdiv = (sel == 0 ? 1 :
		sel == 1 ? 2 :
		sel == 2 ? 4 : 1);

	return cur_posdiv;
}

unsigned char get_clkdiv(struct pll_ctrl_t *pll_p)
{
	unsigned char sel, cur_clkdiv;

	sel = _GET_BITS_VAL_(21:17, cpufreq_read(pll_p->armpll_div_addr));
	cur_clkdiv = (sel == 8 ? 1 :
		sel == 10 ? 2 :
		sel == 11 ? 4 : 1);

	return cur_clkdiv;
}

static void adjust_freq_hopping(struct pll_ctrl_t *pll_p, unsigned int dds)
{
#ifdef CONFIG_MTK_FREQ_HOPPING
	mt_dfs_armpll(pll_p->hopping_id, dds);
#endif
}

/* Frequency API */
static unsigned int pll_to_clk(unsigned int pll_f, unsigned int ckdiv1)
{
	unsigned int freq = pll_f;

	switch (ckdiv1) {
	case 8:
		break;
	case 9:
		freq = freq * 3 / 4;
		break;
	case 10:
		freq = freq * 2 / 4;
		break;
	case 11:
		freq = freq * 1 / 4;
		break;
	case 16:
		break;
	case 17:
		freq = freq * 4 / 5;
		break;
	case 18:
		freq = freq * 3 / 5;
		break;
	case 19:
		freq = freq * 2 / 5;
		break;
	case 20:
		freq = freq * 1 / 5;
		break;
	case 24:
		break;
	case 25:
		freq = freq * 5 / 6;
		break;
	case 26:
		freq = freq * 4 / 6;
		break;
	case 27:
		freq = freq * 3 / 6;
		break;
	case 28:
		freq = freq * 2 / 6;
		break;
	case 29:
		freq = freq * 1 / 6;
		break;
	default:
		break;
	}

	return freq;
}

static unsigned int _cpu_freq_calc(unsigned int con1, unsigned int ckdiv1)
{
	unsigned int freq;
	unsigned int posdiv;

	posdiv = _GET_BITS_VAL_(26:24, con1);

	con1 &= _BITMASK_(21:0);
	freq = ((con1 * 26) >> 14) * 1000;

	switch (posdiv) {
	case 0:
		break;
	case 1:
		freq = freq / 2;
		break;
	case 2:
		freq = freq / 4;
		break;
	case 3:
		freq = freq / 8;
		break;
	default:
		freq = freq / 16;
		break;
	};

	return pll_to_clk(freq, ckdiv1);
}

unsigned int get_cur_phy_freq(struct pll_ctrl_t *pll_p)
{
	unsigned int con1;
	unsigned int ckdiv1;
	unsigned int cur_khz;

	con1 = cpufreq_read(pll_p->armpll_addr);
	ckdiv1 = cpufreq_read(pll_p->armpll_div_addr);
	ckdiv1 = _GET_BITS_VAL_(21:17, ckdiv1);

	cur_khz = _cpu_freq_calc(con1, ckdiv1);

	cpufreq_ver
	("@%s: (%s) = cur_khz = %u, con1[0x%p] = 0x%x, ckdiv1_val = 0x%x\n",
	__func__, pll_p->name, cur_khz, pll_p->armpll_addr, con1, ckdiv1);

	return cur_khz;
}

static void _cpu_clock_switch(struct pll_ctrl_t *pll_p, enum top_ckmuxsel sel)
{
	cpufreq_write_mask(pll_p->armpll_div_addr, 10:9, sel);
}

static enum top_ckmuxsel _get_cpu_clock_switch(struct pll_ctrl_t *pll_p)
{
	return _GET_BITS_VAL_(10:9, cpufreq_read(pll_p->armpll_div_addr));
}

/* upper layer CANNOT use 'set' function in secure path */
static struct pll_ctrl_ops pll_ops_ll = {
	.get_cur_freq		= get_cur_phy_freq,
	.set_armpll_dds		= adjust_armpll_dds,
	.set_armpll_posdiv	= adjust_posdiv,
	.set_armpll_clkdiv	= adjust_clkdiv,
	.set_freq_hopping	= adjust_freq_hopping,
	.clksrc_switch		= _cpu_clock_switch,
	.get_clksrc		= _get_cpu_clock_switch,
	.set_sync_dcm		= sync_dcm_set_mp0_freq,
};

static struct pll_ctrl_ops pll_ops_l = {
	.get_cur_freq		= get_cur_phy_freq,
	.set_armpll_dds		= adjust_armpll_dds,
	.set_armpll_posdiv	= adjust_posdiv,
	.set_armpll_clkdiv	= adjust_clkdiv,
	.set_freq_hopping	= adjust_freq_hopping,
	.clksrc_switch		= _cpu_clock_switch,
	.get_clksrc		= _get_cpu_clock_switch,
	.set_sync_dcm		= sync_dcm_set_mp1_freq,
};

static struct pll_ctrl_ops pll_ops_cci = {
	.get_cur_freq		= get_cur_phy_freq,
	.set_armpll_dds		= adjust_armpll_dds,
	.set_armpll_posdiv	= adjust_posdiv,
	.set_armpll_clkdiv	= adjust_clkdiv,
	.set_freq_hopping	= adjust_freq_hopping,
	.clksrc_switch		= _cpu_clock_switch,
	.get_clksrc		= _get_cpu_clock_switch,
	.set_sync_dcm		= sync_dcm_set_cci_freq,
};

struct pll_ctrl_t pll_ctrl[NR_MT_PLL] = {
	[PLL_LL_CLUSTER] = {
		.name		= __stringify(PLL_LL_CLUSTER),
		.pll_id		= PLL_LL_CLUSTER,
		.hopping_id	= FH_PLL0,	/* ARMPLL1 */
		.pll_ops	= &pll_ops_ll,
	},

	[PLL_L_CLUSTER] = {
		.name		= __stringify(PLL_L_CLUSTER),
		.pll_id		= PLL_L_CLUSTER,
		.hopping_id	= FH_PLL1,	/* ARMPLL2 */
		.pll_ops	= &pll_ops_l,
	},

	[PLL_CCI_CLUSTER] = {
		.name		= __stringify(PLL_CCI_CLUSTER),
		.pll_id		= PLL_CCI_CLUSTER,
		.hopping_id	= FH_PLL3,	/* CCIPLL */
		.pll_ops	= &pll_ops_cci,
	},
};

/* Always put action cpu at last */
struct hp_action_tbl cpu_dvfs_hp_action[] = {
	{
		.action		= CPUFREQ_CPU_DOWN_PREPARE,
		.cluster	= MT_CPU_DVFS_LL,
		.trigged_core	= 1,
		.hp_action_cfg[MT_CPU_DVFS_LL].action_id = FREQ_LOW,
	},

	{
		.action		= CPUFREQ_CPU_DOWN_PREPARE,
		.cluster	= MT_CPU_DVFS_L,
		.trigged_core	= 1,
		.hp_action_cfg[MT_CPU_DVFS_L].action_id = FREQ_LOW,
	},
};

unsigned int nr_hp_action = ARRAY_SIZE(cpu_dvfs_hp_action);
static int can_turbo;
void mt_cpufreq_turbo_action(unsigned long action,
	unsigned int *cpus, enum mt_cpu_dvfs_id cluster_id)
{
	can_turbo = 0;
}

int mt_cpufreq_turbo_config(enum mt_cpu_dvfs_id id,
	unsigned int turbo_f, unsigned int turbo_v)
{
	return 1;
}

int mt_cpufreq_regulator_map(struct platform_device *pdev)
{
	regulator_proc11 = regulator_get(&pdev->dev, "vproc11");
	regulator_proc12 = regulator_get(&pdev->dev, "vproc12");
	regulator_sram11 = regulator_get(&pdev->dev, "vsram_proc11");
	regulator_sram12 = regulator_get(&pdev->dev, "vsram_proc12");

	return 0;
}

int mt_cpufreq_dts_map(void)
{
	struct device_node *node;

	/* apmixed */
	node = of_find_compatible_node(NULL, NULL, APMIXED_NODE);
	apmixed_base = (unsigned long)of_iomap(node, 0);

	/* mcucfg */
	node = of_find_compatible_node(NULL, NULL, MCUCFG_NODE);
	mcucfg_base = (unsigned long)of_iomap(node, 0);

	return 0;
}

#define CPUFREQ_EFUSE_IDX_0 50
#define CPUFREQ_SEG_CODE_IDX_0 7
#define CPUFREQ_BIN_CODE_IDX_0 120

unsigned int _mt_cpufreq_get_cpu_level(void)
{
	unsigned int lv = CPU_LEVEL_0;
	unsigned int buf = CPU_LEVEL_0;
	unsigned int temp = 0;
	unsigned int segcode = 0;
	unsigned int bincode = 0;
	unsigned int turbocode = 0;
	unsigned int a_code = 0;

	temp = get_devinfo_with_index(CPUFREQ_EFUSE_IDX_0);
	segcode = (get_devinfo_with_index(CPUFREQ_SEG_CODE_IDX_0) >> 5) & 0x1;
	bincode = (get_devinfo_with_index(CPUFREQ_BIN_CODE_IDX_0) >> 4) & 0x7;
	turbocode = (get_devinfo_with_index(CPUFREQ_SEG_CODE_IDX_0) >> 3) & 0x1;
	a_code = (get_devinfo_with_index(CPUFREQ_SEG_CODE_IDX_0) >> 1) & 0x1;

	if (temp > 0x40) {
		if (segcode == 1)
			lv = CPU_LEVEL_5;	/* V6 */
		else {
			if (bincode == 0)
				lv = CPU_LEVEL_2;	/* V5_1 */
			else if (bincode == 1)
				lv = CPU_LEVEL_3;	/* V5_2 */
			else if (bincode == 2)
				lv = CPU_LEVEL_4;	/* V5_3 */
			else
				lv = CPU_LEVEL_4;	/* V5_3 */
		}
	} else if (temp == 0x40) {
		if (segcode == 1)
			lv = CPU_LEVEL_5;	/* V6 */
		else
			lv = CPU_LEVEL_1;	/* V4 */
	} else
		lv = CPU_LEVEL_0;	/* V3 */

	if (a_code == 1)
		lv = CPU_LEVEL_7;	/* V5_4 */

	buf = lv;
	if (turbocode == 1)
		lv = CPU_LEVEL_6;	/* V5_T */

#if defined(LV)
	lv = buf;
#endif

	turbo_flag = 0;

	tag_pr_info("%d, %d, (%d, %d)\n",
		lv, turbo_flag, UP_SRATE, DOWN_SRATE);

	return lv;
}

unsigned int cpufreq_get_nr_clusters(void)
{
	return (NR_MT_CPU_DVFS - 1);
}

void cpufreq_get_cluster_cpus(struct cpumask *cpu_mask, unsigned int cid)
{
	if (cid == 0) {
		cpumask_setall(cpu_mask);
		cpumask_clear_cpu(4, cpu_mask);
		cpumask_clear_cpu(5, cpu_mask);
		cpumask_clear_cpu(6, cpu_mask);
		cpumask_clear_cpu(7, cpu_mask);
	} else if (cid == 1) {
		cpumask_clear(cpu_mask);
		cpumask_set_cpu(4, cpu_mask);
		cpumask_set_cpu(5, cpu_mask);
		cpumask_set_cpu(6, cpu_mask);
		cpumask_set_cpu(7, cpu_mask);
	}

	cpufreq_ver("cluster%d: cpumask = %*pbl\n",
		cid, cpumask_pr_args(cpu_mask));
}

unsigned int cpufreq_get_cluster_id(unsigned int cpu_id)
{
	struct cpumask cpu_mask;
	int i;

	for (i = 0; i < NR_MT_CPU_DVFS - 1; i++) {
		cpufreq_get_cluster_cpus(&cpu_mask, i);
		if (cpumask_test_cpu(cpu_id, &cpu_mask)) {
			cpufreq_ver("cluster%d: cpumask = %*pbl\n",
			i, cpumask_pr_args(&cpu_mask));
			return i;
		}
	}

	return 0;
}
