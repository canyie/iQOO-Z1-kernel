#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include "vts_core.h"
#include "vts_init.h"

static unsigned int init_flags;

#define vts_module_init(name, FLAGS) do { \
		int ret; \
		int load; \
		ret = vts_driver_##name##_init(&load); \
		\
		if (load && !ret) { \
			init_flags |= FLAGS; \
			VTI("%s, init_flags:0x%x\n", #name, init_flags); \
		} else if (load && ret) { \
			return ret; \
		} \
	} while (0)

#define vts_module_exit(name, FLAGS) do { \
		if (init_flags & FLAGS) { \
			VTI("%s, init_flags:0x%x\n", #name, init_flags); \
			vts_driver_##name##_exit(); \
		} \
	} while (0)

enum vts_init_flags {
	LOG_MODULE = BIT(0),
	FTM4_MODULE = BIT(1),
	FTM5_MODULE = BIT(2),
	NT_I2C_MODULE = BIT(3),
	NT_NO_FLASH_MODULE = BIT(4),
	SEC_Y761_MODULE = BIT(5),
	GT9885_MODULE = BIT(6),
	GT9886_MODULE = BIT(7),
	FT8719_FLASH_MODULE = BIT(8),
	FT8756_NO_FALSH_MODULE = BIT(9),
	HVT_NO_FLASH_MODULE = BIT(10),
	ILI_TEK_MODULE = BIT(11),
	ILI_9882N_MODULE = BIT(12),
	S3908_MODULE = BIT(13)
};

static int __init vts_core_init(void)
{
	
	vts_module_init(nt_no_flash, NT_NO_FLASH_MODULE);
	
	return 0;
}

static void __exit vts_core_exit(void)
{

	vts_module_exit(nt_no_flash, NT_NO_FLASH_MODULE);

}

#ifdef MODULE
module_init(vts_core_init);
module_exit(vts_core_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VTS");
MODULE_DESCRIPTION("VTS MODULE");
MODULE_VERSION("0.01");
#else
late_initcall(vts_core_init);
module_exit(vts_core_exit);
#endif
