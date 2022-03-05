
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc

obj-y += max20328-i2c.o
obj-$(CONFIG_MTK_TYPEC_ACCDET) += typec_switch.o

ifeq ($(CONFIG_SND_SOC_MT6358),y)
  MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
  subdir-ccflags-y +=  -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
  subdir-ccflags-y +=  -I$(srctree)/drivers/misc/mediatek/accdet/mt6358
endif

ifeq ($(CONFIG_SND_SOC_MT6359),y)
  MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
  subdir-ccflags-y +=  -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
  subdir-ccflags-y +=  -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
endif
