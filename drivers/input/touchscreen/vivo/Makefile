obj-y += vivo_touchscreen_common.o
obj-y += vivo_touchscreen_config.o
obj-y += vivo_ts_function.o
obj-y += vivo_ts_ic_ctl.o
obj-y += vivo_ts_node.o
obj-y += vivo_touchscreen_virtual_key.o
obj-y += vivo_key.o
obj-y += vivo_ts_prox.o

ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif

obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o
vivo_ts-y += vts_init.o