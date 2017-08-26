#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm

obj-$(CONFIG_DRM_IVIP) += ivip.o
ivip-objs := intel_vip_of.o intel_vip_core.o \
intel_vip_conn.o
