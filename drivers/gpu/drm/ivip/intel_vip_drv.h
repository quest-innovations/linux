/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Intel Corporation.
 *
 * Intel Video and Image Processing(VIP) Frame Buffer II driver.
 * Frame Buffer II driver
 *
 * This driver supports the Intel VIP Frame Reader component.
 * More info on the hardware can be found in the Intel Video
 * and Image Processing Suite User Guide at this address
 * http://www.altera.com/literature/ug/ug_vip.pdf.
 *
 * Authors:
 * Walter Goossens <waltergoossens@home.nl>
 * Thomas Chou <thomas@wytron.com.tw>
 * Chris Rauer <crauer@altera.com>
 * Ong, Hean-Loong <hean.loong.ong@intel.com>
 *
 */
#ifndef _INTEL_VIP_DRV_H
#define _INTEL_VIP_DRV_H

#define DRIVER_NAME    "intelvipfb"
#define BYTES_PER_PIXEL	 4
#define CRTC_NUM	        1
#define CONN_NUM	        1

/* control registers */
#define INTELVIPFB_CONTROL	      0
#define INTELVIPFB_STATUS	       0x4
#define INTELVIPFB_INTERRUPT	    0x8
#define INTELVIPFB_FRAME_COUNTER	0xC
#define INTELVIPFB_FRAME_DROP	   0x10
#define INTELVIPFB_FRAME_INFO	   0x14
#define INTELVIPFB_FRAME_START	  0x18
#define INTELVIPFB_FRAME_READER	         0x1C

int intelvipfb_probe(struct device *dev);
int intelvipfb_remove(struct device *dev);
int intelvipfb_setup_crtc(struct drm_device *drm);
struct drm_connector *intelvipfb_conn_setup(struct drm_device *drm);

struct intelvipfb_priv {
	/**
	 * @pipe: Display pipe structure
	 */
	struct drm_simple_display_pipe pipe;

	/**
	 * @drm: DRM device
	 */
	struct drm_device drm;

	/**
	 * @dirty_lock: Serializes framebuffer flushing
	 */
	struct mutex dirty_lock;

	/**
	 * @base: Base memory for the framebuffer
	 */
	void    __iomem *base;

	/**
	 * @fb_dirty: Framebuffer dirty callback
	 */
	int (*fb_dirty)(struct drm_framebuffer *framebuffer,
			struct drm_file *file_priv, unsigned int flags,
			unsigned int color, struct drm_clip_rect *clips,
			unsigned int num_clips);
};

#endif
