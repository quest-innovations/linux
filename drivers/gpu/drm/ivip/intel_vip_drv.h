/*
 * Copyright (C) 2017 Intel Corporation.
 *
 * Intel Video and Image Processing(VIP) Frame Buffer II driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 * Ong, Hean-Loong <hean.loong.ong@intel.com>
 *
 */
#ifndef _INTEL_VIP_DRV_H
#define _INTEL_VIP_DRV_H
#include <linux/io.h>
#include <linux/fb.h>

#define DRIVER_NAME	"intelvipfb"
#define BYTES_PER_PIXEL	4
#define CRTC_NUM		1
#define CONN_NUM		1

/* control registers */
#define INTELVIPFB_CONTROL		0
#define INTELVIPFB_STATUS		0x4
#define INTELVIPFB_INTERRUPT		0x8
#define INTELVIPFB_FRAME_COUNTER	0xC
#define INTELVIPFB_FRAME_DROP		0x10
#define INTELVIPFB_FRAME_INFO		0x14
#define INTELVIPFB_FRAME_START		0x18
#define INTELVIPFB_FRAME_READER		0x1C

int intelvipfb_probe(struct device *dev, void __iomem *base);
int intelvipfb_remove(struct device *dev);
int intelvipfb_setup_crtc(struct drm_device *drm);
struct drm_connector *intelvipfb_conn_setup(struct drm_device *drm);

struct intelvipfb_priv {
	struct drm_simple_display_pipe pipe;
	struct drm_fbdev_cma *fbcma;
	struct drm_device *drm;
	void	__iomem	*base;
};

#endif
