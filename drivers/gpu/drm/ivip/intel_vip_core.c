/*
 * intel_vip_core.c -- Intel Video and Image Processing(VIP)
 * Frame Buffer II driver
 *
 * This driver supports the Intel VIP Frame Reader component.
 * More info on the hardware can be found in the Intel Video
 * and Image Processing Suite User Guide at this address
 * http://www.altera.com/literature/ug/ug_vip.pdf.
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
 * Authors:
 * Ong, Hean-Loong <hean.loong.ong@intel.com>
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "intel_vip_drv.h"

static void intelvipfb_enable(struct drm_simple_display_pipe *pipe,
	       struct drm_crtc_state *crtc_state)
{
	/*
	 * The frameinfo variable has to correspond to the size of the VIP Suite
	 * Frame Reader register 7 which will determine the maximum size used
	 * in this frameinfo
	 */

	u32 frameinfo;
	struct intelvipfb_priv *priv = pipe->plane.dev->dev_private;
	void __iomem *base = priv->base;
	struct drm_plane_state *state = pipe->plane.state;
	dma_addr_t addr;

	addr = drm_fb_cma_get_gem_addr(state->fb, state, 0);

	dev_info(pipe->plane.dev->dev, "Address 0x%x\n", addr);

	frameinfo =
		readl(base + INTELVIPFB_FRAME_READER) & 0x00ffffff;
	writel(frameinfo, base + INTELVIPFB_FRAME_INFO);
	writel(addr, base + INTELVIPFB_FRAME_START);
	/* Finally set the control register to 1 to start streaming */
	writel(1, base + INTELVIPFB_CONTROL);
}

static void intelvipfb_disable(struct drm_simple_display_pipe *pipe)
{
	struct intelvipfb_priv *priv = pipe->plane.dev->dev_private;
	void __iomem *base = priv->base;
	/* set the control register to 0 to stop streaming */
	writel(0, base + INTELVIPFB_CONTROL);
}

static const struct drm_mode_config_funcs intelvipfb_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void intelvipfb_setup_mode_config(struct drm_device *drm)
{
	drm_mode_config_init(drm);
	drm->mode_config.funcs = &intelvipfb_mode_config_funcs;
}

static int intelvipfb_pipe_prepare_fb(struct drm_simple_display_pipe *pipe,
				      struct drm_plane_state *plane_state)
{
	return drm_fb_cma_prepare_fb(&pipe->plane, plane_state);
}

static void intelvipfb_update(struct drm_simple_display_pipe *pipe,
		 struct drm_plane_state *old_state)
{
	struct drm_crtc *crtc = &pipe->crtc;

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}
}

static struct drm_simple_display_pipe_funcs fbpriv_funcs = {
	.prepare_fb = intelvipfb_pipe_prepare_fb,
	.update = intelvipfb_update,
	.enable = intelvipfb_enable,
	.disable = intelvipfb_disable
};

int intelvipfb_probe(struct device *dev, void __iomem *base)
{
	int retval;
	struct drm_device *drm;
	struct intelvipfb_priv *fbpriv = dev_get_drvdata(dev);
	struct drm_connector *connector;

	u32 formats[] = {DRM_FORMAT_XRGB8888};

	dev_set_drvdata(dev, fbpriv);

	drm = fbpriv->drm;

	drm->dev_private = fbpriv;

	intelvipfb_setup_mode_config(drm);

	connector = intelvipfb_conn_setup(drm);
	if (!connector) {
		dev_err(drm->dev, "Connector setup failed\n");
		goto err_mode_config;
	}

	retval = drm_simple_display_pipe_init(drm, &fbpriv->pipe,
					      &fbpriv_funcs, formats,
					      ARRAY_SIZE(formats), connector);
	if (retval < 0) {
		dev_err(drm->dev, "Cannot setup simple display pipe\n");
		goto err_mode_config;
	}

	fbpriv->fbcma = drm_fbdev_cma_init(drm,
			drm->mode_config.preferred_depth,
					   drm->mode_config.num_connector);

	drm_mode_config_reset(drm);

	drm_dev_register(drm, 0);

	return retval;

err_mode_config:

	drm_mode_config_cleanup(drm);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(intelvipfb_probe);

int intelvipfb_remove(struct device *dev)
{
	struct intelvipfb_priv *fbpriv = dev_get_drvdata(dev);
	struct drm_device *drm =  fbpriv->drm;

	drm_dev_unregister(drm);

	if (fbpriv->fbcma)
		drm_fbdev_cma_fini(fbpriv->fbcma);

	drm_mode_config_cleanup(drm);

	drm_dev_unref(drm);

	devm_kfree(dev, fbpriv);

	return 0;
}
EXPORT_SYMBOL_GPL(intelvipfb_remove);

MODULE_AUTHOR("Ong, Hean-Loong <hean.loong.ong@intel.com>");
MODULE_DESCRIPTION("Intel VIP Frame Buffer II driver");
MODULE_LICENSE("GPL v2");
