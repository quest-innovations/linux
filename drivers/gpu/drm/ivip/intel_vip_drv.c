// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Intel Corporation.
 *
 * intel_vip_core.c -- Intel Video and Image Processing(VIP)
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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "intel_vip_drv.h"

static inline struct intelvipfb_priv *
pipe_to_intelvipdrm(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct intelvipfb_priv, pipe);
}

DEFINE_DRM_GEM_CMA_FOPS(drm_fops);

static struct drm_driver intelvipfb_drm = {
	.driver_features =
			DRIVER_MODESET | DRIVER_GEM |
			DRIVER_PRIME | DRIVER_ATOMIC,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
	.name = DRIVER_NAME,
	.date = "20190129",
	.desc = "Intel FPGA VIP SUITE",
	.major = 1,
	.minor = 0,
	.ioctls = NULL,
	.patchlevel = 0,
	.fops = &drm_fops,
};

/*
 * Setting up information derived from OF Device Tree Nodes
 * max-width, max-height, bits per pixel, memory port width
 */

static int intelvipfb_drm_setup(struct device *dev,
					struct intelvipfb_priv *priv)
{
	struct drm_device *drm = &priv->drm;
	struct device_node *np = dev->of_node;
	int mem_word_width;
	int max_h, max_w;
	int ret;

	ret = of_property_read_u32(np, "altr,max-width", &max_w);
	if (ret) {
		dev_err(dev,
			"Missing required parameter 'altr,max-width'");
		return ret;
	}

	ret = of_property_read_u32(np, "altr,max-height", &max_h);
	if (ret) {
		dev_err(dev,
			"Missing required parameter 'altr,max-height'");
		return ret;
	}

	ret = of_property_read_u32(np, "altr,mem-port-width", &mem_word_width);
	if (ret) {
		dev_err(dev, "Missing required parameter 'altr,mem-port-width '");
		return ret;
	}

	if (!(mem_word_width >= 32 && mem_word_width % 32 == 0)) {
		dev_err(dev,
			"mem-word-width is set to %i. must be >= 32 and multiple of 32.",
			 mem_word_width);
		return -ENODEV;
	}

	drm->mode_config.min_width = 640;
	drm->mode_config.min_height = 480;
	drm->mode_config.max_width = max_w;
	drm->mode_config.max_height = max_h;
	drm->mode_config.preferred_depth = 32;

	return 0;
}

static int intelvipfb_of_probe(struct platform_device *pdev)
{
	int retval;
	struct resource *reg_res;
	struct intelvipfb_priv *priv;
	struct device *dev = &pdev->dev;
	struct drm_device *drm;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = &priv->drm;

	/* setup DRM */
	retval = drm_dev_init(drm, &intelvipfb_drm, dev);
	if (retval) {
		dev_err(dev, "[" DRM_NAME ":%s] drm_dev_init failed: %d\n",
			__func__, retval);
		return -ENODEV;
	}

	retval = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (retval)
		return -ENODEV;

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res)
		return -ENOMEM;

	priv->base = devm_ioremap_resource(dev, reg_res);

	if (IS_ERR(priv->base)) {
		dev_err(dev, "devm_ioremap_resource failed\n");
		retval = PTR_ERR(priv->base);
		return -ENOMEM;
	}

	intelvipfb_drm_setup(dev, priv);

	dev_set_drvdata(dev, priv);

	return intelvipfb_probe(dev);
}

static int intelvipfb_of_remove(struct platform_device *pdev)
{
	return intelvipfb_remove(&pdev->dev);
}

static void intelvipfb_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *
				plane_state)
{
	/*
	 * The frameinfo variable has to correspond to the size of the VIP Suite
	 * Frame Reader register 7 which will determine the maximum size used
	 * in this frameinfo
	 */
	struct intelvipfb_priv *priv = pipe_to_intelvipdrm(pipe);
	u32 frameinfo;
	void __iomem *base = priv->base;
	struct drm_plane_state *state = pipe->plane.state;
	dma_addr_t addr;

	addr = drm_fb_cma_get_gem_addr(state->fb, state, 0);

	frameinfo =
		readl(base + INTELVIPFB_FRAME_READER) & 0x00ffffff;
	writel(frameinfo, base + INTELVIPFB_FRAME_INFO);
	writel(addr, base + INTELVIPFB_FRAME_START);
	/* Finally set the control register to 1 to start streaming */
	writel(1, base + INTELVIPFB_CONTROL);
}

static void intelvipfb_disable(struct drm_simple_display_pipe *pipe)
{
	struct intelvipfb_priv *priv = pipe_to_intelvipdrm(pipe);
	void __iomem *base = priv->base;
	/* set the control register to 0 to stop streaming */
	writel(0, base + INTELVIPFB_CONTROL);
}

static const struct drm_mode_config_funcs intelvipfb_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void intelvipfb_setup_mode_config(struct drm_device *drm)
{
	drm_mode_config_init(drm);
	drm->mode_config.funcs = &intelvipfb_mode_config_funcs;
}

void intelvipfb_display_pipe_update(struct drm_simple_display_pipe *pipe,
				    struct drm_plane_state *old_state)
{
	struct intelvipfb_priv *priv = pipe_to_intelvipdrm(pipe);
	struct drm_crtc *crtc = &priv->pipe.crtc;

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}
}
EXPORT_SYMBOL(intelvipfb_display_pipe_update);

static struct drm_simple_display_pipe_funcs priv_funcs = {
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
	.update = intelvipfb_display_pipe_update,
	.enable = intelvipfb_enable,
	.disable = intelvipfb_disable
};

int intelvipfb_probe(struct device *dev)
{
	int retval;
	struct drm_device *drm;
	struct intelvipfb_priv *priv = dev_get_drvdata(dev);
	struct drm_connector *connector;
	u32 formats[] = {DRM_FORMAT_XRGB8888};

	drm = &priv->drm;

	intelvipfb_setup_mode_config(drm);

	connector = intelvipfb_conn_setup(drm);
	if (!connector) {
		dev_err(drm->dev, "Connector setup failed\n");
		goto err_mode_config;
	}

	retval = drm_simple_display_pipe_init(drm,
						&priv->pipe,
						&priv_funcs,
						formats,
						ARRAY_SIZE(formats),
						NULL, connector);

	if (retval < 0) {
		dev_err(drm->dev, "Cannot setup simple display pipe\n");
		goto err_mode_config;
	}

	drm_mode_config_reset(drm);

	drm_dev_register(drm, 0);

	drm_fbdev_generic_setup(drm, 32);

	dev_info(drm->dev, "ivip: Successfully created fb\n");

	return retval;

err_mode_config:

	drm_mode_config_cleanup(drm);
	return -ENODEV;
}

int intelvipfb_remove(struct device *dev)
{
	struct intelvipfb_priv *priv = dev_get_drvdata(dev);
	struct drm_device *drm =  &priv->drm;

	drm_dev_unregister(drm);

	drm_mode_config_cleanup(drm);

	return 0;
}

static const struct of_device_id intelvipfb_of_match[] = {
	{ .compatible = "altr,vip-frame-buffer-2.0" },
	{},
/*
 * The name vip-frame-buffer-2.0 is derived from
 * http://www.altera.com/literature/ug/ug_vip.pdf
 * frame buffer IP cores section 14
 */
};

MODULE_DEVICE_TABLE(of, intelvipfb_of_match);

static struct platform_driver intelvipfb_driver = {
	.probe = intelvipfb_of_probe,
	.remove = intelvipfb_of_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = intelvipfb_of_match,
	},
};

module_platform_driver(intelvipfb_driver);

/* Original author of Altera Frame Buffer*/
MODULE_AUTHOR("Walter Goossens <waltergoossens@home.nl>");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_AUTHOR("Chris Rauer <crauer@altera.com>");
/* Author of Intel FPGA Frame Buffer II*/
MODULE_AUTHOR("Ong, Hean-Loong <hean.loong.ong@intel.com>");
MODULE_DESCRIPTION("Intel VIP Frame Buffer II driver");
MODULE_LICENSE("GPL v2");
