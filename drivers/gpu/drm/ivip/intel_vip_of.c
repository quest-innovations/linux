/*
 * intel_vip_of.c -- Intel Video and Image Processing(VIP)
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

#include <linux/component.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "intel_vip_drv.h"

DEFINE_DRM_GEM_CMA_FOPS(drm_fops);

static void intelvipfb_lastclose(struct drm_device *drm)
{
	struct intelvipfb_priv *priv = drm->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbcma);
}

static struct drm_driver intelvipfb_drm = {
	.driver_features =
			DRIVER_MODESET | DRIVER_GEM |
			DRIVER_PRIME | DRIVER_ATOMIC,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
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
	.lastclose = intelvipfb_lastclose,
	.name = DRIVER_NAME,
	.date = "20170729",
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
				struct intelvipfb_priv *fbpriv)
{
	struct drm_device *drm = fbpriv->drm;
	struct device_node *np = dev->of_node;
	int mem_word_width;
	int max_h, max_w;
	int bpp;
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

	ret = of_property_read_u32(np, "altr,bits-per-symbol", &bpp);
	if (ret) {
		dev_err(dev,
			"Missing required parameter 'altr,bits-per-symbol'");
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
	drm->mode_config.preferred_depth = bpp * BYTES_PER_PIXEL;

	return 0;
}

static int intelvipfb_of_probe(struct platform_device *pdev)
{
	int retval;
	struct resource *reg_res;
	struct intelvipfb_priv *fbpriv;
	struct device *dev = &pdev->dev;
	struct drm_device *drm;

	fbpriv = devm_kzalloc(dev, sizeof(*fbpriv), GFP_KERNEL);
	if (!fbpriv)
		return -ENOMEM;

	/*setup DRM */
	drm = drm_dev_alloc(&intelvipfb_drm, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	retval = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (retval)
		return -ENODEV;

	fbpriv->drm = drm;

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res)
		return -ENOMEM;

	fbpriv->base = devm_ioremap_resource(dev, reg_res);

	if (IS_ERR(fbpriv->base)) {
		dev_err(dev, "devm_ioremap_resource failed\n");
		retval = PTR_ERR(fbpriv->base);
		return -ENOMEM;
	}

	intelvipfb_drm_setup(dev, fbpriv);

	dev_set_drvdata(dev, fbpriv);

	return intelvipfb_probe(dev, fbpriv->base);
}

static int intelvipfb_of_remove(struct platform_device *pdev)
{
	return intelvipfb_remove(&pdev->dev);
}

/*
 * The name vip-frame-buffer-2.0 is derived from
 * http://www.altera.com/literature/ug/ug_vip.pdf
 * frame buffer IP cores section 14
 */

static const struct of_device_id intelvipfb_of_match[] = {
	{ .compatible = "altr,vip-frame-buffer-2.0" },
	{},
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
