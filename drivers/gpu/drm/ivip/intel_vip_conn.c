/*
 * intel_vip_conn.c -- Intel Video and Image Processing(VIP)
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
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_plane_helper.h>

static enum drm_connector_status
intelvipfb_drm_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void intelvipfb_drm_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs intelvipfb_drm_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.detect = intelvipfb_drm_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = intelvipfb_drm_connector_destroy,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int intelvipfb_drm_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	int count;

	count = drm_add_modes_noedid(connector, drm->mode_config.max_width,
				     drm->mode_config.max_height);
	drm_set_preferred_mode(connector, drm->mode_config.max_width,
			       drm->mode_config.max_height);
	return count;
}

static const struct drm_connector_helper_funcs
intelvipfb_drm_connector_helper_funcs = {
	.get_modes = intelvipfb_drm_connector_get_modes,
};

struct drm_connector *
intelvipfb_conn_setup(struct drm_device *drm)
{
	struct drm_connector *conn;
	int ret;

	conn = devm_kzalloc(drm->dev, sizeof(*conn), GFP_KERNEL);
	if (IS_ERR(conn))
		return NULL;

	ret = drm_connector_init(drm, conn, &intelvipfb_drm_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret < 0) {
		dev_err(drm->dev, "failed to initialize drm connector\n");
		ret = -ENOMEM;
		goto error_connector_cleanup;
	}

	drm_connector_helper_add(conn, &intelvipfb_drm_connector_helper_funcs);

	return conn;

error_connector_cleanup:
	drm_connector_cleanup(conn);

	return NULL;
}
