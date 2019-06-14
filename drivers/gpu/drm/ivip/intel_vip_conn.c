// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Intel Corporation.
 *
 * intel_vip_conn.c -- Intel Video and Image Processing(VIP)
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>

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
	.detect = intelvipfb_drm_connector_detect,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.destroy = intelvipfb_drm_connector_destroy,
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

	drm_connector_helper_add(conn, &intelvipfb_drm_connector_helper_funcs);
	ret = drm_connector_init(drm, conn, &intelvipfb_drm_connector_funcs,
			DRM_MODE_CONNECTOR_DisplayPort);
	if (ret < 0) {
		dev_err(drm->dev, "failed to initialize drm connector\n");
		ret = -ENOMEM;
		goto error_connector_cleanup;
	}

	return conn;

error_connector_cleanup:
	drm_connector_cleanup(conn);

	return NULL;
}
