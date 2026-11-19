// SPDX-License-Identifier: GPL-2.0-only

/*
 * WV517S voice-coil motor driver
 *
 * Copyright (c) 2014 Intel Corporation.
 * Copyright (C) 2026 Maurizio Casciano <mauriziocasciano7@gmail.com>
 */

#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define WV517S_MAX_FOCUS_POSITION	1023
#define WV517S_DEFAULT_FOCUS_POSITION	300

#define WV517S_REG_FOCUS		0x41
#define WV517S_REG_DRIVE_MODE		0x43
#define WV517S_DRIVE_MODE_12_6_MS	0x0211

struct wv517s_device {
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev sd;
	struct regmap *regmap;
	bool resuming;
};

static inline struct wv517s_device *to_wv517s(struct v4l2_subdev *sd)
{
	return container_of(sd, struct wv517s_device, sd);
}

static const struct regmap_config wv517s_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = WV517S_REG_DRIVE_MODE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int wv517s_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct wv517s_device *wv517s =
		container_of(ctrl->handler, struct wv517s_device, ctrl_handler);
	struct device *dev = wv517s->sd.dev;
	int pm_ret;
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	/* Runtime resume restores controls while the PM state is RPM_RESUMING. */
	pm_ret = pm_runtime_get_if_active(dev);
	if (!pm_ret && !wv517s->resuming)
		return 0;
	if (pm_ret < 0)
		return pm_ret;

	ret = regmap_write(wv517s->regmap, WV517S_REG_FOCUS, ctrl->val);

	if (pm_ret > 0)
		pm_runtime_put(dev);

	return ret;
}

static const struct v4l2_ctrl_ops wv517s_ctrl_ops = {
	.s_ctrl = wv517s_set_ctrl,
};

static int wv517s_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int wv517s_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops wv517s_internal_ops = {
	.open = wv517s_open,
	.close = wv517s_close,
};

static const struct v4l2_subdev_ops wv517s_subdev_ops = { };

static int wv517s_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct wv517s_device *wv517s = to_wv517s(sd);
	int ret;

	/* Restore the vendor-recommended 12.6 ms ringing-control mode. */
	ret = regmap_write(wv517s->regmap, WV517S_REG_DRIVE_MODE,
			   WV517S_DRIVE_MODE_12_6_MS);
	if (ret)
		return ret;

	wv517s->resuming = true;
	ret = v4l2_ctrl_handler_setup(&wv517s->ctrl_handler);
	wv517s->resuming = false;

	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(wv517s_pm_ops, NULL, wv517s_resume, NULL);

static int wv517s_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wv517s_device *wv517s;
	int ret;

	wv517s = devm_kzalloc(dev, sizeof(*wv517s), GFP_KERNEL);
	if (!wv517s)
		return -ENOMEM;

	wv517s->regmap = devm_regmap_init_i2c(client, &wv517s_regmap_config);
	if (IS_ERR(wv517s->regmap))
		return dev_err_probe(dev, PTR_ERR(wv517s->regmap),
				     "failed to initialize regmap\n");

	v4l2_i2c_subdev_init(&wv517s->sd, client, &wv517s_subdev_ops);
	wv517s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	wv517s->sd.internal_ops = &wv517s_internal_ops;
	wv517s->sd.entity.function = MEDIA_ENT_F_LENS;

	v4l2_ctrl_handler_init(&wv517s->ctrl_handler, 1);
	v4l2_ctrl_new_std(&wv517s->ctrl_handler, &wv517s_ctrl_ops,
			  V4L2_CID_FOCUS_ABSOLUTE, 0,
			  WV517S_MAX_FOCUS_POSITION, 1,
			  WV517S_DEFAULT_FOCUS_POSITION);
	if (wv517s->ctrl_handler.error) {
		ret = wv517s->ctrl_handler.error;
		goto err_free_ctrl_handler;
	}
	wv517s->sd.ctrl_handler = &wv517s->ctrl_handler;

	ret = media_entity_pads_init(&wv517s->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrl_handler;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = wv517s_resume(dev);
	if (ret)
		goto err_disable_pm;

	ret = v4l2_async_register_subdev(&wv517s->sd);
	if (ret)
		goto err_disable_pm;

	pm_runtime_idle(dev);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	media_entity_cleanup(&wv517s->sd.entity);
err_free_ctrl_handler:
	v4l2_ctrl_handler_free(&wv517s->ctrl_handler);

	return ret;
}

static void wv517s_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct wv517s_device *wv517s = to_wv517s(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	v4l2_ctrl_handler_free(&wv517s->ctrl_handler);
	media_entity_cleanup(&sd->entity);
}

static const struct i2c_device_id wv517s_id_table[] = {
	{ .name = "wv517s" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wv517s_id_table);

static struct i2c_driver wv517s_i2c_driver = {
	.driver = {
		.name = "wv517s",
		.pm = pm_ptr(&wv517s_pm_ops),
	},
	.probe = wv517s_probe,
	.remove = wv517s_remove,
	.id_table = wv517s_id_table,
};
module_i2c_driver(wv517s_i2c_driver);

MODULE_AUTHOR("Maurizio Casciano <mauriziocasciano7@gmail.com>");
MODULE_DESCRIPTION("WV517S VCM driver");
MODULE_LICENSE("GPL");
