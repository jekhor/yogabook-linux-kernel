// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Cherry Trail ACPI INT33FE pseudo device driver for devices with
 * microUSB connector (e.g. without of FUSB302 USB Type-C controller)
 *
 * Copyright (C) 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * At least one Intel Cherry Trail based device which ship with Windows 10
 * (Lenovo YogaBook YB1-X91L/F tablet), have this weird INT33FE ACPI device
 * with a CRS table with 2 I2cSerialBusV2 resources, for 2 different chips
 * attached to various i2c busses:
 * 1. The Whiskey Cove pmic, which is also described by the INT34D3 ACPI device
 * 2. TI BQ27542 Fuel Gauge Controller
 *
 * So this driver is a stub / pseudo driver whose only purpose is to
 * instantiate i2c-client for battery fuel gauge, so that standard i2c driver
 * for these chip can bind to the it.
 */

#define DEBUG

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/pd.h>

#include "intel_cht_int33fe_common.h"

struct cht_int33fe_data {
	struct i2c_client *battery_fg;
};

static const char * const bq27xxx_suppliers[] = { "bq25890-charger" };

static const struct property_entry bq27xxx_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("supplied-from", bq27xxx_suppliers),
	{ }
};

static int cht_int33fe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_board_info board_info;
	struct cht_int33fe_data *data;
	int ret;

	ret = cht_int33fe_check_hw_compatible(dev, INT33FE_HW_TYPEB);
	if (ret < 0)
		return ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(&board_info, 0, sizeof(board_info));
	strscpy(board_info.type, "bq27542", ARRAY_SIZE(board_info.type));
	board_info.dev_name = "bq27542";
	board_info.properties = bq27xxx_props;
	data->battery_fg = i2c_acpi_new_device(dev, 1, &board_info);

	if (IS_ERR(data->battery_fg)) {
		dev_err(dev, "Failed to register battery fuel gauge: %ld\n",
			PTR_ERR(data->battery_fg));
		return PTR_ERR(data->battery_fg);
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int cht_int33fe_remove(struct platform_device *pdev)
{
	struct cht_int33fe_data *data = platform_get_drvdata(pdev);

	i2c_unregister_device(data->battery_fg);

	return 0;
}

static const struct acpi_device_id cht_int33fe_acpi_ids[] = {
	{ "INT33FE", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, cht_int33fe_acpi_ids);

static struct platform_driver cht_int33fe_driver = {
	.driver	= {
		.name = "Intel Cherry Trail ACPI INT33FE USB type B driver",
		.acpi_match_table = ACPI_PTR(cht_int33fe_acpi_ids),
	},
	.probe = cht_int33fe_probe,
	.remove = cht_int33fe_remove,
};

module_platform_driver(cht_int33fe_driver);

MODULE_DESCRIPTION("Intel Cherry Trail ACPI INT33FE pseudo device driver (typeB conn.)");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL v2");
