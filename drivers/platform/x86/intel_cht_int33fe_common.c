// SPDX-License-Identifier: GPL-2.0
/*
 * Common code for Intel Cherry Trail ACPI INT33FE pseudo device drivers
 * (microUSB and TypeC connector variants)
 *
 * Copyright (c) 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include "intel_cht_int33fe_common.h"

#define EXPECTED_PTYPE		4

static int cht_int33fe_i2c_res_filter(struct acpi_resource *ares, void *data)
{
	struct acpi_resource_i2c_serialbus *sb;
	int *count = data;

	if (i2c_acpi_get_i2c_resource(ares, &sb))
		(*count)++;

	return 1;
}

static int cht_int33fe_count_i2c_clients(struct device *dev)
{
	struct acpi_device *adev;
	LIST_HEAD(resource_list);
	int count = 0;

	adev = ACPI_COMPANION(dev);
	if (!adev)
		return -EINVAL;

	acpi_dev_get_resources(adev, &resource_list,
			       cht_int33fe_i2c_res_filter, &count);

	acpi_dev_free_resource_list(&resource_list);

	return count;
}

int cht_int33fe_check_hw_compatible(struct device *dev,
				    enum int33fe_hw_type hw_type)
{
	unsigned long long ptyp;
	acpi_status status;
	int i2c_expected;
	int ret;

	i2c_expected = (hw_type == INT33FE_HW_TYPEC) ? 4 : 2;

	status = acpi_evaluate_integer(ACPI_HANDLE(dev), "PTYP", NULL, &ptyp);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "Error getting PTYPE\n");
		return -ENODEV;
	}

	/*
	 * The same ACPI HID is used for different configurations check PTYP
	 * to ensure that we are dealing with the expected config.
	 */
	if (ptyp != EXPECTED_PTYPE)
		return -ENODEV;

	/* Check presence of INT34D3 (hardware-rev 3) expected for ptype == 4 */
	if (!acpi_dev_present("INT34D3", "1", 3)) {
		dev_err(dev, "Error PTYPE == %d, but no INT34D3 device\n",
			EXPECTED_PTYPE);
		return -ENODEV;
	}

	ret = cht_int33fe_count_i2c_clients(dev);
	if (ret < 0)
		return ret;

	if (ret != i2c_expected) {
		dev_info(dev, "I2C clients count (%d) is not %d, ignore (probably %s hardware)",
			 ret, i2c_expected,
			 (hw_type == INT33FE_HW_TYPEC) ? "microUSB" : "Type C");
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cht_int33fe_check_hw_compatible);

MODULE_DESCRIPTION("Intel Cherry Trail ACPI INT33FE pseudo device driver (common part)");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL v2");
