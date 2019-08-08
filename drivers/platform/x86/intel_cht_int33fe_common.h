/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common code for Intel Cherry Trail ACPI INT33FE pseudo device drivers
 * (microUSB and TypeC connector variants), header file
 *
 * Copyright (c) 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 */

#ifndef _INTEL_CHT_INT33FE_COMMON_H
#define _INTEL_CHT_INT33FE_COMMON_H

#include <linux/device.h>

enum int33fe_hw_type {
	INT33FE_HW_TYPEC,
	INT33FE_HW_MUSB,
};

int cht_int33fe_check_hw_compatible(struct device *dev,
				    enum int33fe_hw_type hw_type);

#endif /* _INTEL_CHT_INT33FE_COMMON_H */

