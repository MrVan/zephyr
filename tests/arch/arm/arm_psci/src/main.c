/*
 * Copyright (c) 2020 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <drivers/psci.h>

#define PSCI_DEV_NAME "PSCI"

void test_psci_func(void)
{
	const struct device *psci;
	uint32_t ver;

	psci = device_get_binding(PSCI_DEV_NAME);
	zassert_not_null(psci, "Could not get psci device");

	ver = psci_get_version(psci);
	zassert_false((PSCI_VERSION_MAJOR(ver) == 0 &&
		       PSCI_VERSION_MINOR(ver) < 2),
		       "Wrong PSCI firware version");
}

void test_main(void)
{
	ztest_test_suite(psci_func,
		ztest_unit_test(test_psci_func));
	ztest_run_test_suite(psci_func);
}
