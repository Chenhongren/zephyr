/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include <zephyr/ztest.h>


ZTEST_SUITE(a1_1_tests, NULL, NULL, NULL, NULL, NULL);

int helper(void)
{
	char *s = malloc(100);

	s[0] = '!';
	s[1] = '\0';
	printf("string is: %s\n", s);
	return 0;
}

/**
 * @brief Test Asserts
 *
 * This test verifies various assert macros provided by ztest.
 *
 */
ZTEST(a1_1_tests, test_assert)
{
	helper();

	zassert_true(1, "1 was false");
	zassert_false(0, "0 was true");
	zassert_is_null(NULL, "NULL was not NULL");
	zassert_not_null("foo", "\"foo\" was NULL");
	zassert_equal(1, 1, "1 was not equal to 1");
	zassert_equal_ptr(NULL, NULL, "NULL was not equal to NULL");
}
