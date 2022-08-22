/*
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (C) 2010-2020 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/zephyr.h>
#include <stdlib.h>
#include <arm_math.h>
#include "../../common/benchmark_common.h"

#define PATTERN_LENGTH	(256)

static const q7_t input1[PATTERN_LENGTH] = {
	0x17, 0xCC, 0x33, 0xF2, 0x14, 0x01, 0x2A, 0xF2,
	0xEC, 0x22, 0x01, 0xF7, 0x2A, 0xB5, 0xCF, 0xED,
	0x2F, 0xE8, 0xCB, 0xE8, 0xF2, 0x08, 0xC6, 0xF2,
	0x34, 0x2F, 0xE1, 0xEF, 0xE8, 0xF1, 0xC1, 0xEF,
	0x3B, 0x23, 0x07, 0x01, 0xFE, 0x07, 0x03, 0xF7,
	0x45, 0xF5, 0xE9, 0xF5, 0x0F, 0x13, 0xF5, 0xF2,
	0x06, 0x1C, 0xF4, 0xFB, 0x12, 0xAF, 0x09, 0x0C,
	0xE8, 0x03, 0xF2, 0x10, 0x2D, 0xD3, 0x16, 0xF5,
	0x32, 0x0C, 0x59, 0xDA, 0x0C, 0x1C, 0xB5, 0x49,
	0x23, 0x05, 0xA1, 0x21, 0x1B, 0x1E, 0xCE, 0xC1,
	0xE0, 0x58, 0x22, 0x01, 0xDB, 0x05, 0x06, 0x30,
	0xCF, 0xCC, 0x0D, 0xA1, 0x38, 0x35, 0xFE, 0x19,
	0xD5, 0x01, 0x15, 0x37, 0x10, 0xDC, 0xF4, 0xD0,
	0x09, 0x0F, 0x1C, 0xA3, 0xCA, 0xF7, 0x29, 0x19,
	0x10, 0xF5, 0xFB, 0xD8, 0x0A, 0xA4, 0xD3, 0x08,
	0x44, 0xDF, 0x2F, 0x5E, 0xFA, 0xFC, 0x48, 0x0B,
	0xFA, 0xB9, 0x43, 0xF3, 0xCC, 0x6A, 0x51, 0x0B,
	0xF8, 0xC6, 0xD2, 0xE0, 0xD6, 0xFA, 0xE0, 0x1B,
	0xCC, 0x31, 0x03, 0xE1, 0x18, 0x11, 0x04, 0xCC,
	0xFA, 0xF6, 0xFE, 0x25, 0x0E, 0x34, 0x15, 0xE7,
	0xDD, 0xE3, 0xFE, 0xDE, 0x12, 0xCD, 0xE0, 0xF6,
	0xD6, 0xF5, 0xDE, 0x0D, 0x28, 0x09, 0xF2, 0xC2,
	0x1A, 0x54, 0xBB, 0xEB, 0xB7, 0xDB, 0x12, 0x05,
	0x0F, 0xEE, 0x48, 0x00, 0x02, 0x11, 0xCA, 0xA9,
	0x0C, 0x7F, 0x1D, 0x34, 0x13, 0xC8, 0x15, 0xFA,
	0xDE, 0x39, 0xFE, 0x02, 0x09, 0x07, 0x2C, 0xF4,
	0xEE, 0xE9, 0x11, 0x22, 0xE2, 0xAB, 0x37, 0xC1,
	0xEA, 0x00, 0x3E, 0xEC, 0x23, 0x03, 0xEE, 0xBB,
	0xE2, 0x6D, 0x17, 0x00, 0xEF, 0x2F, 0x19, 0x31,
	0x05, 0xCB, 0xFF, 0x0A, 0x29, 0xA6, 0xD9, 0x42,
	0x1A, 0xDE, 0x21, 0x31, 0x3E, 0x34, 0xFB, 0x11,
	0xE3, 0x8F, 0xF1, 0xC6, 0xEF, 0x51, 0x01, 0xE0
	};

static const q7_t input2[PATTERN_LENGTH] = {
	0x0F, 0x06, 0xDE, 0x12, 0xE8, 0x03, 0xF2, 0xEC,
	0x2C, 0xC7, 0x0C, 0xF3, 0xF6, 0xE2, 0xFB, 0x14,
	0xC0, 0xDD, 0xE5, 0xDB, 0xD6, 0xCE, 0x19, 0xEB,
	0xF7, 0x0D, 0x20, 0x0C, 0x1B, 0xB9, 0xF4, 0xF2,
	0x02, 0xE9, 0x21, 0x3F, 0xE4, 0x02, 0x75, 0x0D,
	0x31, 0x0F, 0x38, 0x16, 0xEA, 0x14, 0x7F, 0xEC,
	0x06, 0x20, 0xE7, 0x0C, 0x27, 0xDA, 0x02, 0xF2,
	0xF8, 0xF4, 0xFE, 0x36, 0xCC, 0xCC, 0x0B, 0xCF,
	0xFD, 0x01, 0xDD, 0xF7, 0x30, 0x18, 0xED, 0x34,
	0xE7, 0xFF, 0x05, 0xE5, 0x1E, 0xE7, 0xDB, 0xE1,
	0xDC, 0x17, 0xE1, 0x0B, 0x1D, 0xD6, 0x01, 0xF2,
	0x17, 0x24, 0x45, 0xD8, 0xD8, 0xF2, 0xE9, 0xE1,
	0xD1, 0xEF, 0x07, 0x2D, 0x07, 0xF8, 0x07, 0xB7,
	0x13, 0x3C, 0xCA, 0xDB, 0x00, 0xC8, 0x2B, 0xFC,
	0xE4, 0xFF, 0x33, 0x0B, 0x1A, 0x2C, 0xE3, 0x05,
	0x1D, 0x16, 0xED, 0xB3, 0x10, 0xFF, 0xD4, 0x09,
	0x11, 0xFD, 0x01, 0x4A, 0xF8, 0xF4, 0x45, 0x15,
	0x4F, 0x08, 0x17, 0x13, 0x0B, 0xCE, 0xE9, 0xB4,
	0x07, 0x0B, 0xD5, 0x15, 0xF3, 0xE9, 0x0E, 0xE7,
	0xF5, 0xFA, 0x10, 0x01, 0xAD, 0xE5, 0xF3, 0xFC,
	0xE6, 0xFE, 0xCE, 0xDE, 0x40, 0xE7, 0x1F, 0x3D,
	0x0E, 0x20, 0x19, 0xFC, 0xF9, 0xF7, 0xC1, 0xDE,
	0x48, 0xFB, 0x07, 0x09, 0x20, 0x13, 0x23, 0xEA,
	0xED, 0xF3, 0x0F, 0x2C, 0xE1, 0x22, 0xD5, 0xD6,
	0xB7, 0x69, 0x05, 0x58, 0x23, 0xD1, 0xF5, 0xA0,
	0xE3, 0x10, 0x26, 0xFE, 0x19, 0x3B, 0x1D, 0xF6,
	0xFE, 0x2E, 0x0D, 0xF5, 0x38, 0xF4, 0xCB, 0xE3,
	0x4D, 0xFE, 0xE6, 0x6B, 0xF5, 0xCC, 0xCA, 0xDA,
	0xD5, 0xEA, 0x1D, 0x35, 0x06, 0x0B, 0x1E, 0x17,
	0xC5, 0xD4, 0x04, 0xF0, 0x03, 0xFE, 0x04, 0xD3,
	0x1E, 0xD2, 0x0B, 0x08, 0xCD, 0xC1, 0xDF, 0xFE,
	0xF7, 0xDB, 0xD0, 0x03, 0x4F, 0xE7, 0x0D, 0x1A
	};

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_add_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_add_q7(input1, input2, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_sub_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_sub_q7(input1, input2, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_mult_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_mult_q7(input1, input2, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_abs_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_abs_q7(input1, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_negate_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_negate_q7(input1, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_offset_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_offset_q7(input1, 1.0, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_scale_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q7_t *output;

	/* Allocate output buffer */
	output = malloc(PATTERN_LENGTH * sizeof(q7_t));
	zassert_not_null(output, "output buffer allocation failed");

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_scale_q7(input1, 0x45, 1, output, PATTERN_LENGTH);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Free output buffer */
	free(output);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST(basicmath_q7_benchmark, test_benchmark_vec_dot_prod_q7)
{
	uint32_t irq_key, timestamp, timespan;
	q31_t output;

	/* Begin benchmark */
	benchmark_begin(&irq_key, &timestamp);

	/* Execute function */
	arm_dot_prod_q7(input1, input2, PATTERN_LENGTH, &output);

	/* End benchmark */
	timespan = benchmark_end(irq_key, timestamp);

	/* Print result */
	TC_PRINT(BENCHMARK_TYPE " = %u\n", timespan);
}

ZTEST_SUITE(basicmath_q7_benchmark, NULL, NULL, NULL, NULL, NULL);
