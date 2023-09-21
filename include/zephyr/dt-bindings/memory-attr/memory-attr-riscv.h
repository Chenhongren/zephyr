/*
 * Copyright (c) 2023 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MEM_ATTR_RISCV_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MEM_ATTR_RISCV_H_

#include <zephyr/sys/util_macro.h>
#include <zephyr/dt-bindings/memory-attr/memory-attr.h>

/*
 * Architecture specific RISCV related attributes.
 */
#define DT_MEM_RISCV_MASK			DT_MEM_ARCH_ATTR_MASK
#define DT_MEM_RISCV_GET(x)			((x) & DT_MEM_RISCV_MASK)
#define DT_MEM_RISCV(x)				((x) << DT_MEM_ARCH_ATTR_SHIFT)

#define  ATTR_RISCV_TYPE_MAIN			BIT(0)
#define  ATTR_RISCV_TYPE_IO			BIT(1)
#define  ATTR_RISCV_TYPE_EMPTY			BIT(2)
#define  ATTR_RISCV_AMO_SWAP			BIT(3)
#define  ATTR_RISCV_AMO_LOGICAL			BIT(4)
#define  ATTR_RISCV_AMO_ARITHMETIC		BIT(5)
#define  ATTR_RISCV_IO_IDEMPOTENT_READ		BIT(6)
#define  ATTR_RISCV_IO_IDEMPOTENT_WRITE		BIT(7)

#define DT_MEM_RISCV_TYPE_MAIN			DT_MEM_RISCV(ATTR_RISCV_TYPE_MAIN)
#define DT_MEM_RISCV_TYPE_IO			DT_MEM_RISCV(ATTR_RISCV_TYPE_IO)
#define DT_MEM_RISCV_TYPE_EMPTY			DT_MEM_RISCV(ATTR_RISCV_TYPE_EMPTY)
#define DT_MEM_RISCV_AMO_SWAP			DT_MEM_RISCV(ATTR_RISCV_AMO_SWAP)
#define DT_MEM_RISCV_AMO_LOGICAL		DT_MEM_RISCV(ATTR_RISCV_AMO_LOGICAL)
#define DT_MEM_RISCV_AMO_ARITHMETIC		DT_MEM_RISCV(ATTR_RISCV_AMO_ARITHMETIC)
#define DT_MEM_RISCV_IO_IDEMPOTENT_READ		DT_MEM_RISCV(ATTR_RISCV_IO_IDEMPOTENT_READ)
#define DT_MEM_RISCV_IO_IDEMPOTENT_WRITE	DT_MEM_RISCV(ATTR_RISCV_IO_IDEMPOTENT_WRITE)
#define DT_MEM_RISCV_UNKNOWN			DT_MEM_ARCH_ATTR_UNKNOWN

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MEM_ATTR_RISCV_H_ */
