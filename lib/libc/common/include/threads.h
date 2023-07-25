/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_LIBC_COMMON_INCLUDE_THREADS_H_
#define ZEPHYR_LIB_LIBC_COMMON_INCLUDE_THREADS_H_

#include <time.h>

#include <machine/_threads.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*thrd_start_t)(void *arg);

enum {
	thrd_success,
#define thrd_success thrd_success
	thrd_nomem,
#define thrd_nomem thrd_nomem
	thrd_timedout,
#define thrd_timedout thrd_timedout
	thrd_busy,
#define thrd_busy thrd_busy
	thrd_error,
#define thrd_error thrd_error
};

int thrd_create(thrd_t *thr, thrd_start_t func, void *arg);
int thrd_equal(thrd_t lhs, thrd_t rhs);
thrd_t thrd_current(void);
int thrd_sleep(const struct timespec *duration, struct timespec *remaining);
void thrd_yield(void);
_Noreturn void thrd_exit(int res);
int thrd_detach(thrd_t thr);
int thrd_join(thrd_t thr, int *res);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_LIB_LIBC_COMMON_INCLUDE_THREADS_H_ */
