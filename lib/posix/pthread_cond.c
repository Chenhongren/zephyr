/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <ksched.h>
#include <zephyr/wait_q.h>
#include <zephyr/posix/pthread.h>

#include "posix_internal.h"

extern struct k_spinlock z_pthread_spinlock;

int64_t timespec_to_timeoutms(const struct timespec *abstime);

static int cond_wait(pthread_cond_t *cv, pthread_mutex_t *mu,
		     k_timeout_t timeout)
{
	int ret;
	k_spinlock_key_t key;
	struct posix_mutex *m;

	key = k_spin_lock(&z_pthread_spinlock);
	m = to_posix_mutex(mu);
	if (m == NULL) {
		k_spin_unlock(&z_pthread_spinlock, key);
		return EINVAL;
	}

	__ASSERT_NO_MSG(m->lock_count == 1U);
	m->lock_count = 0U;
	m->owner = NULL;
	_ready_one_thread(&m->wait_q);
	ret = z_sched_wait(&z_pthread_spinlock, key, &cv->wait_q, timeout, NULL);

	/* FIXME: this extra lock (and the potential context switch it
	 * can cause) could be optimized out.  At the point of the
	 * signal/broadcast, it's possible to detect whether or not we
	 * will be swapping back to this particular thread and lock it
	 * (i.e. leave the lock variable unchanged) on our behalf.
	 * But that requires putting scheduler intelligence into this
	 * higher level abstraction and is probably not worth it.
	 */
	pthread_mutex_lock(mu);

	return ret == -EAGAIN ? ETIMEDOUT : ret;
}

int pthread_cond_signal(pthread_cond_t *cv)
{
	z_sched_wake(&cv->wait_q, 0, NULL);
	return 0;
}

int pthread_cond_broadcast(pthread_cond_t *cv)
{
	z_sched_wake_all(&cv->wait_q, 0, NULL);
	return 0;
}

int pthread_cond_wait(pthread_cond_t *cv, pthread_mutex_t *mut)
{
	return cond_wait(cv, mut, K_FOREVER);
}

int pthread_cond_timedwait(pthread_cond_t *cv, pthread_mutex_t *mut,
			   const struct timespec *abstime)
{
	int32_t timeout = (int32_t)timespec_to_timeoutms(abstime);
	return cond_wait(cv, mut, K_MSEC(timeout));
}
