/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file @brief mutex kernel services
 *
 * This module contains routines for handling mutex locking and unlocking.
 *
 * Mutexes implement a priority inheritance algorithm that boosts the priority
 * level of the owning thread to match the priority level of the highest
 * priority thread waiting on the mutex.
 *
 * Each mutex that contributes to priority inheritance must be released in the
 * reverse order in which it was acquired.  Furthermore each subsequent mutex
 * that contributes to raising the owning thread's priority level must be
 * acquired at a point after the most recent "bumping" of the priority level.
 *
 * For example, if thread A has two mutexes contributing to the raising of its
 * priority level, the second mutex M2 must be acquired by thread A after
 * thread A's priority level was bumped due to owning the first mutex M1.
 * When releasing the mutex, thread A must release M2 before it releases M1.
 * Failure to follow this nested model may result in threads running at
 * unexpected priority levels (too high, or too low).
 */

#include <kernel.h>
#include <kernel_structs.h>
#include <toolchain.h>
#include <ksched.h>
#include <wait_q.h>
#include <errno.h>
#include <init.h>
#include <syscall_handler.h>
#include <tracing/tracing.h>
#include <sys/check.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

/* We use a global spinlock here because some of the synchronization
 * is protecting things like owner thread priorities which aren't
 * "part of" a single k_mutex.  Should move those bits of the API
 * under the scheduler lock so we can break this up.
 */
static struct k_spinlock lock;

int z_impl_k_mutex_init(struct k_mutex *mutex)
{
	mutex->owner = NULL;
	mutex->lock_count = 0U;

	z_waitq_init(&mutex->wait_q);

	z_object_init(mutex);

	SYS_PORT_TRACING_OBJ_INIT(k_mutex, mutex, 0);

	return 0;
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_k_mutex_init(struct k_mutex *mutex)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(mutex, K_OBJ_MUTEX));
	return z_impl_k_mutex_init(mutex);
}
#include <syscalls/k_mutex_init_mrsh.c>
#endif

static int32_t new_prio_for_inheritance(int32_t target, int32_t limit)
{
	int new_prio = z_is_prio_higher(target, limit) ? target : limit;

	new_prio = z_get_new_prio_with_ceiling(new_prio);

	return new_prio;
}

static bool adjust_owner_prio(struct k_mutex *mutex, int32_t new_prio)
{
	if (mutex->owner->base.prio != new_prio) {

		LOG_DBG("%p (ready (y/n): %c) prio changed to %d (was %d)",
			mutex->owner, z_is_thread_ready(mutex->owner) ?
			'y' : 'n',
			new_prio, mutex->owner->base.prio);

		return z_set_prio(mutex->owner, new_prio);
	}
	return false;
}

int z_impl_k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	int new_prio;
	k_spinlock_key_t key;
	bool resched = false;

	__ASSERT(!arch_is_in_isr(), "mutexes cannot be used inside ISRs");

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_mutex, lock, mutex, timeout);

	key = k_spin_lock(&lock);

	if (likely((mutex->lock_count == 0U) || (mutex->owner == _current))) {

		mutex->owner_orig_prio = (mutex->lock_count == 0U) ?
					_current->base.prio :
					mutex->owner_orig_prio;

		mutex->lock_count++;
		mutex->owner = _current;

		LOG_DBG("%p took mutex %p, count: %d, orig prio: %d",
			_current, mutex, mutex->lock_count,
			mutex->owner_orig_prio);

		k_spin_unlock(&lock, key);

		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, 0);

		return 0;
	}

	if (unlikely(K_TIMEOUT_EQ(timeout, K_NO_WAIT))) {
		k_spin_unlock(&lock, key);

		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, -EBUSY);

		return -EBUSY;
	}

	SYS_PORT_TRACING_OBJ_FUNC_BLOCKING(k_mutex, lock, mutex, timeout);

	new_prio = new_prio_for_inheritance(_current->base.prio,
					    mutex->owner->base.prio);

	LOG_DBG("adjusting prio up on mutex %p", mutex);

	if (z_is_prio_higher(new_prio, mutex->owner->base.prio)) {
		resched = adjust_owner_prio(mutex, new_prio);
	}

	int got_mutex = z_pend_curr(&lock, key, &mutex->wait_q, timeout);

	LOG_DBG("on mutex %p got_mutex value: %d", mutex, got_mutex);

	LOG_DBG("%p got mutex %p (y/n): %c", _current, mutex,
		got_mutex ? 'y' : 'n');

	if (got_mutex == 0) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, 0);
		return 0;
	}

	/* timed out */

	LOG_DBG("%p timeout on mutex %p", _current, mutex);

	key = k_spin_lock(&lock);

	struct k_thread *waiter = z_waitq_head(&mutex->wait_q);

	new_prio = (waiter != NULL) ?
		new_prio_for_inheritance(waiter->base.prio, mutex->owner_orig_prio) :
		mutex->owner_orig_prio;

	LOG_DBG("adjusting prio down on mutex %p", mutex);

	resched = adjust_owner_prio(mutex, new_prio) || resched;

	if (resched) {
		z_reschedule(&lock, key);
	} else {
		k_spin_unlock(&lock, key);
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, -EAGAIN);

	return -EAGAIN;
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_k_mutex_lock(struct k_mutex *mutex,
				      k_timeout_t timeout)
{
	Z_OOPS(Z_SYSCALL_OBJ(mutex, K_OBJ_MUTEX));
	return z_impl_k_mutex_lock(mutex, timeout);
}
#include <syscalls/k_mutex_lock_mrsh.c>
#endif

int z_impl_k_mutex_unlock(struct k_mutex *mutex)
{
	struct k_thread *new_owner;

	__ASSERT(!arch_is_in_isr(), "mutexes cannot be used inside ISRs");

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_mutex, unlock, mutex);

	CHECKIF(mutex->owner == NULL) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, -EINVAL);

		return -EINVAL;
	}
	/*
	 * The current thread does not own the mutex.
	 */
	CHECKIF(mutex->owner != _current) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, -EPERM);

		return -EPERM;
	}

	/*
	 * Attempt to unlock a mutex which is unlocked. mutex->lock_count
	 * cannot be zero if the current thread is equal to mutex->owner,
	 * therefore no underflow check is required. Use assert to catch
	 * undefined behavior.
	 */
	__ASSERT_NO_MSG(mutex->lock_count > 0U);

	LOG_DBG("mutex %p lock_count: %d", mutex, mutex->lock_count);

	/*
	 * If we are the owner and count is greater than 1, then decrement
	 * the count and return and keep current thread as the owner.
	 */
	if (mutex->lock_count > 1U) {
		mutex->lock_count--;
		goto k_mutex_unlock_return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	adjust_owner_prio(mutex, mutex->owner_orig_prio);

	/* Get the new owner, if any */
	new_owner = z_unpend_first_thread(&mutex->wait_q);

	mutex->owner = new_owner;

	LOG_DBG("new owner of mutex %p: %p (prio: %d)",
		mutex, new_owner, new_owner ? new_owner->base.prio : -1000);

	if (new_owner != NULL) {
		/*
		 * new owner is already of higher or equal prio than first
		 * waiter since the wait queue is priority-based: no need to
		 * adjust its priority
		 */
		mutex->owner_orig_prio = new_owner->base.prio;
		arch_thread_return_value_set(new_owner, 0);
		z_ready_thread(new_owner);
		z_reschedule(&lock, key);
	} else {
		mutex->lock_count = 0U;
		k_spin_unlock(&lock, key);
	}


k_mutex_unlock_return:
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, 0);

	return 0;
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_k_mutex_unlock(struct k_mutex *mutex)
{
	Z_OOPS(Z_SYSCALL_OBJ(mutex, K_OBJ_MUTEX));
	return z_impl_k_mutex_unlock(mutex);
}
#include <syscalls/k_mutex_unlock_mrsh.c>
#endif
