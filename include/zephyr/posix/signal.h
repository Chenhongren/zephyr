/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_SIGNAL_H_
#define ZEPHYR_INCLUDE_POSIX_SIGNAL_H_

#include "posix_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_POSIX_SIGNAL
#define SIGHUP    1  /**< Hangup */
#define SIGINT    2  /**< Interrupt */
#define SIGQUIT   3  /**< Quit */
#define SIGILL    4  /**< Illegal instruction */
#define SIGTRAP   5  /**< Trace/breakpoint trap */
#define SIGABRT   6  /**< Aborted */
#define SIGBUS    7  /**< Bus error */
#define SIGFPE    8  /**< Arithmetic exception */
#define SIGKILL   9  /**< Killed */
#define SIGUSR1   10 /**< User-defined signal 1 */
#define SIGSEGV   11 /**< Invalid memory reference */
#define SIGUSR2   12 /**< User-defined signal 2 */
#define SIGPIPE   13 /**< Broken pipe */
#define SIGALRM   14 /**< Alarm clock */
#define SIGTERM   15 /**< Terminated */
/* 16 not used */
#define SIGCHLD   17 /**< Child status changed */
#define SIGCONT   18 /**< Continued */
#define SIGSTOP   19 /**< Stop executing */
#define SIGTSTP   20 /**< Stopped */
#define SIGTTIN   21 /**< Stopped (read) */
#define SIGTTOU   22 /**< Stopped (write) */
#define SIGURG    23 /**< Urgent I/O condition */
#define SIGXCPU   24 /**< CPU time limit exceeded */
#define SIGXFSZ   25 /**< File size limit exceeded */
#define SIGVTALRM 26 /**< Virtual timer expired */
#define SIGPROF   27 /**< Profiling timer expired */
/* 28 not used */
#define SIGPOLL   29 /**< Pollable event occurred */
/* 30 not used */
#define SIGSYS    31 /**< Bad system call */

#define SIGRTMIN 32
#define SIGRTMAX (SIGRTMIN + CONFIG_POSIX_RTSIG_MAX)
#define _NSIG (SIGRTMAX + 1)

BUILD_ASSERT(CONFIG_POSIX_RTSIG_MAX >= 0);

typedef struct {
	unsigned long sig[DIV_ROUND_UP(_NSIG, BITS_PER_LONG)];
} sigset_t;

char *strsignal(int signum);
int sigemptyset(sigset_t *set);
int sigfillset(sigset_t *set);
int sigaddset(sigset_t *set, int signo);
int sigdelset(sigset_t *set, int signo);
int sigismember(const sigset_t *set, int signo);
#endif /* CONFIG_POSIX_SIGNAL */

#ifndef SIGEV_NONE
#define SIGEV_NONE 1
#endif

#ifndef SIGEV_SIGNAL
#define SIGEV_SIGNAL 2
#endif

#ifndef SIGEV_THREAD
#define SIGEV_THREAD 3
#endif

typedef int	sig_atomic_t;		/* Atomic entity type (ANSI) */

union sigval {
	int sival_int;
	void *sival_ptr;
};

struct sigevent {
	int sigev_notify;
	int sigev_signo;
	union sigval sigev_value;
	void (*sigev_notify_function)(union sigval val);
	pthread_attr_t *sigev_notify_attributes;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_POSIX_SIGNAL_H_ */
