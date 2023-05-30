/*
 * Copyright (c) 2020 Tobias Svehagen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>

#include <zephyr/net/socket.h>
#include <zephyr/posix/poll.h>
#include <zephyr/posix/sys/eventfd.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/ztest.h>

#define TESTVAL 10

ZTEST_SUITE(eventfd, NULL, NULL, NULL, NULL, NULL);

static int is_blocked(int fd, short *event)
{
	struct pollfd pfd;
	int ret;

	pfd.fd = fd;
	pfd.events = *event;

	ret = poll(&pfd, 1, 0);
	zassert_true(ret >= 0, "poll failed %d", ret);

	*event = pfd.revents;

	return ret == 0;
}

ZTEST(eventfd, test_eventfd)
{
	int fd = eventfd(0, 0);

	zassert_true(fd >= 0, "fd == %d", fd);

	close(fd);
}

ZTEST(eventfd, test_read_nonblock)
{
	eventfd_t val = 0;
	int fd, ret;

	fd = eventfd(0, EFD_NONBLOCK);
	zassert_true(fd >= 0, "fd == %d", fd);

	ret = eventfd_read(fd, &val);
	zassert_true(ret == -1, "read unset ret %d", ret);
	zassert_true(errno == EAGAIN, "errno %d", errno);

	ret = eventfd_write(fd, TESTVAL);
	zassert_true(ret == 0, "write ret %d", ret);

	ret = eventfd_read(fd, &val);
	zassert_true(ret == 0, "read set ret %d", ret);
	zassert_true(val == TESTVAL, "red set val %d", val);

	ret = eventfd_read(fd, &val);
	zassert_true(ret == -1, "read subsequent ret %d val %d", ret, val);
	zassert_true(errno == EAGAIN, "errno %d", errno);

	close(fd);
}

ZTEST(eventfd, test_write_then_read)
{
	eventfd_t val;
	int fd, ret;

	fd = eventfd(0, 0);
	zassert_true(fd >= 0, "fd == %d", fd);

	ret = eventfd_write(fd, 3);
	zassert_true(ret == 0, "write ret %d", ret);

	ret = eventfd_write(fd, 2);
	zassert_true(ret == 0, "write ret %d", ret);

	ret = eventfd_read(fd, &val);
	zassert_true(ret == 0, "read ret %d", ret);
	zassert_true(val == 5, "val == %d", val);

	close(fd);

	/* Test EFD_SEMAPHORE */

	fd = eventfd(0, EFD_SEMAPHORE);
	zassert_true(fd >= 0, "fd == %d", fd);

	ret = eventfd_write(fd, 3);
	zassert_true(ret == 0, "write ret %d", ret);

	ret = eventfd_write(fd, 2);
	zassert_true(ret == 0, "write ret %d", ret);

	ret = eventfd_read(fd, &val);
	zassert_true(ret == 0, "read ret %d", ret);
	zassert_true(val == 1, "val == %d", val);

	close(fd);
}

ZTEST(eventfd, test_poll_timeout)
{
	struct pollfd pfd;
	int fd, ret;

	fd = eventfd(0, 0);
	zassert_true(fd >= 0, "fd == %d", fd);

	pfd.fd = fd;
	pfd.events = POLLIN;

	ret = poll(&pfd, 1, 500);
	zassert_true(ret == 0, "poll ret %d", ret);

	close(fd);
}

static void eventfd_poll_unset_common(int fd)
{
	eventfd_t val = 0;
	short event;
	int ret;

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 1, "eventfd not blocked with initval == 0");

	ret = eventfd_write(fd, TESTVAL);
	zassert_equal(ret, 0, "write ret %d", ret);

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 0, "eventfd blocked after write");
	zassert_equal(event, POLLIN, "POLLIN not set");

	ret = eventfd_write(fd, TESTVAL);
	zassert_equal(ret, 0, "write ret %d", ret);

	ret = eventfd_read(fd, &val);
	zassert_equal(ret, 0, "read ret %d", ret);
	zassert_equal(val, 2*TESTVAL, "val == %d, expected %d", val, TESTVAL);

	/* eventfd shall block on subsequent reads */

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 1, "eventfd not blocked after read");
}

ZTEST(eventfd, test_unset_poll_event_block)
{
	int fd;

	fd = eventfd(0, 0);
	zassert_true(fd >= 0, "fd == %d", fd);

	eventfd_poll_unset_common(fd);

	close(fd);
}

ZTEST(eventfd, test_unset_poll_event_nonblock)
{
	int fd;

	fd = eventfd(0, EFD_NONBLOCK);
	zassert_true(fd >= 0, "fd == %d", fd);

	eventfd_poll_unset_common(fd);

	close(fd);
}

static void eventfd_poll_set_common(int fd)
{
	eventfd_t val = 0;
	short event;
	int ret;

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 0, "eventfd is blocked with initval != 0");

	ret = eventfd_read(fd, &val);
	zassert_equal(ret, 0, "read ret %d", ret);
	zassert_equal(val, TESTVAL, "val == %d", val);

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 1, "eventfd is not blocked after read");
}

ZTEST(eventfd, test_set_poll_event_block)
{
	int fd;

	fd = eventfd(TESTVAL, 0);
	zassert_true(fd >= 0, "fd == %d", fd);

	eventfd_poll_set_common(fd);

	close(fd);
}

ZTEST(eventfd, test_set_poll_event_nonblock)
{
	int fd;

	fd = eventfd(TESTVAL, EFD_NONBLOCK);
	zassert_true(fd >= 0, "fd == %d", fd);

	eventfd_poll_set_common(fd);

	close(fd);
}

ZTEST(eventfd, test_overflow)
{
	short event;
	int fd, ret;

	fd = eventfd(0, EFD_NONBLOCK);
	zassert_true(fd >= 0, "fd == %d", fd);

	event = POLLOUT;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 0, "eventfd write blocked with initval == 0");

	ret = eventfd_write(fd, UINT64_MAX);
	zassert_equal(ret, -1, "fd == %d", fd);
	zassert_equal(errno, EINVAL, "did not get EINVAL");

	ret = eventfd_write(fd, UINT64_MAX-1);
	zassert_equal(ret, 0, "fd == %d", fd);

	event = POLLOUT;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 1, "eventfd write not blocked with cnt == UINT64_MAX-1");

	ret = eventfd_write(fd, 1);
	zassert_equal(ret, -1, "fd == %d", fd);
	zassert_equal(errno, EAGAIN, "did not get EINVAL");

	close(fd);
}

ZTEST(eventfd, test_zero_shall_not_unblock)
{
	short event;
	int fd, ret;

	fd = eventfd(0, 0);
	zassert_true(fd >= 0, "fd == %d", fd);

	ret = eventfd_write(fd, 0);
	zassert_equal(ret, 0, "fd == %d", fd);

	event = POLLIN;
	ret = is_blocked(fd, &event);
	zassert_equal(ret, 1, "eventfd unblocked by zero");

	close(fd);
}
