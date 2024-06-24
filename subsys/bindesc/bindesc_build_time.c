/*
 * Copyright (c) 2023 Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bindesc.h>

/* Include generated header */
#include <bindesc_build_time.h>

#if defined(CONFIG_BINDESC_BUILD_TIME_YEAR)
BINDESC_UINT_DEFINE(build_time_year, BINDESC_ID_BUILD_TIME_YEAR, BUILD_TIME_YEAR);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_YEAR) */

#if defined(CONFIG_BINDESC_BUILD_TIME_MONTH)
BINDESC_UINT_DEFINE(build_time_month, BINDESC_ID_BUILD_TIME_MONTH, BUILD_TIME_MONTH);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_MONTH) */

#if defined(CONFIG_BINDESC_BUILD_TIME_DAY)
BINDESC_UINT_DEFINE(build_time_day, BINDESC_ID_BUILD_TIME_DAY, BUILD_TIME_DAY);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_DAY) */

#if defined(CONFIG_BINDESC_BUILD_TIME_HOUR)
BINDESC_UINT_DEFINE(build_time_hour, BINDESC_ID_BUILD_TIME_HOUR, BUILD_TIME_HOUR);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_HOUR) */

#if defined(CONFIG_BINDESC_BUILD_TIME_MINUTE)
BINDESC_UINT_DEFINE(build_time_minute, BINDESC_ID_BUILD_TIME_MINUTE, BUILD_TIME_MINUTE);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_MINUTE) */

#if defined(CONFIG_BINDESC_BUILD_TIME_SECOND)
BINDESC_UINT_DEFINE(build_time_second, BINDESC_ID_BUILD_TIME_SECOND, BUILD_TIME_SECOND);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_SECOND) */

#if defined(CONFIG_BINDESC_BUILD_TIME_UNIX)
BINDESC_UINT_DEFINE(build_time_unix, BINDESC_ID_BUILD_TIME_UNIX, BUILD_TIME_UNIX);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_UNIX) */

#if defined(CONFIG_BINDESC_BUILD_DATE_TIME_STRING)
BINDESC_STR_DEFINE(build_date_time_string, BINDESC_ID_BUILD_DATE_TIME_STRING,
			BUILD_DATE_TIME_STRING);
#endif /* defined(CONFIG_BINDESC_BUILD_DATE_TIME_STRING) */

#if defined(CONFIG_BINDESC_BUILD_DATE_STRING)
BINDESC_STR_DEFINE(build_date_string, BINDESC_ID_BUILD_DATE_STRING, BUILD_DATE_STRING);
#endif /* defined(CONFIG_BINDESC_BUILD_DATE_STRING) */

#if defined(CONFIG_BINDESC_BUILD_TIME_STRING)
BINDESC_STR_DEFINE(build_time_string, BINDESC_ID_BUILD_TIME_STRING, BUILD_TIME_STRING);
#endif /* defined(CONFIG_BINDESC_BUILD_TIME_STRING) */
