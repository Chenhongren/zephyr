/*
 * Copyright (c) 2018, Intel Corporation
 * All rights reserved.
 *
 * Author:	Seppo Ingalsuo <seppo.ingalsuo@linux.intel.com>
 *		Sathish Kuttan <sathish.k.kuttan@intel.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

#include "pdm_decim_fir.h"

static const int32_t fir_int32_05_4331_5100_010_095[251] = {
	-250963,
	-530472,
	-956449,
	-1440505,
	-1861966,
	-2058350,
	-1862594,
	-1155766,
	78930,
	1719645,
	3501958,
	5061143,
	6013975,
	6066039,
	5113131,
	3303104,
	1032343,
	-1135681,
	-2614668,
	-2968117,
	-2057986,
	-118896,
	2276521,
	4363476,
	5409644,
	4953617,
	2982225,
	-20415,
	-3181416,
	-5487913,
	-6107986,
	-4685208,
	-1505051,
	2535666,
	6167372,
	8134966,
	7619351,
	4554012,
	-280001,
	-5435156,
	-9220394,
	-10246686,
	-7922490,
	-2726786,
	3853867,
	9734139,
	12878468,
	11991508,
	7020539,
	-717311,
	-8859727,
	-14705610,
	-16097433,
	-12195790,
	-3874308,
	6431337,
	15415517,
	19946809,
	18139792,
	10093658,
	-1980159,
	-14334496,
	-22829097,
	-24298020,
	-17696570,
	-4621840,
	11019276,
	24149376,
	30144976,
	26434112,
	13524367,
	-4930822,
	-23124320,
	-34892082,
	-35768599,
	-24626114,
	-4311380,
	19011680,
	37690383,
	45054463,
	37744852,
	17102968,
	-10919933,
	-37416293,
	-53300866,
	-52364722,
	-33596881,
	-1935268,
	32928131,
	59438198,
	67925581,
	54022129,
	20569914,
	-22737901,
	-62048140,
	-83661289,
	-78659667,
	-46330665,
	4859442,
	59276671,
	98707953,
	108233225,
	81552245,
	23955872,
	-48257755,
	-112070264,
	-144691658,
	-131171943,
	-70276392,
	23448172,
	122479978,
	193637242,
	208100323,
	151336952,
	29895897,
	-127299559,
	-274102498,
	-357874627,
	-333256339,
	-175377009,
	111180608,
	490397234,
	900786668,
	1268179136,
	1522305381,
	1613059887,
	1522305381,
	1268179136,
	900786668,
	490397234,
	111180608,
	-175377009,
	-333256339,
	-357874627,
	-274102498,
	-127299559,
	29895897,
	151336952,
	208100323,
	193637242,
	122479978,
	23448172,
	-70276392,
	-131171943,
	-144691658,
	-112070264,
	-48257755,
	23955872,
	81552245,
	108233225,
	98707953,
	59276671,
	4859442,
	-46330665,
	-78659667,
	-83661289,
	-62048140,
	-22737901,
	20569914,
	54022129,
	67925581,
	59438198,
	32928131,
	-1935268,
	-33596881,
	-52364722,
	-53300866,
	-37416293,
	-10919933,
	17102968,
	37744852,
	45054463,
	37690383,
	19011680,
	-4311380,
	-24626114,
	-35768599,
	-34892082,
	-23124320,
	-4930822,
	13524367,
	26434112,
	30144976,
	24149376,
	11019276,
	-4621840,
	-17696570,
	-24298020,
	-22829097,
	-14334496,
	-1980159,
	10093658,
	18139792,
	19946809,
	15415517,
	6431337,
	-3874308,
	-12195790,
	-16097433,
	-14705610,
	-8859727,
	-717311,
	7020539,
	11991508,
	12878468,
	9734139,
	3853867,
	-2726786,
	-7922490,
	-10246686,
	-9220394,
	-5435156,
	-280001,
	4554012,
	7619351,
	8134966,
	6167372,
	2535666,
	-1505051,
	-4685208,
	-6107986,
	-5487913,
	-3181416,
	-20415,
	2982225,
	4953617,
	5409644,
	4363476,
	2276521,
	-118896,
	-2057986,
	-2968117,
	-2614668,
	-1135681,
	1032343,
	3303104,
	5113131,
	6066039,
	6013975,
	5061143,
	3501958,
	1719645,
	78930,
	-1155766,
	-1862594,
	-2058350,
	-1861966,
	-1440505,
	-956449,
	-530472,
	-250963

};

struct pdm_decim pdm_decim_int32_05_4331_5100_010_095 = {
	5, 251, 2, 4331, 5100, 10, 95, fir_int32_05_4331_5100_010_095
};
