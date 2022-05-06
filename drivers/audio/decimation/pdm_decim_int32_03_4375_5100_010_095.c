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

static const int32_t fir_int32_03_4375_5100_010_095[157] = {
	350904,
	1127891,
	2233546,
	3059556,
	2752177,
	818057,
	-2252661,
	-4944515,
	-5550704,
	-3466227,
	53089,
	2496755,
	1904111,
	-1421730,
	-4818556,
	-5204443,
	-1721082,
	3155305,
	5311444,
	2454515,
	-3518616,
	-7589471,
	-5713308,
	1327821,
	7901341,
	7958087,
	527903,
	-8633996,
	-11354795,
	-4214518,
	7627116,
	13970242,
	8263364,
	-5861946,
	-16549651,
	-13529964,
	2213919,
	17870757,
	19056207,
	2854066,
	-18029728,
	-24979102,
	-9860164,
	16175117,
	30545984,
	18605951,
	-11894271,
	-35172531,
	-28918160,
	4746528,
	38201092,
	40590666,
	5825429,
	-38712951,
	-53159148,
	-20283384,
	35723256,
	66120531,
	39265868,
	-27910967,
	-78795928,
	-63663750,
	13458962,
	90416047,
	95193346,
	10755142,
	-99897049,
	-137497231,
	-51075841,
	105457455,
	200047804,
	124561003,
	-101612190,
	-313384381,
	-297343743,
	53701878,
	639681661,
	1187800564,
	1411050887,
	1187800564,
	639681661,
	53701878,
	-297343743,
	-313384381,
	-101612190,
	124561003,
	200047804,
	105457455,
	-51075841,
	-137497231,
	-99897049,
	10755142,
	95193346,
	90416047,
	13458962,
	-63663750,
	-78795928,
	-27910967,
	39265868,
	66120531,
	35723256,
	-20283384,
	-53159148,
	-38712951,
	5825429,
	40590666,
	38201092,
	4746528,
	-28918160,
	-35172531,
	-11894271,
	18605951,
	30545984,
	16175117,
	-9860164,
	-24979102,
	-18029728,
	2854066,
	19056207,
	17870757,
	2213919,
	-13529964,
	-16549651,
	-5861946,
	8263364,
	13970242,
	7627116,
	-4214518,
	-11354795,
	-8633996,
	527903,
	7958087,
	7901341,
	1327821,
	-5713308,
	-7589471,
	-3518616,
	2454515,
	5311444,
	3155305,
	-1721082,
	-5204443,
	-4818556,
	-1421730,
	1904111,
	2496755,
	53089,
	-3466227,
	-5550704,
	-4944515,
	-2252661,
	818057,
	2752177,
	3059556,
	2233546,
	1127891,
	350904

};

struct pdm_decim pdm_decim_int32_03_4375_5100_010_095 = {
	3, 157, 1, 4375, 5100, 10, 95, fir_int32_03_4375_5100_010_095
};
