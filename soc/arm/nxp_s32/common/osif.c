/*
 * Copyright 2022-2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <OsIf.h>
#include <OsIf_Cfg_TypesDef.h>

/* Required by OsIf timer initialization but not used with Zephyr, so no values configured */
static const OsIf_ConfigType osif_config;
const OsIf_ConfigType *const OsIf_apxPredefinedConfig[OSIF_MAX_COREIDX_SUPPORTED] = {
	&osif_config
};

/*
 * OsIf call to get the processor number of the core making the access.
 */
uint8_t Sys_GetCoreID(void)
{
	return ((uint8_t)(IP_MSCM->CPXNUM & MSCM_CPXNUM_CPN_MASK));
}
