/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <soc.h>

#include <sys/byteorder.h>

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"
#include "hal/radio_df.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_chan.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_sync.h"
#include "lll_df_types.h"

#include "lll_internal.h"
#include "lll_adv_internal.h"
#include "lll_tim_internal.h"
#include "lll_prof_internal.h"
#include "lll_df_internal.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME bt_ctlr_lll_adv_sync
#include "common/log.h"
#include "hal/debug.h"

static int init_reset(void);
static int prepare_cb(struct lll_prepare_param *p);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_done(void *param);

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
static void isr_tx(void *param);
static void pdu_b2b_update(struct lll_adv_sync *lll, struct pdu_adv *pdu, uint32_t cte_len_us);
static void pdu_b2b_aux_ptr_update(struct pdu_adv *pdu, uint8_t phy, uint8_t flags,
				   uint8_t chan_idx, uint32_t offset_us, uint32_t cte_len_us);
static void switch_radio_complete_and_b2b_tx(const struct lll_adv_sync *lll, uint8_t phy_s);
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

int lll_adv_sync_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_adv_sync_reset(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

void lll_adv_sync_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT(err >= 0);

	/* Invoke common pipeline handling of prepare */
	err = lll_prepare(lll_is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT(!err || err == -EINPROGRESS);
}

static int init_reset(void)
{
	return 0;
}

static bool is_instant_or_past(uint16_t event_counter, uint16_t instant)
{
	uint16_t instant_latency;

	instant_latency = (event_counter - instant) &
			  EVENT_INSTANT_MAX;

	return instant_latency <= EVENT_INSTANT_LATENCY_MAX;
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_adv_sync *lll;
	uint32_t ticks_at_event;
	uint32_t ticks_at_start;
	uint8_t data_chan_count;
	uint8_t *data_chan_map;
	uint16_t event_counter;
	uint8_t data_chan_use;
	struct pdu_adv *pdu;
	struct ull_hdr *ull;
	uint32_t cte_len_us;
	uint32_t remainder;
	uint32_t start_us;
	uint8_t phy_s;
	uint8_t upd;

	DEBUG_RADIO_START_A(1);

	lll = p->param;

	/* Calculate the current event latency */
	lll->latency_event = lll->latency_prepare + p->lazy;

	/* Calculate the current event counter value */
	event_counter = lll->event_counter + lll->latency_event;

	/* Update event counter to next value */
	lll->event_counter = (event_counter + 1);

	/* Reset accumulated latencies */
	lll->latency_prepare = 0;

	/* Process channel map update, if any */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(event_counter, lll->chm_instant)) {
		/* At or past the instant, use channelMapNew */
		lll->chm_first = lll->chm_last;
	}

	/* Calculate the radio channel to use */
	data_chan_map = lll->chm[lll->chm_first].data_chan_map;
	data_chan_count = lll->chm[lll->chm_first].data_chan_count;
	data_chan_use = lll_chan_sel_2(event_counter, lll->data_chan_id,
				       data_chan_map, data_chan_count);

	/* Start setting up of Radio h/w */
	radio_reset();
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	radio_tx_power_set(lll->adv->tx_pwr_lvl);
#else
	radio_tx_power_set(RADIO_TXP_DEFAULT);
#endif

	phy_s = lll->adv->phy_s;

	/* TODO: if coded we use S8? */
	radio_phy_set(phy_s, lll->adv->phy_flags);
	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT, PDU_AC_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_PHY(phy_s));
	radio_aa_set(lll->access_addr);
	radio_crc_configure(PDU_CRC_POLYNOMIAL,
				sys_get_le24(lll->crc_init));
	lll_chan_set(data_chan_use);

	upd = 0U;
	pdu = lll_adv_sync_data_latest_get(lll, NULL, &upd);
	LL_ASSERT(pdu);

#if defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
	lll_df_cte_tx_enable(lll, pdu, &cte_len_us);
#else
	cte_len_us = 0U;
#endif /* CONFIG_BT_CTLR_DF_ADV_CTE_TX) */

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	if (upd) {
		/* AuxPtr offsets for b2b TX are fixed for given chain so we can
		 * calculate them here in advance.
		 */
		pdu_b2b_update(lll, pdu, cte_len_us);
	}
#endif
	radio_pkt_tx_set(pdu);

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	if (pdu->adv_ext_ind.ext_hdr_len && pdu->adv_ext_ind.ext_hdr.aux_ptr) {
		lll->last_pdu = pdu;

		radio_isr_set(isr_tx, lll);
		radio_tmr_tifs_set(EVENT_SYNC_B2B_MAFS_US);
		switch_radio_complete_and_b2b_tx(lll, phy_s);
	} else
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */
	{
		radio_isr_set(isr_done, lll);
		radio_switch_complete_and_disable();
	}

	ticks_at_event = p->ticks_at_expire;
	ull = HDR_LLL2ULL(lll);
	ticks_at_event += lll_event_offset_get(ull);

	ticks_at_start = ticks_at_event;
	ticks_at_start += HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	remainder = p->remainder;
	start_us = radio_tmr_start(1, ticks_at_start, remainder);

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
	radio_gpio_pa_setup();

	radio_gpio_pa_lna_enable(start_us + radio_tx_ready_delay_get(phy_s, 1) -
				 HAL_RADIO_GPIO_PA_OFFSET);
#else /* !HAL_RADIO_GPIO_HAVE_PA_PIN */
	ARG_UNUSED(start_us);
#endif /* !HAL_RADIO_GPIO_HAVE_PA_PIN */

#if defined(CONFIG_BT_CTLR_XTAL_ADVANCED) && \
	(EVENT_OVERHEAD_PREEMPT_US <= EVENT_OVERHEAD_PREEMPT_MIN_US)
	/* check if preempt to start has changed */
	if (lll_preempt_calc(ull, (TICKER_ID_ADV_SYNC_BASE +
				   ull_adv_sync_lll_handle_get(lll)),
			     ticks_at_event)) {
		radio_isr_set(lll_isr_abort, lll);
		radio_disable();
	} else
#endif /* CONFIG_BT_CTLR_XTAL_ADVANCED */
	{
		uint32_t ret;

		ret = lll_prepare_done(lll);
		LL_ASSERT(!ret);
	}

	DEBUG_RADIO_START_A(1);

	return 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct lll_adv_sync *lll;
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		radio_isr_set(isr_done, param);
		radio_disable();
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT(err >= 0);

	/* Accumulate the latency as event is aborted while being in pipeline */
	lll = prepare_param->param;
	lll->latency_prepare += (prepare_param->lazy + 1);

	lll_done(param);
}

static void isr_done(void *param)
{
	struct lll_adv_sync *lll = param;

#if defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
	if (lll->cte_started) {
		lll_df_conf_cte_tx_disable();
	}
#endif /* CONFIG_BT_CTLR_DF_ADV_CTE_TX */

	/* Signal thread mode to remove Channel Map Update Indication in the
	 * ACAD.
	 */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(lll->event_counter, lll->chm_instant)) {
		struct node_rx_hdr *rx;

		/* Allocate, prepare and dispatch Channel Map Update
		 * complete message towards ULL, then subsequently to
		 * the thread context.
		 */
		rx = ull_pdu_rx_alloc();
		LL_ASSERT(rx);

		rx->type = NODE_RX_TYPE_SYNC_CHM_COMPLETE;
		rx->rx_ftr.param = lll;

		ull_rx_put(rx->link, rx);
		ull_rx_sched();
	}

	lll_isr_done(lll);
}

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
static void isr_tx(void *param)
{
	struct lll_adv_sync *lll_sync;
	struct pdu_adv *pdu;
	struct lll_adv *lll;
	uint32_t cte_len_us;

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
	}

	/* Clear radio tx status and events */
	lll_isr_tx_status_reset();

	lll_sync = param;
	lll = lll_sync->adv;

	/* FIXME: Use implementation defined channel index */
	lll_chan_set(0);

	pdu = lll_adv_pdu_linked_next_get(lll_sync->last_pdu);
	LL_ASSERT(pdu);
	lll_sync->last_pdu = pdu;

#if defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
	lll_df_cte_tx_enable(lll_sync, pdu, &cte_len_us);
#else
	cte_len_us = 0;
#endif /* CONFIG_BT_CTLR_DF_ADV_CTE_TX */

	/* setup tIFS switching */
	if (pdu->adv_ext_ind.ext_hdr_len && pdu->adv_ext_ind.ext_hdr.aux_ptr) {
		radio_tmr_tifs_set(EVENT_SYNC_B2B_MAFS_US);
		radio_isr_set(isr_tx, lll_sync);
		switch_radio_complete_and_b2b_tx(lll_sync, lll->phy_s);
	} else {
		radio_isr_set(isr_done, lll_sync);
		radio_switch_complete_and_disable();
	}

	radio_pkt_tx_set(pdu);

	/* assert if radio packet ptr is not set and radio started rx */
	LL_ASSERT(!radio_is_ready());

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_cputime_capture();
	}

	/* capture end of AUX_SYNC_IND/AUX_CHAIN_IND PDU, used for calculating
	 * next PDU timestamp.
	 */
	radio_tmr_end_capture();

#if defined(HAL_RADIO_GPIO_HAVE_LNA_PIN)
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		/* PA/LNA enable is overwriting packet end used in ISR
		 * profiling, hence back it up for later use.
		 */
		lll_prof_radio_end_backup();
	}

	radio_gpio_lna_setup();
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() +
				 EVENT_SYNC_B2B_MAFS_US -
				 (EVENT_CLOCK_JITTER_US << 1) + cte_len_us -
				 radio_tx_chain_delay_get(lll->phy_s, 0) -
				 HAL_RADIO_GPIO_LNA_OFFSET);
#endif /* HAL_RADIO_GPIO_HAVE_LNA_PIN */

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_send();
	}
}

static void pdu_b2b_update(struct lll_adv_sync *lll, struct pdu_adv *pdu, uint32_t cte_len_us)
{
	while (pdu) {
		/* FIXME: Use implementation defined channel index */
		pdu_b2b_aux_ptr_update(pdu, lll->adv->phy_s, lll->adv->phy_flags, 0,
				       EVENT_SYNC_B2B_MAFS_US, cte_len_us);
		pdu = lll_adv_pdu_linked_next_get(pdu);
	}
}

static void pdu_b2b_aux_ptr_update(struct pdu_adv *pdu, uint8_t phy, uint8_t flags,
				   uint8_t chan_idx, uint32_t offset_us, uint32_t cte_len_us)
{
	struct pdu_adv_com_ext_adv *com_hdr;
	struct pdu_adv_aux_ptr *aux_ptr;
	struct pdu_adv_ext_hdr *hdr;
	uint8_t *dptr;

	com_hdr = &pdu->adv_ext_ind;
	hdr = &com_hdr->ext_hdr;
	/* Skip flags */
	dptr = hdr->data;

	if (!com_hdr->ext_hdr_len || !hdr->aux_ptr) {
		return;
	}

	LL_ASSERT(!hdr->adv_addr);
	LL_ASSERT(!hdr->tgt_addr);

	if (hdr->cte_info) {
		dptr++;
	}

	if (IS_ENABLED(CONFIG_BT_CTLR_ADV_PERIODIC_ADI_SUPPORT) && hdr->adi != 0) {
		dptr += sizeof(struct pdu_adv_adi);
	} else {
		LL_ASSERT(!hdr->adi);
	}

	/* Update AuxPtr */
	aux_ptr = (void *)dptr;
	offset_us += PDU_AC_US(pdu->len, phy, flags);
	/* Add CTE length to PDUs that have CTE attached.
	 * Periodic advertising chain may include PDUs without CTE.
	 */
	if (hdr->cte_info) {
		offset_us += cte_len_us;
	}
	offset_us = offset_us / OFFS_UNIT_30_US;
	if (!!(offset_us >> OFFS_UNIT_BITS)) {
		aux_ptr->offs = offset_us / (OFFS_UNIT_300_US / OFFS_UNIT_30_US);
		aux_ptr->offs_units = OFFS_UNIT_VALUE_300_US;
	} else {
		aux_ptr->offs = offset_us;
		aux_ptr->offs_units = OFFS_UNIT_VALUE_30_US;
	}
	aux_ptr->chan_idx = chan_idx;
	aux_ptr->ca = (lll_clock_ppm_local_get() <= SCA_50_PPM) ?
		      SCA_VALUE_50_PPM : SCA_VALUE_500_PPM;
	aux_ptr->phy = find_lsb_set(phy) - 1;
}

static void switch_radio_complete_and_b2b_tx(const struct lll_adv_sync *lll, uint8_t phy_s)
{
#if defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
	if (lll->cte_started) {
		radio_switch_complete_and_phy_end_b2b_tx(phy_s, 0, phy_s, 0);
	} else
#endif /* CONFIG_BT_CTLR_DF_ADV_CTE_TX */
	{
		radio_switch_complete_and_b2b_tx(phy_s, 0, phy_s, 0);
	}
}
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */
