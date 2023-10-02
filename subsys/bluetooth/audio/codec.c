/*
 * Copyright (c) 2022 Bose Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/** @file
 *  @brief Bluetooth LE-Audio codec LTV parsing
 *
 *  Helper functions to parse codec config data as specified in the Bluetooth assigned numbers for
 *  Generic Audio.
 */

#include <stdlib.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_audio_codec, CONFIG_BT_AUDIO_CODEC_LOG_LEVEL);

int bt_audio_codec_cfg_freq_to_freq_hz(enum bt_audio_codec_config_freq freq)
{
	switch (freq) {
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_8KHZ:
		return 8000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_11KHZ:
		return 11025;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_16KHZ:
		return 16000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_22KHZ:
		return 22050;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_24KHZ:
		return 24000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_32KHZ:
		return 32000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_44KHZ:
		return 44100;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_48KHZ:
		return 48000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_88KHZ:
		return 88200;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_96KHZ:
		return 96000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_176KHZ:
		return 176400;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_192KHZ:
		return 192000;
	case BT_AUDIO_CODEC_CONFIG_LC3_FREQ_384KHZ:
		return 384000;
	default:
		return -EINVAL;
	}
}

int bt_audio_codec_cfg_freq_hz_to_freq(uint32_t freq_hz)
{
	switch (freq_hz) {
	case 8000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_8KHZ;
	case 11025U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_11KHZ;
	case 16000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_16KHZ;
	case 22050U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_22KHZ;
	case 24000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_24KHZ;
	case 32000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_32KHZ;
	case 44100U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_44KHZ;
	case 48000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_48KHZ;
	case 88200U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_88KHZ;
	case 96000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_96KHZ;
	case 176400U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_176KHZ;
	case 192000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_192KHZ;
	case 384000U:
		return BT_AUDIO_CODEC_CONFIG_LC3_FREQ_384KHZ;
	default:
		return -EINVAL;
	}
}

struct search_type_param {
	bool found;
	uint8_t type;
	uint8_t data_len;
	const uint8_t **data;
};

static bool parse_cb(struct bt_data *data, void *user_data)
{
	struct search_type_param *param = (struct search_type_param *)user_data;

	if (param->type == data->type) {
		param->found = true;
		param->data_len = data->data_len;
		*param->data = data->data;

		return false;
	}

	return true;
}

#if CONFIG_BT_AUDIO_CODEC_CFG_MAX_DATA_SIZE > 0

static void init_net_buf_simple_from_codec_cfg(struct net_buf_simple *buf,
					       struct bt_audio_codec_cfg *codec_cfg)
{
	buf->__buf = codec_cfg->data;
	buf->data = codec_cfg->data;
	buf->size = sizeof(codec_cfg->data);
	buf->len = codec_cfg->data_len;
}

uint8_t bt_audio_codec_cfg_get_val(const struct bt_audio_codec_cfg *codec_cfg, uint8_t type,
				   const uint8_t **data)
{
	struct search_type_param param = {
		.found = false,
		.type = type,
		.data_len = 0,
		.data = data,
	};
	int err;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return 0;
	}

	CHECKIF(data == NULL) {
		LOG_DBG("data is NULL");
		return 0;
	}

	*data = NULL;

	err = bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, parse_cb, &param);
	if (err != 0 && err != -ECANCELED) {
		LOG_DBG("Could not parse the data: %d", err);
		return 0;
	}

	if (param.data == NULL) {
		LOG_DBG("Could not find the type %u", type);
		return 0;
	}

	return param.data_len;
}

int bt_audio_codec_cfg_set_val(struct bt_audio_codec_cfg *codec_cfg, uint8_t type,
			       const uint8_t *data, size_t data_len)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	CHECKIF(data == NULL) {
		LOG_DBG("data is NULL");
		return -EINVAL;
	}

	CHECKIF(data_len == 0U || data_len > UINT8_MAX) {
		LOG_DBG("Invalid data_len %zu", data_len);
		return -EINVAL;
	}

	for (uint16_t i = 0U; i < codec_cfg->data_len;) {
		uint8_t *len = &codec_cfg->data[i++];
		const uint8_t data_type = codec_cfg->data[i++];
		const uint8_t value_len = *len - sizeof(data_type);

		if (data_type == type) {
			uint8_t *value = &codec_cfg->data[i];

			if (data_len == value_len) {
				memcpy(value, data, data_len);
			} else {
				const int16_t diff = data_len - value_len;
				uint8_t *old_next_data_start;
				uint8_t *new_next_data_start;
				uint8_t data_len_to_move;

				/* Check if this is the last value in the buffer */
				if (value + value_len == codec_cfg->data + codec_cfg->data_len) {
					data_len_to_move = 0U;
				} else {
					old_next_data_start = value + value_len + 1;
					new_next_data_start = value + data_len + 1;
					data_len_to_move = codec_cfg->data_len -
							   (old_next_data_start - codec_cfg->data);
				}

				if (diff < 0) {
					/* In this case we need to move memory around after the copy
					 * to fit the new shorter data
					 */

					memcpy(value, data, data_len);
					if (data_len_to_move > 0U) {
						memmove(new_next_data_start, old_next_data_start,
							data_len_to_move);
					}
				} else {
					/* In this case we need to move memory around before
					 * the copy to fit the new longer data
					 */
					if ((codec_cfg->data_len + diff) >
					    ARRAY_SIZE(codec_cfg->data)) {
						LOG_DBG("Cannot fit data_len %zu in buf with len "
							"%u and size %u",
							data_len, codec_cfg->data_len,
							ARRAY_SIZE(codec_cfg->data));
						return -ENOMEM;
					}

					if (data_len_to_move > 0) {
						memmove(new_next_data_start, old_next_data_start,
							data_len_to_move);
					}

					memcpy(value, data, data_len);
				}

				codec_cfg->data_len += diff;
				*len += diff;
			}

			return codec_cfg->data_len;
		}

		i += value_len;
	}

	/* If we reach here, we did not find the data in the buffer, so we simply add it */
	if ((codec_cfg->data_len + data_len) <= ARRAY_SIZE(codec_cfg->data)) {
		struct net_buf_simple buf;

		init_net_buf_simple_from_codec_cfg(&buf, codec_cfg);

		net_buf_simple_add_u8(&buf, data_len + sizeof(type));
		net_buf_simple_add_u8(&buf, type);
		if (data_len > 0) {
			net_buf_simple_add_mem(&buf, data, data_len);
		}
		codec_cfg->data_len = buf.len;
	} else {
		LOG_DBG("Cannot fit data_len %zu in codec_cfg with len %u and size %u", data_len,
			codec_cfg->data_len, ARRAY_SIZE(codec_cfg->data));
		return -ENOMEM;
	}

	return codec_cfg->data_len;
}

int bt_audio_codec_cfg_get_freq(const struct bt_audio_codec_cfg *codec_cfg)
{
	enum bt_audio_codec_config_freq freq;
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cfg_get_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_FREQ, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	freq = data[0];
	if (bt_audio_codec_cfg_freq_to_freq_hz(freq) < 0) {
		LOG_DBG("Invalid freq value: 0x%02X", freq);
		return -EBADMSG;
	}

	return freq;
}

int bt_audio_codec_cfg_set_freq(struct bt_audio_codec_cfg *codec_cfg,
				enum bt_audio_codec_config_freq freq)
{
	uint8_t freq_u8;

	if (bt_audio_codec_cfg_freq_to_freq_hz(freq) < 0) {
		LOG_DBG("Invalid freq value: %d", freq);
		return -EINVAL;
	}

	freq_u8 = (uint8_t)freq;

	return bt_audio_codec_cfg_set_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_FREQ, &freq_u8,
					  sizeof(freq_u8));
}

int bt_audio_codec_cfg_get_frame_duration_us(const struct bt_audio_codec_cfg *codec_cfg)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cfg_get_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_DURATION, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	switch (data[0]) {
	case BT_AUDIO_CODEC_CONFIG_LC3_DURATION_7_5:
		return 7500;
	case BT_AUDIO_CODEC_CONFIG_LC3_DURATION_10:
		return 10000;
	default:
		return -EBADMSG;
	}
}

int bt_audio_codec_cfg_get_chan_allocation(const struct bt_audio_codec_cfg *codec_cfg,
					   enum bt_audio_location *chan_allocation)
{
	const uint8_t *data;
	uint8_t data_len;

	*chan_allocation = 0;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return -EINVAL;
	}

	CHECKIF(chan_allocation == NULL) {
		return -EINVAL;
	}

	data_len =
		bt_audio_codec_cfg_get_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_CHAN_ALLOC, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint32_t)) {
		return -EBADMSG;
	}

	*chan_allocation = sys_get_le32(data);

	return 0;
}

int bt_audio_codec_cfg_set_chan_allocation(struct bt_audio_codec_cfg *codec_cfg,
					   enum bt_audio_location chan_allocation)
{
	uint32_t chan_allocation_u32;

	if ((chan_allocation & BT_AUDIO_LOCATION_ANY) != chan_allocation) {
		LOG_DBG("Invalid chan_allocation value: 0x%08X", chan_allocation);
		return -EINVAL;
	}

	chan_allocation_u32 = sys_cpu_to_le32((uint32_t)chan_allocation);

	return bt_audio_codec_cfg_set_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_CHAN_ALLOC,
					  (const uint8_t *)&chan_allocation_u32,
					  sizeof(chan_allocation_u32));
}

int bt_audio_codec_cfg_get_octets_per_frame(const struct bt_audio_codec_cfg *codec_cfg)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return -EINVAL;
	}

	data_len =
		bt_audio_codec_cfg_get_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_FRAME_LEN, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint16_t)) {
		return -EBADMSG;
	}

	return sys_get_le16(data);
}

int bt_audio_codec_cfg_set_octets_per_frame(struct bt_audio_codec_cfg *codec_cfg,
					    uint16_t octets_per_frame)
{
	uint16_t octets_per_frame_le16;

	octets_per_frame_le16 = sys_cpu_to_le16(octets_per_frame);

	return bt_audio_codec_cfg_set_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_FRAME_LEN,
					  (uint8_t *)&octets_per_frame_le16,
					  sizeof(octets_per_frame_le16));
}

int bt_audio_codec_cfg_get_frame_blocks_per_sdu(const struct bt_audio_codec_cfg *codec_cfg,
						bool fallback_to_default)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cfg_get_val(codec_cfg,
					      BT_AUDIO_CODEC_CONFIG_LC3_FRAME_BLKS_PER_SDU, &data);
	if (data == NULL) {
		if (fallback_to_default) {
			return 1;
		}

		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

int bt_audio_codec_cfg_set_frame_blocks_per_sdu(struct bt_audio_codec_cfg *codec_cfg,
						uint8_t frame_blocks)
{
	return bt_audio_codec_cfg_set_val(codec_cfg, BT_AUDIO_CODEC_CONFIG_LC3_FRAME_BLKS_PER_SDU,
					  &frame_blocks, sizeof(frame_blocks));
}
#endif /* CONFIG_BT_AUDIO_CODEC_CFG_MAX_DATA_SIZE > 0 */

#if CONFIG_BT_AUDIO_CODEC_CFG_MAX_METADATA_SIZE > 0 ||                                             \
	CONFIG_BT_AUDIO_CODEC_CAP_MAX_METADATA_SIZE > 0

static int codec_meta_get_val(const uint8_t meta[], size_t meta_len, uint8_t type,
			      const uint8_t **data)
{
	struct search_type_param param = {
		.type = type,
		.data_len = 0,
		.data = data,
	};
	int err;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(data == NULL) {
		LOG_DBG("data is NULL");
		return -EINVAL;
	}

	*data = NULL;

	err = bt_audio_data_parse(meta, meta_len, parse_cb, &param);
	if (err != 0 && err != -ECANCELED) {
		LOG_DBG("Could not parse the meta data: %d", err);
		return err;
	}

	if (!param.found) {
		LOG_DBG("Could not find the type %u", type);
		return -ENODATA;
	}

	return param.data_len;
}

static int codec_meta_get_pref_context(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_PREF_CONTEXT, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (ret != sizeof(uint16_t)) {
		return -EBADMSG;
	}

	return sys_get_le16(data);
}

static int codec_meta_get_stream_context(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_STREAM_CONTEXT, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (ret != sizeof(uint16_t)) {
		return -EBADMSG;
	}

	return sys_get_le16(data);
}

static int codec_meta_get_program_info(const uint8_t meta[], size_t meta_len,
				       const uint8_t **program_info)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(program_info == NULL) {
		LOG_DBG("program_info is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_PROGRAM_INFO, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	*program_info = data;

	return ret;
}

static int codec_meta_get_stream_lang(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_STREAM_LANG, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (ret != 3) { /* Stream language is 3 octets */
		return -EBADMSG;
	}

	return sys_get_le24(data);
}

static int codec_meta_get_ccid_list(const uint8_t meta[], size_t meta_len,
				    const uint8_t **ccid_list)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(ccid_list == NULL) {
		LOG_DBG("ccid_list is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_CCID_LIST, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	*ccid_list = data;

	return ret;
}

static int codec_meta_get_parental_rating(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_PARENTAL_RATING, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (ret != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

static int codec_meta_get_program_info_uri(const uint8_t meta[], size_t meta_len,
					   const uint8_t **program_info_uri)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(program_info_uri == NULL) {
		LOG_DBG("program_info_uri is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_PROGRAM_INFO_URI, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	*program_info_uri = data;

	return ret;
}

static int codec_meta_get_audio_active_state(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_AUDIO_STATE, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (ret != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

static int codec_meta_get_bcast_audio_immediate_rend_flag(const uint8_t meta[], size_t meta_len)
{
	const uint8_t *data;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	return codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_BROADCAST_IMMEDIATE,
				  &data);
}

static int codec_meta_get_extended(const uint8_t meta[], size_t meta_len,
				   const uint8_t **extended_meta)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(extended_meta == NULL) {
		LOG_DBG("extended_meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_EXTENDED, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	*extended_meta = data;

	return ret;
}

static int codec_meta_get_vendor(const uint8_t meta[], size_t meta_len, const uint8_t **vendor_meta)
{
	const uint8_t *data;
	int ret;

	CHECKIF(meta == NULL) {
		LOG_DBG("meta is NULL");
		return -EINVAL;
	}

	CHECKIF(vendor_meta == NULL) {
		LOG_DBG("vendor_meta is NULL");
		return -EINVAL;
	}

	ret = codec_meta_get_val(meta, meta_len, BT_AUDIO_METADATA_TYPE_VENDOR, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	*vendor_meta = data;

	return ret;
}

#if CONFIG_BT_AUDIO_CODEC_CFG_MAX_METADATA_SIZE > 0
int bt_audio_codec_cfg_meta_get_val(const struct bt_audio_codec_cfg *codec_cfg, uint8_t type,
				    const uint8_t **data)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_val(codec_cfg->meta, codec_cfg->meta_len, type, data);
}

int bt_audio_codec_cfg_meta_get_pref_context(const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_pref_context(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_stream_context(const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_stream_context(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_program_info(const struct bt_audio_codec_cfg *codec_cfg,
					     const uint8_t **program_info)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_program_info(codec_cfg->meta, codec_cfg->meta_len, program_info);
}

int bt_audio_codec_cfg_meta_get_stream_lang(const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_stream_lang(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_ccid_list(const struct bt_audio_codec_cfg *codec_cfg,
					  const uint8_t **ccid_list)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_ccid_list(codec_cfg->meta, codec_cfg->meta_len, ccid_list);
}

int bt_audio_codec_cfg_meta_get_parental_rating(const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_parental_rating(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_program_info_uri(const struct bt_audio_codec_cfg *codec_cfg,
						 const uint8_t **program_info_uri)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_program_info_uri(codec_cfg->meta, codec_cfg->meta_len,
					       program_info_uri);
}

int bt_audio_codec_cfg_meta_get_audio_active_state(const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_audio_active_state(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_bcast_audio_immediate_rend_flag(
	const struct bt_audio_codec_cfg *codec_cfg)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	LOG_ERR("codec_cfg->meta_len %zu", codec_cfg->meta_len);

	return codec_meta_get_bcast_audio_immediate_rend_flag(codec_cfg->meta, codec_cfg->meta_len);
}

int bt_audio_codec_cfg_meta_get_extended(const struct bt_audio_codec_cfg *codec_cfg,
					 const uint8_t **extended_meta)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_extended(codec_cfg->meta, codec_cfg->meta_len, extended_meta);
}

int bt_audio_codec_cfg_meta_get_vendor(const struct bt_audio_codec_cfg *codec_cfg,
				       const uint8_t **vendor_meta)
{
	CHECKIF(codec_cfg == NULL) {
		LOG_DBG("codec_cfg is NULL");
		return -EINVAL;
	}

	return codec_meta_get_vendor(codec_cfg->meta, codec_cfg->meta_len, vendor_meta);
}
#endif /* CONFIG_BT_AUDIO_CODEC_CFG_MAX_METADATA_SIZE > 0 */

#if CONFIG_BT_AUDIO_CODEC_CAP_MAX_METADATA_SIZE > 0
int bt_audio_codec_cap_meta_get_val(const struct bt_audio_codec_cap *codec_cap, uint8_t type,
				    const uint8_t **data)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_val(codec_cap->meta, codec_cap->meta_len, type, data);
}

int bt_audio_codec_cap_meta_get_pref_context(const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_pref_context(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_stream_context(const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_stream_context(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_program_info(const struct bt_audio_codec_cap *codec_cap,
					     const uint8_t **program_info)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_program_info(codec_cap->meta, codec_cap->meta_len, program_info);
}

int bt_audio_codec_cap_meta_get_stream_lang(const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_stream_lang(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_ccid_list(const struct bt_audio_codec_cap *codec_cap,
					  const uint8_t **ccid_list)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_ccid_list(codec_cap->meta, codec_cap->meta_len, ccid_list);
}

int bt_audio_codec_cap_meta_get_parental_rating(const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_parental_rating(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_program_info_uri(const struct bt_audio_codec_cap *codec_cap,
						 const uint8_t **program_info_uri)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_program_info_uri(codec_cap->meta, codec_cap->meta_len,
					       program_info_uri);
}

int bt_audio_codec_cap_meta_get_audio_active_state(const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_audio_active_state(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_bcast_audio_immediate_rend_flag(
	const struct bt_audio_codec_cap *codec_cap)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_bcast_audio_immediate_rend_flag(codec_cap->meta, codec_cap->meta_len);
}

int bt_audio_codec_cap_meta_get_extended(const struct bt_audio_codec_cap *codec_cap,
					 const uint8_t **extended_meta)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_extended(codec_cap->meta, codec_cap->meta_len, extended_meta);
}

int bt_audio_codec_cap_meta_get_vendor(const struct bt_audio_codec_cap *codec_cap,
				       const uint8_t **vendor_meta)
{
	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	return codec_meta_get_vendor(codec_cap->meta, codec_cap->meta_len, vendor_meta);
}
#endif /* CONFIG_BT_AUDIO_CODEC_CAP_MAX_METADATA_SIZE > 0 */
#endif /* CONFIG_BT_AUDIO_CODEC_CFG_MAX_METADATA_SIZE > 0 ||                                       \
	* CONFIG_BT_AUDIO_CODEC_CAP_MAX_METADATA_SIZE > 0                                          \
	*/

#if CONFIG_BT_AUDIO_CODEC_CAP_MAX_DATA_SIZE > 0

uint8_t bt_audio_codec_cap_get_val(const struct bt_audio_codec_cap *codec_cap, uint8_t type,
				   const uint8_t **data)
{
	struct search_type_param param = {
		.type = type,
		.data_len = 0,
		.data = data,
	};
	int err;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return 0;
	}

	CHECKIF(data == NULL) {
		LOG_DBG("data is NULL");
		return 0;
	}

	*data = NULL;

	err = bt_audio_data_parse(codec_cap->data, codec_cap->data_len, parse_cb, &param);
	if (err != 0 && err != -ECANCELED) {
		LOG_DBG("Could not parse the data: %d", err);
		return 0;
	}

	if (param.data == NULL) {
		LOG_DBG("Could not find the type %u", type);
		return 0;
	}

	return param.data_len;
}

int bt_audio_codec_cap_get_freq(const struct bt_audio_codec_cap *codec_cap)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cap_get_val(codec_cap, BT_AUDIO_CODEC_LC3_FREQ, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint16_t)) {
		return -EBADMSG;
	}

	return sys_get_le16(data);
}

int bt_audio_codec_cap_get_frame_duration(const struct bt_audio_codec_cap *codec_cap)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cap_get_val(codec_cap, BT_AUDIO_CODEC_LC3_DURATION, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

int bt_audio_codec_cap_get_supported_audio_chan_counts(const struct bt_audio_codec_cap *codec_cap)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cap_get_val(codec_cap, BT_AUDIO_CODEC_LC3_CHAN_COUNT, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

int bt_audio_codec_cap_get_octets_per_frame(
	const struct bt_audio_codec_cap *codec_cap,
	struct bt_audio_codec_octets_per_codec_frame *codec_frame)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	CHECKIF(codec_frame == NULL) {
		LOG_DBG("codec_frame is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cap_get_val(codec_cap, BT_AUDIO_CODEC_LC3_FRAME_LEN, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint32_t)) {
		return -EBADMSG;
	}

	codec_frame->min = sys_get_le16(data);
	codec_frame->max = sys_get_le16(data + sizeof(codec_frame->min));

	return 0;
}

int bt_audio_codec_cap_get_max_codec_frames_per_sdu(const struct bt_audio_codec_cap *codec_cap)
{
	const uint8_t *data;
	uint8_t data_len;

	CHECKIF(codec_cap == NULL) {
		LOG_DBG("codec_cap is NULL");
		return -EINVAL;
	}

	data_len = bt_audio_codec_cap_get_val(codec_cap, BT_AUDIO_CODEC_LC3_FRAME_COUNT, &data);
	if (data == NULL) {
		return -ENODATA;
	}

	if (data_len != sizeof(uint8_t)) {
		return -EBADMSG;
	}

	return data[0];
}

#endif /* CONFIG_BT_AUDIO_CODEC_CAP_MAX_DATA_SIZE > 0 */
