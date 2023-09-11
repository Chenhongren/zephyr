/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief IEEE 802.15.4 L2 stack public header
 */

#ifndef ZEPHYR_INCLUDE_NET_IEEE802154_H_
#define ZEPHYR_INCLUDE_NET_IEEE802154_H_

#include <limits.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/crypto/cipher.h>
#include <zephyr/net/ieee802154_radio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ieee802154 IEEE 802.15.4 and Thread APIs
 * @ingroup connectivity
 *
 * @brief IEEE 802.15.4 native and OpenThread L2, configuration, management and
 * driver APIs
 *
 * @details The IEEE 802.15.4 and Thread subsystems comprise the OpenThread L2
 * subsystem, the native IEEE 802.15.4 L2 subsystem ("Soft" MAC), a mostly
 * vendor and protocol agnostic driver API shared between the OpenThread and
 * native L2 stacks ("Hard" MAC and PHY) as well as several APIs to configure
 * the subsystem (shell, net management, Kconfig, devicetree, etc.).
 *
 * The **OpenThread subsystem API** integrates the external <a
 * href="https://openthread.io">OpenThread</a> stack into Zephyr. It builds upon
 * Zephyr's native IEEE 802.15.4 driver API.
 *
 * The **native IEEE 802.15.4 subsystem APIs** are exposed at different levels
 * and address several audiences:
 *  - shell (end users, application developers):
 *    - a set of IEEE 802.15.4 shell commands (see `shell> ieee802154 help`)
 *  - application API (application developers):
 *    - IPv6, DGRAM and RAW sockets for actual peer-to-peer, multicast and
 *      broadcast data exchange between nodes including connection specific
 *      configuration (sample coming soon, see
 *      https://github.com/linux-wpan/wpan-tools/tree/master/examples for now
 *      which inspired our API and therefore has a similar socket API),
 *    - Kconfig and devicetree configuration options (net config library
 *      extension, subsystem-wide MAC and PHY Kconfig/DT options, driver/vendor
 *      specific Kconfig/DT options, watch out for options prefixed with
 *      IEEE802154/ieee802154),
 *    - Network Management: runtime configuration of the IEEE 802.15.4
 *      protocols stack at the MAC (L2) and PHY (L1) levels
 *      (see @ref ieee802154_mgmt),
 *  - L2 integration (subsystem contributors):
 *    - see @ref ieee802154_l2
 *    - implementation of Zephyr's internal L2-level socket and network context
 *      abstractions (context/socket operations, see @ref net_l2),
 *    - protocol-specific extension to the interface structure (see @ref net_if)
 *    - protocol-specific extensions to the network packet structure
 *      (see @ref net_pkt),
 *
 *  - OpenThread and native IEEE 802.15.4 share a common **driver API** (driver
 *    maintainers/contributors):
 *    - see @ref ieee802154_driver
 *    - a basic, mostly PHY-level driver API to be implemented by all drivers,
 *    - several "hard MAC" (hardware/firmware offloading) extension points for
 *      performance critical or timing sensitive aspects of the protocol
 */

/**
 * @defgroup ieee802154_l2 IEEE 802.15.4 L2
 * @ingroup ieee802154
 *
 * @brief IEEE 802.15.4 L2 APIs
 *
 * @details This API provides integration with Zephyr's sockets and network
 * contexts. **Application and driver developers should never interface directly
 * with this API.** It is of interest to subsystem maintainers only.
 *
 * The API implements and extends the following structures:
 *    - implements Zephyr's internal L2-level socket and network context
 *      abstractions (context/socket operations, see @ref net_l2),
 *    - protocol-specific extension to the interface structure (see @ref net_if)
 *    - protocol-specific extensions to the network packet structure
 *      (see @ref net_pkt),
 * @{
 */

/* References are to the IEEE 802.15.4-2020 standard */
#define IEEE802154_MAX_PHY_PACKET_SIZE	127 /* see section 11.3, aMaxPhyPacketSize */
#define IEEE802154_FCS_LENGTH		2   /* see section 7.2.1.1 */
#define IEEE802154_MTU			(IEEE802154_MAX_PHY_PACKET_SIZE - IEEE802154_FCS_LENGTH)
/* TODO: Support flexible MTU and FCS lengths for IEEE 802.15.4-2015ff */

#define IEEE802154_SHORT_ADDR_LENGTH	2
#define IEEE802154_EXT_ADDR_LENGTH	8
#define IEEE802154_MAX_ADDR_LENGTH	IEEE802154_EXT_ADDR_LENGTH

#define IEEE802154_NO_CHANNEL		USHRT_MAX

/* See IEEE 802.15.4-2020, sections 6.1 and 7.3.5 */
#define IEEE802154_BROADCAST_ADDRESS	     0xffff
#define IEEE802154_NO_SHORT_ADDRESS_ASSIGNED 0xfffe

/* See IEEE 802.15.4-2020, section 6.1 */
#define IEEE802154_BROADCAST_PAN_ID 0xffff

/* See IEEE 802.15.4-2020, section 7.3.5 */
#define IEEE802154_SHORT_ADDRESS_NOT_ASSOCIATED IEEE802154_BROADCAST_ADDRESS
#define IEEE802154_PAN_ID_NOT_ASSOCIATED	IEEE802154_BROADCAST_PAN_ID

struct ieee802154_security_ctx {
	uint32_t frame_counter;
	struct cipher_ctx enc;
	struct cipher_ctx dec;
	uint8_t key[16];
	uint8_t key_len;
	uint8_t level	: 3;
	uint8_t key_mode	: 2;
	uint8_t _unused	: 3;
};

enum ieee802154_device_role {
	IEEE802154_DEVICE_ROLE_ENDDEVICE,
	IEEE802154_DEVICE_ROLE_COORDINATOR,
	IEEE802154_DEVICE_ROLE_PAN_COORDINATOR,
};

/* This not meant to be used by any code but the IEEE 802.15.4 L2 stack */
struct ieee802154_context {
	/* PAN ID
	 *
	 * The identifier of the PAN on which the device is operating. If this
	 * value is 0xffff, the device is not associated. See section 8.4.3.1,
	 * table 8-94, macPanId.
	 *
	 * in CPU byte order
	 */
	uint16_t pan_id;

	/* Channel Number
	 *
	 * The RF channel to use for all transmissions and receptions, see
	 * section 11.3, table 11-2, phyCurrentChannel. The allowable range
	 * of values is PHY dependent as defined in section 10.1.3.
	 *
	 * in CPU byte order
	 */
	uint16_t channel;

	/* Short Address
	 *
	 * Range:
	 *  * 0x0000–0xfffd: associated, short address was assigned
	 *  * 0xfffe: associated but no short address assigned
	 *  * 0xffff: not associated (default),
	 *
	 * See section 6.4.1, table 6-4 (Usage of the shart address) and
	 * section 8.4.3.1, table 8-94, macShortAddress.
	 *
	 * in CPU byte order
	 */
	uint16_t short_addr;

	/* Extended Address
	 *
	 * The extended address is device specific, usually permanently stored
	 * on the device and immutable.
	 *
	 * See section 8.4.3.1, table 8-94, macExtendedAddress.
	 *
	 * in little endian
	 */
	uint8_t ext_addr[IEEE802154_MAX_ADDR_LENGTH];

	struct net_linkaddr_storage linkaddr; /* in big endian */
#ifdef CONFIG_NET_L2_IEEE802154_SECURITY
	struct ieee802154_security_ctx sec_ctx;
#endif
#ifdef CONFIG_NET_L2_IEEE802154_MGMT
	struct ieee802154_req_params *scan_ctx; /* guarded by scan_ctx_lock */
	struct k_sem scan_ctx_lock;

	/* see section 8.4.3.1, table 8-94, macCoordExtendedAddress, the address
	 * of the coordinator through which the device is associated.
	 *
	 * A value of zero indicates that a coordinator extended address is
	 * unknown (default).
	 *
	 * in little endian
	 */
	uint8_t coord_ext_addr[IEEE802154_MAX_ADDR_LENGTH];

	/* see section 8.4.3.1, table 8-94, macCoordShortAddress, the short
	 * address assigned to the coordinator through which the device is
	 * associated.
	 *
	 * A value of 0xfffe indicates that the coordinator is only using its
	 * extended address. A value of 0xffff indicates that this value is
	 * unknown.
	 *
	 * in CPU byte order
	 */
	uint16_t coord_short_addr;
#endif
	int16_t tx_power;
	enum net_l2_flags flags;

	/* The sequence number added to the transmitted Data frame or MAC
	 * command, see section 8.4.3.1, table 8-94, macDsn.
	 */
	uint8_t sequence;

	/* See section 6.1: A device may be operating as end device
	 * (0 - default), coordinator (1), or PAN coordinator (2).
	 *
	 * A value of 3 is undefined.
	 *
	 * Can be read/set via enum ieee802154_device_role.
	 */
	uint8_t device_role : 2;

	uint8_t _unused : 5;

	uint8_t ack_requested : 1; /* guarded by ack_lock */
	uint8_t ack_seq;	   /* guarded by ack_lock */
	struct k_sem ack_lock;

	struct k_sem ctx_lock; /* guards all mutable context attributes unless
				* otherwise mentioned on attribute level
				*/
};

#define IEEE802154_L2_CTX_TYPE	struct ieee802154_context

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_IEEE802154_H_ */
