/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_vlan_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <errno.h>

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>

struct ud {
	struct net_if *first;
	struct net_if *second;
};

static void iface_cb(struct net_if *iface, void *user_data)
{
	struct ud *ud = user_data;

	if (net_if_l2(iface) != &NET_L2_GET_NAME(VIRTUAL)) {
		return;
	}

	if (ud->first == NULL) {
		ud->first = iface;
		return;
	}

	ud->second = iface;
}

static int setup_iface(struct net_if *iface, struct net_if *vlan,
		       const char *ipv6_addr, const char *ipv4_addr,
		       const char *netmask, uint16_t vlan_tag)
{
	struct net_if_addr *ifaddr;
	struct in_addr addr4;
	struct in6_addr addr6, netaddr6;
	int ret;

	ret = net_eth_vlan_enable(iface, vlan_tag);
	if (ret < 0) {
		LOG_ERR("Cannot enable VLAN for tag %d (%d)", vlan_tag, ret);
	}

	if (IS_ENABLED(CONFIG_NET_IPV6)) {
		if (net_addr_pton(AF_INET6, ipv6_addr, &addr6)) {
			LOG_ERR("Invalid address: %s", ipv6_addr);
			return -EINVAL;
		}

		ifaddr = net_if_ipv6_addr_add(vlan, &addr6,
					      NET_ADDR_MANUAL, 0);
		if (!ifaddr) {
			LOG_ERR("Cannot add %s to interface %p",
				ipv6_addr, vlan);
			return -EINVAL;
		}

		net_ipv6_addr_prefix_mask((uint8_t *)&addr6,
					  (uint8_t *)&netaddr6,
					  CONFIG_NET_SAMPLE_IFACE_MY_IPV6_PREFIXLEN);

		if (!net_if_ipv6_prefix_add(vlan, &netaddr6,
					    CONFIG_NET_SAMPLE_IFACE_MY_IPV6_PREFIXLEN,
					    (uint32_t)0xffffffff)) {
			LOG_ERR("Cannot add %s with prefix_len %d to interface %p",
				ipv6_addr,
				CONFIG_NET_SAMPLE_IFACE_MY_IPV6_PREFIXLEN,
				vlan);
			return -EINVAL;
		}
	}

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		if (net_addr_pton(AF_INET, ipv4_addr, &addr4)) {
			LOG_ERR("Invalid address: %s", ipv4_addr);
			return -EINVAL;
		}

		ifaddr = net_if_ipv4_addr_add(vlan, &addr4,
					      NET_ADDR_MANUAL, 0);
		if (!ifaddr) {
			LOG_ERR("Cannot add %s to interface %p",
				ipv4_addr, vlan);
			return -EINVAL;
		}

		if (netmask && netmask[0]) {
			struct in_addr nm;

			if (net_addr_pton(AF_INET, netmask, &nm)) {
				LOG_ERR("Invalid netmask: %s", ipv4_addr);
				return -EINVAL;
			}

			net_if_ipv4_set_netmask_by_addr(vlan, &addr4, &nm);
		}
	}

	LOG_DBG("Interface %p VLAN tag %d setup done.", vlan, vlan_tag);

	return 0;
}

static int init_app(void)
{
	struct net_if *iface;
	struct ud ud;
	int ret;

	iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
	if (!iface) {
		LOG_ERR("No ethernet interfaces found.");
		return -ENOENT;
	}

	memset(&ud, 0, sizeof(ud));

	net_if_foreach(iface_cb, &ud);

	ret = setup_iface(iface, ud.first,
			  CONFIG_NET_SAMPLE_IFACE2_MY_IPV6_ADDR,
			  CONFIG_NET_SAMPLE_IFACE2_MY_IPV4_ADDR,
			  CONFIG_NET_SAMPLE_IFACE2_MY_IPV4_NETMASK,
			  CONFIG_NET_SAMPLE_IFACE2_VLAN_TAG);
	if (ret < 0) {
		return ret;
	}

	ret = setup_iface(iface, ud.second,
			  CONFIG_NET_SAMPLE_IFACE3_MY_IPV6_ADDR,
			  CONFIG_NET_SAMPLE_IFACE3_MY_IPV4_ADDR,
			  CONFIG_NET_SAMPLE_IFACE3_MY_IPV4_NETMASK,
			  CONFIG_NET_SAMPLE_IFACE3_VLAN_TAG);
	if (ret < 0) {
		return ret;
	}

	/* Bring up the VLAN interface automatically */
	net_if_up(ud.first);
	net_if_up(ud.second);

	return ret;
}

int main(void)
{
	init_app();
	return 0;
}
