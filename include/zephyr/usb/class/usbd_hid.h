/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USBD HID device API header
 */

#ifndef ZEPHYR_INCLUDE_USBD_HID_CLASS_DEVICE_H_
#define ZEPHYR_INCLUDE_USBD_HID_CLASS_DEVICE_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/usb/class/hid.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USBD HID Device API
 * @defgroup usbd_hid_device USBD HID device API
 * @ingroup usb
 * @{
 */

/*
 * HID Device overview:
 *
 *         +---------------------+
 *         |                     |
 *         |                     |
 *         |     HID Device      |
 *         |  User "top half"    |
 *         |  of the device that +-------+
 *         |  deals with input   |       |
 *         |     sampling        |       |
 *         |                     |       |
 *         |                     |       |
 *         | ------------------- |       |
 *         |                     |       |
 *         |   HID Device user   |       |
 *         |      callbacks      |       |
 *         |      handlers       |       |
 *         +---------------------+       |
 *                    ^                  |  HID Device Driver API:
 *                    |                  |
 *  set_protocol()    |                  |  register_app()
 *  get_report()      |                  |  submit_report(
 *  ....              |                  |  ...
 *                    v                  |
 *         +---------------------+       |
 *         |                     |       |
 *         |      HID Device     |       |
 *         |     "bottom half"   |<------+
 *         |    USB HID class    |
 *         |    implementation   |
 *         |                     |
 *         |                     |
 *         +---------------------+
 *                    ^
 *                    v
 *          +--------------------+
 *          |                    |
 *          |     USB Device     |
 *          |       Support      |
 *          |                    |
 *          +--------------------+
 */

/**
 * @brief HID device user callbacks
 *
 * Each device depends on a user part that handles feature, input, and output
 * report processing according to the device functionality described by the
 * report descriptor. Which callbacks must be implemented depends on the device
 * functionality. The USB device part of the HID device, cannot interpret
 * device specific report descriptor and only handles USB specific parts,
 * transfers and validation of requests, all reports are opaque to it.
 * Callbacks are called from the USB device stack thread and must not block.
 */
struct hid_device_ops {
	/**
	 * The interface ready callback is called with the ready argument set
	 * to true when the corresponding interface is part of the active
	 * configuration and the application can e.g. begin submitting input
	 * reports, and with the argument set to false when the interface is no
	 * longer active. This callback is optional.
	 */
	void (*iface_ready)(const struct device *dev, const bool ready);

	/*
	 * Callback to get a feature, input, or output reports, specified by
	 * type an id, from the HID device to the host. This callback is called
	 * for the HID Get Report request. If there is no report ID in the
	 * report descriptor, the id argument is zero. The callback
	 * implementation must check the arquments, such as whether the report
	 * type is supported, and return a nonzero value to indicate an
	 * unsupported type or an error.
	 */
	int (*get_report)(const struct device *dev,
			  const uint8_t type, const uint8_t id,
			  const uint16_t len, uint8_t *const buf);

	/*
	 * Callback to set a feature, input, or output request, specified by
	 * type an id, from the not to the HID device. This callback is called
	 * for the HID Set Report request. If there is no report ID in the
	 * report descriptor, the id argument is zero. The callback
	 * implementation must check the arquments, such as whether the report
	 * type is supported, and return a nonzero value to indicate an
	 * unsupported type or an error.
	 */
	int (*set_report)(const struct device *dev,
			  const uint8_t type, const uint8_t id,
			  const uint16_t len, const uint8_t *const buf);

	/*
	 * Notification to limit intput report frequency.
	 * The application should mute an input report submission until a new
	 * event occurs or until the time specified by the duration value has
	 * elapsed. If a report ID is used in the report descriptor, the
	 * application must store the duration and handle the specified report
	 * accordingly.
	 */
	void (*set_idle)(const struct device *dev,
			 const uint8_t id, const uint32_t duration);

	/*
	 * If a report ID is used in the report descriptor, the application
	 * must implement this callback and return the duration for the
	 * specified report ID.
	 */
	uint32_t (*get_idle)(const struct device *dev, const uint8_t id);

	/*
	 * Notification that the host has changed the protocol from
	 * Boot Protocol(0) to Report Protocol(1) or vice versa.
	 */
	void (*set_protocol)(const struct device *dev, const uint8_t proto);

	/*
	 * Notification that input report submitted with
	 * hid_device_submit_report() has been sent.
	 */
	void (*input_report_done)(const struct device *dev);

	/*
	 * New output report callback. Callback will only be called for reports
	 * received through the optional interrupt OUT pipe. If there is no
	 * interrupt OUT pipe, output reports will be received using set_report().
	 * If a report ID is used in the report descriptor, the host places the ID
	 * in the buffer first, followed by the report data.
	 */
	void (*output_report)(const struct device *dev, const uint16_t len,
			      const uint8_t *const buf);
};

/**
 * @brief Register HID device report descriptor and user callbacks
 *
 * The application must register report descriptor and user callbacks before
 * USB device support is initialized and enabled.
 *
 * @param[in]  dev   Pointer to HID device
 * @param[in]  rdesc Pointer to HID report descriptor
 * @param[in]  rsize Size of HID report descriptor
 * @param[in]  ops   Pointer to HID device callbacks
 */
int hid_device_register(const struct device *dev,
			const uint8_t *const rdesc, const uint16_t rsize,
			const struct hid_device_ops *const ops);

/**
 * @brief Submit new input report
 *
 * Submit a new input report to be sent via the interrupt IN pipe. If sync is
 * true, the functions will block until the report is sent.
 *
 * @param[in] dev    Pointer to HID device
 * @param[in] size   Size of the input report
 * @param[in] report Input report buffer
 * @param[in] sync   If true, blocks until report is sent
 *
 * @return 0 on success, negative errno code on fail.
 */
int hid_device_submit_report(const struct device *dev,
			     const uint16_t size, const uint8_t *const report,
			     const bool sync);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USBD_HID_CLASS_DEVICE_H_ */
