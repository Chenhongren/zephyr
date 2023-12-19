
# USB Application Programming Guide

In ZephyrOS, there are two USB stacks: **"usb_dc"** and **"usbd."** The "usb_dc" stack is the older USB stack, while "usbd" is the newer one. Due to various issues with **"usb_dc"**  stack in the existing design, there is an upstream plan, as outlined in #[issue42066](https://github.com/zephyrproject-rtos/zephyr/issues/42066 "issue42066"), to redesign the USB device support called the "usbd" stack. And the upstream plan to fully support "usbd" stack in **Zephyr 3.7.0**.

The Kernel Configs "CONFIG_USB_DEVICE_STACK" and "CONFIG_USB_DEVICE_STACK_NEXT" are available for selecting either old or new USB drivers. The detail setting is shown as below.

## (old)USB_DC Stack
### Configurations
- **KConfig**
	The USB information can be assigned by the following Kernel Config including VID, PID, manufacturer, product and remote-wakeup supported...etc.
	```c
	## ZephyrOS USB_DC stack
	CONFIG_USB_DEVICE_STACK=y

	## USB Information
	CONFIG_USB_DEVICE_MANUFACTURER="Google"
	CONFIG_USB_DEVICE_PRODUCT="Roach"
	CONFIG_USB_DEVICE_SN="0"
	CONFIG_USB_DEVICE_VID=0x1801
	CONFIG_USB_DEVICE_PID=0x5999
	CONFIG_USB_DEVICE_HID=y
	CONFIG_USB_SELF_POWERED=n
	CONFIG_USB_DEVICE_REMOTE_WAKEUP=y
	CONFIG_USB_HID_BOOT_PROTOCOL=y
	CONFIG_HID_INTERRUPT_EP_MPS=64
	CONFIG_USB_HID_DEVICE_COUNT=2

	## Examples: KEYBOARD and TOUCHPAD definitions
	CONFIG_USB_DC_HID_KEYBOARD=y
	CONFIG_USB_DC_HID_VIVALDI=y
	CONFIG_USB_DC_HID_TOUCHPAD=y
	```
- **DTS(.overlay)**
	```c
	/* USB node for IT82xx2 chip */
	zephyr_udc0: &usb0 {
		status = "okay";
		pinctrl-0 = <&usb0_dm_gph5_default
			     &usb0_dp_gph6_default>;
		pinctrl-names = "default";
	};
	```

### APIs

#### USB Enable
```usb_enable(status_cb)```: Enable USB subsystem

```c
	ret = usb_enable(status_cb);
	if (ret != 0) {
		// Failed to enable USB
	}
```

#### Remote Wakeup
Device can do remote-wakekup by involking ```usb_wakeup_request()``` if the KConfig(```CONFIG_USB_DEVICE_REMOTE_WAKEUP```) is enabled.

```c
	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		usb_wakeup_request();
	}
```
#### HID class APIs
- ```usb_hid_register_device()```: Register HID device
	```c
	/* HID : Report Descriptor (8-byte: report id, modifier and six keycode bytes) */
	static const uint8_t hid_report_desc[] = {
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_KEYBOARD),
		HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(8),
		HID_INPUT(0x01),
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP_KEYPAD),
		HID_USAGE_MIN8(HID_KEYBOARD_MODIFIER_LOW),
		HID_USAGE_MAX8(HID_KEYBOARD_MODIFIER_HIGH),
		HID_LOGICAL_MIN8(0x00),
		HID_LOGICAL_MAX8(0x01),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(8),
		HID_INPUT(0x02),
		HID_REPORT_COUNT(6),
		HID_REPORT_SIZE(8),
		HID_LOGICAL_MIN8(0x0),
		HID_LOGICAL_MAX8(0xa4),
		HID_USAGE_MIN8(0x00),
		HID_USAGE_MAX8(0xa4),
		HID_INPUT(0x00),
		HID_END_COLLECTION
	};

	static int kb_get_report(const struct device *dev,
				 struct usb_setup_packet *setup, int32_t *len,
				 uint8_t **data)
	{
		// Handle GET_REPORT request
		return 0;
	}

	static void protocol_cb(const struct device *dev, uint8_t protocol)
	{
		// Handle SET_PROTOCOL request
	}

	static void int_in_ready_cb(const struct device *dev)
	{
		// The current interrupt IN transfer has completed.
	}
	static const struct hid_ops ops = {
		.protocol_change = protocol_cb,
		.get_report = kb_get_report,
		.int_in_ready = int_in_ready_cb,
	};

	usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &ops);
	```

- ```usb_hid_set_proto_code(dev, proto_code)```: Set HID protocol code(keyboard, mouse...)
	```c
	if (usb_hid_set_proto_code(hid_dev, HID_BOOT_IFACE_CODE_KEYBOARD)) {
		// Failed to set protocol code
	}
	```

- ```usb_hid_init(dev)```: Initialize HID device
	```c
	usb_hid_init(hid_dev);
	```

- ```hid_int_ep_write(dev, report, report_size, bytes_written_to_ep_buf)```: Submit reports
	```c
	ret = hid_int_ep_write(hid_dev, (uint8_t *)&kb_data, size, NULL);
	if (ret) {
		// Failed to submit report
	}
	```

### USB Application Samples
Boards | Tool | Result | Link
-------|------|--------|------
IT82xx2 EVB | hid-mouse   | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/hid-mouse
IT82xx2 EVB | hid         | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/hid
IT82xx2 EVB | console     | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green)   | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/console
IT82xx2 EVB | cdc_acm     | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green)   | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/cdc_acm
Roach       | keyboard and touchpad | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green)   | https://source.chromium.org/chromiumos/chromiumos/codesearch/+/main:src/platform/ec/zephyr/subsys/usb_dc/

## (new)USBD Stack

### Overview
- **Block Digram**
	``` flow
	                    +------------------+
	                    | User Application |
	                    +------------------+
	                             | Device stack API
	+-----------------------------------------------+
	|USB stack  +---------------------------------+ |
	|           |USB device class implementations | |
	|           +---------------------------------+ |
	|                            | USBD Class API   |
	|                 +---------------------+       |
	|                 | USB device framwork |       |
	|                 +---------------------+       |
	+-----------------------------------------------+
	                             | UDC driver API
	                    +------------------+
	                    | UDC common layer |
	                    +------------------+
	                             |
	              +------------------------------+
	              | UDC device controller driver |
	              |      (udc_it82xx2.c)         |
	              +------------------------------+
	```
- **Floders**

	**Application samples**: https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb

	USB device class implementations: https://github.com/zephyrproject-rtos/zephyr/tree/main/subsys/usb/device_next/class

	USB device framwork: https://github.com/zephyrproject-rtos/zephyr/tree/main/subsys/usb/device_next

	UDC common layer: https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/usb/udc/udc_common.c

	**UDC device controller driver**: https://github.com/zephyrproject-rtos/zephyr/tree/main/drivers/usb/udc

### Configurations
- **KConfig**
	```c
	## ZephyrOS USBD stack
	CONFIG_USB_DEVICE_STACK_NEXT=y

	## Choose log level
	CONFIG_LOG=y
	CONFIG_USBD_LOG_LEVEL_WRN=y
	CONFIG_UDC_DRIVER_LOG_LEVEL_WRN=y

	## Examples: Application definitions
	CONFIG_USBD_HID_SUPPORT=y
	CONFIG_USBD_HID_VID=0x18D1
	CONFIG_USBD_HID_PID=0x5999
	CONFIG_USBD_HID_SN="0"

	## Examples: KEYBOARD and TOUCHPAD definitions
	CONFIG_USB_DC_HID_KEYBOARD=y
	CONFIG_USB_DC_HID_VIVALDI=y
	CONFIG_USB_DC_HID_TOUCHPAD=y
	```

- **DTS(.overlay)**
	```c
	/* USB node for IT82xx2 chip */
	zephyr_udc0: &usb0 {
		status = "okay";
		pinctrl-0 = <&usb0_dm_gph5_default
			     &usb0_dp_gph6_default>;
		pinctrl-names = "default";
	};

	/* Example: Keyboard HID node */
	hid_kb_dev: hid_kb_dev {
		compatible = "zephyr,hid-device";
		interface-name = "HID0";
		boot-protocol-code = "keyboard";
		polling-rate = <1>;
		in-report-size = <64>;
	};
	```

	**[NOTE]** Properties of HID node:

		- (required) compatible: should be "zephyr,hid-device"
		- (optional) interface-name: HID device name
		- (required) boot-protocol-code: support "none", "keyboard" and "mouse"
		- (required) polling-rate: input type reports polling rate.
		- (required) in-report-size: The size of the longest input report that the HID device can generate.
		- (optional) out-report-size: The size of the longest output report that the HID device can generate


### Device stack APIs

#### Initialization

- **Step1**: Add descriptors through ```usbd_add_descriptor(usbd_contex, descriptor)```

	Developer can assign **product name**, **manufacturer**, **serial number** and **language** strings by the API ```usbd_add_descriptor(usbd_contex, descriptor)```. Additional, the **PID** and **VID** are defined with the marco ```USBD_DEVICE_DEFINE(usbd_contex, dev, vid, pid)```.
	```c
	/* Assign PID and VID */
	USBD_DEVICE_DEFINE(usbd_hid,
			   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
			   CONFIG_USBD_HID_VID, CONFIG_USBD_HID_PID);

	/* Assign language, manufacturer, product name and serial number */
	USBD_DESC_LANG_DEFINE(lang);
	USBD_DESC_MANUFACTURER_DEFINE(mfr, CONFIG_USBD_HID_MANUFACTURER);
	USBD_DESC_PRODUCT_DEFINE(product, CONFIG_USBD_HID_PRODUCT);
	USBD_DESC_SERIAL_NUMBER_DEFINE(sn, CONFIG_USBD_HID_SN);

	err = usbd_add_descriptor(&usbd_hid, &lang);
	err = usbd_add_descriptor(&usbd_hid, &mfr);
	err = usbd_add_descriptor(&usbd_hid, &product);
	sn.custom_sn = 1;
	err = usbd_add_descriptor(&usbd_hid, &sn);
	```

- **Step2**: Add configuration through ```usbd_add_configuration(usbd_contex, config)```

	Set **self powered** and **remote wakeup** attributes

	```c
	static const uint8_t attributes = USB_SCD_SELF_POWERED | USB_SCD_REMOTE_WAKEUP;

	USBD_CONFIGURATION_DEFINE(config_hid,
				  attributes,
				  CONFIG_USBD_HID_MAX_POWER);

	err = usbd_add_configuration(&usbd_hid, &config_hid);
	```

- **Step3**: Pull nodes that are enabled

	```c
	STRUCT_SECTION_FOREACH(usbd_class_node, node) {
		/* Pull everything that is enabled in our configuration. */
		err = usbd_register_class(&usbd_hid, node->name, 1);
		if (err) {
			LOG_ERR("Failed to register %s (%d)", node->name, err);
			return err;
		}
	}
	```

- **Step4**: Set class, subclass and protocol codes using ```usbd_device_set_code_triple(usbd_contex, class, subcalss, protocol)```
	```c
	usbd_device_set_code_triple(&usbd_hid, 0, 0, 0);
	```

- **Step5**: Register USB notification message callback using ```usbd_msg_register_cb(usbd_contex, callback_function)```
	```c
	err = usbd_msg_register_cb(&usbd_hid, usbd_hid_msg_cb);
	```
	**[NOTE]** USB message notification system:
	``` flow
	+------------+
	| EVENTs     |
	| .remove    |           +------------------------+
	| .ready     |           | USBD CORE              |
	| .suspend   |------- -->| (usbd_core.c/          |-----+
	| .resume    |           |  usbd_event_handler()) |     |
	| .reset     |           +------------------------+     |      +-------------------+    +-------------------+
	| .error     |                                          |      | USBD MESSAGE      |    | APPLICATION       |
	+------------+                                          +----->| (usbd_msg.c)      |--->| (samples\subsys\  |
	                                                        |      +-------------------+    |  usb\cdc_acm\src\ |
	+-----------------+      +--------------------------+   |                               |  main.c)          |
	| SETUP REQ       |----->| CLASS                    |---+                               +-------------------+
	| (class-specific |      | (subsys\usb\device_next\ |
	|  notifications) |      |  class\usbd_cdc_acm.c)   |
	+-----------------+      +--------------------------+
	```

- **Step6**: Initialize USB using ```usbd_init(usbd_contex)```
	```c
	err = usbd_init(&usbd_hid);
	```

- **Step7**: Enable USB using```usbd_enable(usbd_contex)```

	```c
	err = usbd_enable(&usbd_hid);
	```

#### Remote Wakeup

The ```usbd_wakeup_request()``` can be used to wake up the controller in application layer. And then, verify the controller state using ```usbd_is_suspended()``` to ensure that the controller has successfully awakened from the suspended mode.

```c
/* Execute remote wakeup */
ret = usbd_wakeup_request(&usbd_hid);
if (ret != 0) {
	// Failed to wakeup the controller
}

/* Check if the controller is suspended */
if (usbd_is_suspended(&usbd_hid)) {
	// Failed to wakeup the controller
}
```

#### HID class APIs

- **Register HID report descriptors**
	```hid_device_register()```: Register HID device with report descriptors and hid operation callbacks
	```c
	/* HID : Report Descriptor (8-byte: report id, modifier and six keycode bytes) */
	static const uint8_t hid_report_desc[] = {
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_KEYBOARD),
		HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(8),
		HID_INPUT(0x01),
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP_KEYPAD),
		HID_USAGE_MIN8(HID_KEYBOARD_MODIFIER_LOW),
		HID_USAGE_MAX8(HID_KEYBOARD_MODIFIER_HIGH),
		HID_LOGICAL_MIN8(0x00),
		HID_LOGICAL_MAX8(0x01),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(8),
		HID_INPUT(0x02),
		HID_REPORT_COUNT(6),
		HID_REPORT_SIZE(8),
		HID_LOGICAL_MIN8(0x0),
		HID_LOGICAL_MAX8(0xa4),
		HID_USAGE_MIN8(0x00),
		HID_USAGE_MAX8(0xa4),
		HID_INPUT(0x00),
		HID_END_COLLECTION
	};

	static void kb_iface_ready(const struct device *dev, const bool ready)
	{
		// Configuration is ready
	}

	static int kb_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
	{
		// Handle GET_REPORT request
		return 0;
	}

	static void kb_set_protocol(const struct device *dev, uint8_t protocol)
	{
		// Handle SET_PROTOCOL request
		...
	}

	static void kb_in_ready(const struct device *dev)
	{
		// The current interrupt IN transfer has completed.
		...
	}

	/* Define HID operation callbacks*/
	static const struct hid_device_ops ops = {
		.iface_ready = kb_iface_ready,
		.get_report = kb_get_report,
		.set_report = NULL,
		.set_idle = NULL,
		.get_idle = NULL,
		.set_protocol = kb_set_protocol,
		.input_report_done = kb_in_ready,
		.output_report = NULL,
	};

	ret = hid_device_register(hid_dev, hid_report_desc,
				  sizeof(hid_report_desc),
				  &ops);
	if (ret) {
		// Failed to register HID device
	}
	```

- **Submit reports**
	```hid_device_submit_report(dev, report_size, report, sync)```: Submit reports
	```c
	ret = hid_device_submit_report(hid_dev, size, (uint8_t *)report, false);
	if (ret) {
		// Failed to submit report
	}
	```

### Code Flow for HID keyboard sample
![hid keyboard sample](https://drive.google.com/uc?id=1gCvFcBCjTh6Bk2XOf38tcUs15aoHZfPE)

### USB Application Samples
Boards | Tool | Result | Link
-------|------|--------|------
IT82xx2 EVB | hid-mouse    | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/hid-mouse
IT82xx2 EVB | cdc_acm      | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) | https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/cdc_acm
Roach       | keyboard and touchpad | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) ![badge](https://img.shields.io/badge/build%20and%20test-dependencies-blue) | https://source.chromium.org/chromiumos/chromiumos/codesearch/+/main:src/platform/ec/zephyr/subsys/usb_dc/
Roach       | Chapter9 test | ![badge](https://img.shields.io/badge/build%20and%20test-pass-green) ![badge](https://img.shields.io/badge/build%20and%20test-dependencies-blue) | https://www.usb.org/document-library/usb20cv-64-bit-version-1497-xp-version |

**[NOTE]**
1. Roach has dependency on zephyr PRs and application.

	-[PR#65801 usb: device_next: add initial HID device support](https://github.com/zephyrproject-rtos/zephyr/pull/65801)

	-[PR#68256 usb: device_next: implementation of USB device support notification](https://github.com/zephyrproject-rtos/zephyr/pull/68256)

   -[PR#65792 drivers: udc: add IT82xx2 USB device controller driver](https://github.com/zephyrproject-rtos/zephyr/pull/65792)

   -[subsys: usb: Fix boot protocol and buffer length (get_report) issues](https://chromium-review.googlesource.com/c/chromiumos/third_party/zephyr/+/5300041)

   -[roach: (usbd stack) support hid over usb for keyboard and touchpad devices](https://chromium-review.googlesource.com/c/chromiumos/platform/ec/+/5172429/4)

2. USB20CV or USB3CV can only run on the English language version of Windows.

## Limitations
-  For the IT82xx2 chip, multiple endpoints share a single FIFO. The direction of the FIFO cannot be changed during runtime, or the OUT transactions will be lost. Therefore, it is necessary to assign the FIFO direction during build time. Currently, there is one endpoint for control, ten endpoints for IN, and five endpoints for OUT.
	- Endpoint 0: control endpoint
	- Endpoint 1/3/4/6/7/9/10/12/13/15: IN endpoints
	- Endpoint 2/5/8/11/14: OUT endpoints

- The FIFO is stuck if the previous transaction isn't received by controller.

**[NOTE]**

- Shared FIFO

	FIFO index | Endpoints | Direction
	-----------|-----------|-----------
	0 | 0           | bidirectional
	1 | 1 4 7 10 13 | IN
	2 | 2 5 8 11 14 | OUT
	3 | 3 6 9 12 15 | IN

- Directions

	.OUT endpoint: from host to device

	.IN endpoint: from device to host

## References
- Zephyr: [Planned USB device/host support rework/enhancements](https://github.com/zephyrproject-rtos/zephyr/issues/42066)
- Zephyr: [USB](https://docs.zephyrproject.org/latest/connectivity/usb/index.html)
- USB IF: [USB2.0 specification](https://www.usb.org/document-library/usb-20-specification)
- USB IF: [Device Class Definition for HID 1.11](https://www.usb.org/document-library/device-class-definition-hid-111)
- USB IF: [HID Usage Table](https://www.usb.org/document-library/hid-usage-tables-15)