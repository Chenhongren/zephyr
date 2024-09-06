"""
Copyright (c) 2021 Zephyr Project members and individual contributors
SPDX-License-Identifier: Apache-2.0

This module contains a variable with a list of tuples (old_url, new_url) for
pages to redirect. This list allows redirecting old URLs (caused by reorganizing
doc directories)

Notes:
    - Please keep this list sorted alphabetically.
    - URLs must be relative to document root (with NO leading slash), and
      without the html extension).
"""

REDIRECTS = [
    # zephyr-keep-sorted-start
    ('application/index', 'develop/application/index'),
    ('boards/arduino/uno_r4_minima/doc/index', 'boards/arduino/uno_r4/doc/index'),
    ('boards/x86/ehl_crb/doc/index', 'boards/x86/intel_ehl/doc/index'),
    ('boards/x86/intel_ehl/doc/index', 'boards/intel/ehl/doc/index'),
    ('boards/x86/intel_rpl/doc/index', 'boards/intel/rpl/doc/index'),
    ('boards/x86/rpl_crb/doc/index', 'boards/x86/intel_rpl/doc/index'),
    ('connectivity/bluetooth/audio', 'connectivity/bluetooth/api/audio/audio'),
    ('connectivity/bluetooth/bap', 'connectivity/bluetooth/api/audio/bap'),
    ('connectivity/bluetooth/bluetooth-audio-arch', 'connectivity/bluetooth/bluetooth-le-audio-arch'),
    ('connectivity/bluetooth/bluetooth-le-audio-arch', 'connectivity/bluetooth/api/audio/bluetooth-le-audio-arch'),
    ('connectivity/bluetooth/cap', 'connectivity/bluetooth/api/audio/cap'),
    ('connectivity/bluetooth/coordinated_sets', 'connectivity/bluetooth/api/audio/coordinated_sets'),
    ('connectivity/bluetooth/dis-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/gap-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/gatt-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/dis', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/gap', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/gatt', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/l2cap', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/mesh', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/rfcomm', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/ics/sm', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/l2cap-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/media', 'connectivity/bluetooth/api/audio/media'),
    ('connectivity/bluetooth/mesh-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/microphone', 'connectivity/bluetooth/api/audio/microphone'),
    ('connectivity/bluetooth/overview', 'connectivity/bluetooth/features'),
    ('connectivity/bluetooth/rfcomm-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/shell/bap', 'connectivity/bluetooth/api/audio/shell/bap'),
    ('connectivity/bluetooth/shell/bap_broadcast_assistant', 'connectivity/bluetooth/api/audio/shell/bap_broadcast_assistant'),
    ('connectivity/bluetooth/shell/bap_scan_delegator', 'connectivity/bluetooth/api/audio/shell/bap_scan_delegator'),
    ('connectivity/bluetooth/shell/cap', 'connectivity/bluetooth/api/audio/shell/cap'),
    ('connectivity/bluetooth/shell/ccp', 'connectivity/bluetooth/api/audio/shell/ccp'),
    ('connectivity/bluetooth/shell/csip', 'connectivity/bluetooth/api/audio/shell/csip'),
    ('connectivity/bluetooth/shell/gmap', 'connectivity/bluetooth/api/audio/shell/gmap'),
    ('connectivity/bluetooth/shell/mcp', 'connectivity/bluetooth/api/audio/shell/mcp'),
    ('connectivity/bluetooth/shell/pbp', 'connectivity/bluetooth/api/audio/shell/pbp'),
    ('connectivity/bluetooth/shell/tmap', 'connectivity/bluetooth/api/audio/shell/tmap'),
    ('connectivity/bluetooth/sm-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('connectivity/bluetooth/volume', 'connectivity/bluetooth/api/audio/volume'),
    ('connectivity/networking/networking-api-usage', 'connectivity/networking/api/index'),
    ('development_process/code_flow', 'project/code_flow'),
    ('development_process/index', 'project/index'),
    ('development_process/issues', 'project/issues'),
    ('development_process/proposals', 'project/proposals'),
    ('getting_started/index', 'develop/getting_started/index'),
    ('getting_started/toolchain_3rd_party_x_compilers', 'develop/toolchains/index'),
    ('getting_started/toolchain_custom_cmake', 'develop/toolchains/custom_cmake'),
    ('getting_started/toolchain_other_x_compilers', 'develop/toolchains/other_x_compilers'),
    ('guides/arch/arm_cortex_m', 'hardware/arch/arm_cortex_m'),
    ('guides/arch/index', 'hardware/arch/index'),
    ('guides/arch/x86', 'hardware/arch/x86'),
    ('guides/beyond-GSG', 'develop/beyond-GSG'),
    ('guides/bluetooth/index', 'connectivity/bluetooth/index'),
    ('guides/bluetooth/sm-pics', 'connectivity/bluetooth/bluetooth-qual'),
    ('guides/build/index', 'build/cmake/index'),
    ('guides/build/kconfig/extensions', 'build/kconfig/extensions'),
    ('guides/build/kconfig/menuconfig', 'build/kconfig/menuconfig'),
    ('guides/build/kconfig/preprocessor-functions', 'build/kconfig/preprocessor-functions'),
    ('guides/build/kconfig/setting', 'build/kconfig/setting'),
    ('guides/build/kconfig/tips', 'build/kconfig/tips'),
    ('guides/coccinelle', 'develop/tools/coccinelle'),
    ('guides/code-relocation', 'kernel/code-relocation'),
    ('guides/crypto/index', 'services/crypto/index'),
    ('guides/crypto/tinycrypt', 'services/crypto/tinycrypt'),
    ('guides/device_mgmt/dfu', 'services/device_mgmt/dfu'),
    ('guides/device_mgmt/index', 'services/device_mgmt/index'),
    ('guides/device_mgmt/mcumgr', 'services/device_mgmt/mcumgr'),
    ('guides/device_mgmt/ota', 'services/device_mgmt/ota'),
    ('guides/dts/api-usage', 'build/dts/api-usage'),
    ('guides/dts/bindings', 'build/dts/bindings'),
    ('guides/dts/design', 'build/dts/design'),
    ('guides/dts/dt-vs-kconfig', 'build/dts/dt-vs-kconfig'),
    ('guides/dts/howtos', 'build/dts/howtos'),
    ('guides/dts/index', 'build/dts/index'),
    ('guides/dts/intro', 'build/dts/intro'),
    ('guides/dts/troubleshooting', 'build/dts/troubleshooting'),
    ('guides/emulator/index', 'hardware/emulator/index'),
    ('guides/env_vars', 'develop/env_vars'),
    ('guides/flash_debug/host-tools', 'develop/flash_debug/host-tools'),
    ('guides/flash_debug/index', 'develop/flash_debug/index'),
    ('guides/flash_debug/probes', 'develop/flash_debug/probes'),
    ('guides/kconfig/extensions', 'build/kconfig/extensions'),
    ('guides/kconfig/index', 'build/kconfig/index'),
    ('guides/kconfig/menuconfig', 'build/kconfig/menuconfig'),
    ('guides/kconfig/preprocessor-functions', 'build/kconfig/preprocessor-functions'),
    ('guides/kconfig/setting', 'build/kconfig/setting'),
    ('guides/kconfig/tips', 'build/kconfig/tips'),
    ('guides/modules', 'develop/modules'),
    ('guides/networking/index', 'connectivity/networking/index'),
    ('guides/optimizations/index', 'develop/optimizations/index'),
    ('guides/optimizations/tools', 'develop/optimizations/tools'),
    ('guides/pinctrl/index', 'hardware/pinctrl/index'),
    ('guides/platformio/index', 'develop/tools/index'),
    ('guides/pm/device', 'services/pm/device'),
    ('guides/pm/device_runtime', 'services/pm/device_runtime'),
    ('guides/pm/index', 'services/pm/index'),
    ('guides/pm/overview', 'services/pm/overview'),
    ('guides/pm/power_domain', 'services/pm/power_domain'),
    ('guides/pm/system', 'services/pm/system'),
    ('guides/portability/index', 'services/portability/index'),
    ('guides/porting/arch', 'hardware/porting/arch'),
    ('guides/porting/board_porting', 'hardware/porting/board_porting'),
    ('guides/porting/index', 'hardware/porting/index'),
    ('guides/porting/shields', 'hardware/porting/shields'),
    ('guides/smf/index', 'services/smf/index'),
    ('guides/test/coverage', 'develop/test/coverage'),
    ('guides/test/index', 'develop/test/index'),
    ('guides/test/twister', 'develop/test/twister'),
    ('guides/test/ztest', 'develop/test/ztest'),
    ('guides/tfm/build', 'services/tfm/build'),
    ('guides/tfm/index', 'services/tfm/index'),
    ('guides/tfm/integration', 'services/tfm/integration'),
    ('guides/tfm/overview', 'services/tfm/overview'),
    ('guides/tfm/requirements', 'services/tfm/requirements'),
    ('guides/tfm/testsuites', 'services/tfm/testsuites'),
    ('guides/west/basics', 'develop/west/basics'),
    ('guides/west/build-flash-debug', 'develop/west/build-flash-debug'),
    ('guides/west/built-in', 'develop/west/built-in'),
    ('guides/west/config', 'develop/west/config'),
    ('guides/west/extensions', 'develop/west/extensions'),
    ('guides/west/index', 'develop/west/index'),
    ('guides/west/install', 'develop/west/install'),
    ('guides/west/manifest', 'develop/west/manifest'),
    ('guides/west/moving-to-west', 'develop/west/moving-to-west'),
    ('guides/west/release-notes', 'develop/west/release-notes'),
    ('guides/west/sign', 'develop/west/sign'),
    ('guides/west/troubleshooting', 'develop/west/troubleshooting'),
    ('guides/west/west-apis', 'develop/west/west-apis'),
    ('guides/west/why', 'develop/west/why'),
    ('guides/west/without-west', 'develop/west/without-west'),
    ('guides/west/workspaces', 'develop/west/workspaces'),
    ('guides/west/zephyr-cmds', 'develop/west/zephyr-cmds'),
    ('guides/zephyr_cmake_package', 'build/zephyr_cmake_package'),
    ('hardware/peripherals/eeprom', 'hardware/peripherals/eeprom/index'),
    ('hardware/peripherals/sensor', 'hardware/peripherals/sensor/index'),
    ('kernel/libc/index', 'develop/languages/c/index'),
    ('reference/api/api_lifecycle', 'develop/api/api_lifecycle'),
    ('reference/api/index', 'develop/api/index'),
    ('reference/api/overview', 'develop/api/overview'),
    ('reference/api/terminology', 'develop/api/terminology'),
    ('reference/drivers/index', 'kernel/drivers/index'),
    ('reference/file_system/index', 'services/file_system/index'),
    ('reference/libc/index', 'kernel/libc/index'),
    ('reference/logging/index', 'services/logging/index'),
    ('reference/misc/notify', 'services/notify'),
    ('reference/misc/timeutil', 'kernel/timeutil'),
    ('reference/modbus/index', 'services/modbus/index'),
    ('reference/networking/sockets', 'connectivity/networking/api/sockets'),
    ('reference/peripherals/adc', 'hardware/peripherals/adc'),
    ('reference/peripherals/dac', 'hardware/peripherals/dac'),
    ('reference/peripherals/dma', 'hardware/peripherals/dma'),
    ('reference/peripherals/eeprom', 'hardware/peripherals/eeprom/index'),
    ('reference/peripherals/espi', 'hardware/peripherals/espi'),
    ('reference/peripherals/flash', 'hardware/peripherals/flash'),
    ('reference/peripherals/gna', 'hardware/peripherals/index'),
    ('reference/peripherals/gpio', 'hardware/peripherals/gpio'),
    ('reference/peripherals/hwinfo', 'hardware/peripherals/hwinfo'),
    ('reference/peripherals/i2c', 'hardware/peripherals/i2c'),
    ('reference/peripherals/index', 'hardware/peripherals/index'),
    ('reference/peripherals/ipm', 'hardware/peripherals/ipm'),
    ('reference/peripherals/kscan', 'hardware/peripherals/kscan'),
    ('reference/peripherals/led', 'hardware/peripherals/led'),
    ('reference/peripherals/mbox', 'hardware/peripherals/mbox'),
    ('reference/peripherals/mdio', 'hardware/peripherals/mdio'),
    ('reference/peripherals/mspi', 'hardware/peripherals/mspi'),
    ('reference/peripherals/peci', 'hardware/peripherals/peci'),
    ('reference/peripherals/pinmux', 'hardware/pinctrl/index'),
    ('reference/peripherals/ps2', 'hardware/peripherals/ps2'),
    ('reference/peripherals/pwm', 'hardware/peripherals/pwm'),
    ('reference/peripherals/reset', 'hardware/peripherals/reset'),
    ('reference/peripherals/rtc', 'hardware/peripherals/rtc'),
    ('reference/peripherals/sensor', 'hardware/peripherals/sensor/index'),
    ('reference/peripherals/spi', 'hardware/peripherals/spi'),
    ('reference/peripherals/tcpc', 'hardware/peripherals/tcpc'),
    ('reference/peripherals/uart', 'hardware/peripherals/uart'),
    ('reference/peripherals/video', 'hardware/peripherals/video'),
    ('reference/pm/index', 'services/pm/api/index'),
    ('reference/settings/index', 'services/settings/index'),
    ('reference/shell/index', 'services/shell/index'),
    ('reference/storage/fcb/fcb', 'services/storage/fcb/fcb'),
    ('reference/storage/index', 'services/storage/index'),
    ('reference/storage/nvs/nvs', 'services/storage/nvs/nvs'),
    ('reference/task_wdt/index', 'services/task_wdt/index'),
    ('reference/usb/hid', 'connectivity/usb/device/api/usb_device_hid'),
    ('reference/usb/index', 'connectivity/usb/device/usb_device'),
    ('reference/usb/udc', 'connectivity/usb/device/api/usb_dc'),
    ('reference/usb/uds', 'connectivity/usb/device/usb_device'),
    ('reference/usb/uds_cdc_acm', 'connectivity/usb/device/usb_device'),
    ('reference/usb/uds_testing', 'connectivity/usb/device/usb_device'),
    ('reference/usermode/index', 'kernel/usermode/index'),
    ('reference/usermode/overview', 'kernel/usermode/overview'),
    ('reference/usermode/syscalls', 'kernel/usermode/syscalls'),
    ('reference/util/index', 'kernel/util/index'),
    ('samples/application_development/with_mcuboot/README', 'samples/sysbuild/with_mcuboot/README'),
    ('samples/bluetooth/broadcast_audio_assistant/README', 'samples/bluetooth/bap_broadcast_assistant/README'),
    ('samples/bluetooth/broadcast_audio_sink/README', 'samples/bluetooth/bap_broadcast_sink/README'),
    ('samples/bluetooth/broadcast_audio_source/README', 'samples/bluetooth/bap_broadcast_source/README'),
    ('samples/bluetooth/central_iso/README', 'samples/bluetooth/iso_central/README'),
    ('samples/bluetooth/peripheral_iso/README', 'samples/bluetooth/iso_peripheral/README'),
    ('samples/bluetooth/public_broadcast_sink/README', 'samples/bluetooth/pbp_public_broadcast_sink/README'),
    ('samples/bluetooth/public_broadcast_source/README', 'samples/bluetooth/pbp_public_broadcast_source/README'),
    ('samples/bluetooth/unicast_audio_client/README', 'samples/bluetooth/bap_unicast_client/README'),
    ('samples/bluetooth/unicast_audio_server/README', 'samples/bluetooth/bap_unicast_server/README'),
    ('samples/boards/96b_argonkey/microphone/README', 'samples/boards/96boards/argonkey/microphone/README'),
    ('samples/boards/96b_argonkey/sensors/README', 'samples/boards/96boards/argonkey/sensors/README'),
    ('samples/boards/esp32/deep_sleep/README', 'samples/boards/espressif/deep_sleep/README'),
    ('samples/boards/esp32/flash_encryption/README', 'samples/boards/espressif/flash_encryption/README'),
    ('samples/boards/esp32/flash_memory_mapped/README', 'samples/boards/espressif/flash_memory_mapped/README'),
    ('samples/boards/esp32/light_sleep/README', 'samples/boards/espressif/light_sleep/README'),
    ('samples/boards/esp32/spiram_test/README', 'samples/boards/espressif/spiram_test/README'),
    ('samples/boards/esp32/wifi_apsta_mode/README', 'samples/boards/espressif/wifi_apsta_mode/README'),
    ('samples/boards/esp32/xt_wdt/README', 'samples/boards/espressif/xt_wdt/README'),
    ('samples/boards/mec15xxevb_assy6853/power_management/README', 'samples/boards/microchip/mec15xxevb_assy6853/power_management/README'),
    ('samples/boards/mimxrt1060_evk/system_off/README', 'samples/boards/nxp/mimxrt1060_evk/system_off/README'),
    ('samples/boards/mimxrt1170_evk_cm7/magic_addr/README', 'samples/boards/nxp/mimxrt1170_evk_cm7/magic_addr/README'),
    ('samples/boards/mimxrt595_evk/system_off/README', 'samples/boards/nxp/mimxrt595_evk/system_off/README'),
    ('samples/boards/nrf/battery/README', 'samples/boards/nordic/battery/README'),
    ('samples/boards/nrf/clock_skew/README', 'samples/boards/nordic/clock_skew/README'),
    ('samples/boards/nrf/dynamic_pinctrl/README', 'samples/boards/nordic/dynamic_pinctrl/README'),
    ('samples/boards/nrf/ieee802154/802154_rpmsg/README', 'samples/boards/nordic/ieee802154/802154_rpmsg/README'),
    ('samples/boards/nrf/mesh/onoff-app/README', 'samples/boards/nordic/mesh/onoff-app/README'),
    ('samples/boards/nrf/mesh/onoff_level_lighting_vnd_app/README', 'samples/boards/nordic/mesh/onoff_level_lighting_vnd_app/README'),
    ('samples/boards/nrf/nrf53_sync_rtc/README', 'samples/boards/nordic/nrf53_sync_rtc/README'),
    ('samples/boards/nrf/nrf_led_matrix/README', 'samples/boards/nordic/nrf_led_matrix/README'),
    ('samples/boards/nrf/nrfx/README', 'samples/boards/nordic/nrfx/README'),
    ('samples/boards/nrf/nrfx_prs/README', 'samples/boards/nordic/nrfx_prs/README'),
    ('samples/boards/nrf/system_off/README', 'samples/boards/nordic/system_off/README'),
    ('samples/boards/nxp_s32/netc/README', 'samples/boards/nxp/s32/netc/README'),
    ('samples/boards/qomu/README', 'samples/boards/quicklogic/qomu/README'),
    ('samples/boards/stm32/backup_sram/README', 'samples/boards/st/backup_sram/README'),
    ('samples/boards/stm32/bluetooth/interactive_gui/README', 'samples/boards/st/bluetooth/interactive_gui/README'),
    ('samples/boards/stm32/ccm/README', 'samples/boards/st/ccm/README'),
    ('samples/boards/stm32/h7_dual_core/README', 'samples/boards/st/h7_dual_core/README'),
    ('samples/boards/stm32/i2c_timing/README', 'samples/boards/st/i2c_timing/README'),
    ('samples/boards/stm32/mco/README', 'samples/boards/st/mco/README'),
    ('samples/boards/stm32/power_mgmt/adc/README', 'samples/boards/st/power_mgmt/adc/README'),
    ('samples/boards/stm32/power_mgmt/blinky/README', 'samples/boards/st/power_mgmt/blinky/README'),
    ('samples/boards/stm32/power_mgmt/serial_wakeup/README', 'samples/boards/st/power_mgmt/serial_wakeup/README'),
    ('samples/boards/stm32/power_mgmt/standby_shutdown/README', 'samples/boards/st/power_mgmt/standby_shutdown/README'),
    ('samples/boards/stm32/power_mgmt/stm32wb_ble/README', 'samples/boards/st/power_mgmt/stm32wb_ble/README'),
    ('samples/boards/stm32/power_mgmt/stop3/README', 'samples/boards/st/power_mgmt/stop3/README'),
    ('samples/boards/stm32/power_mgmt/suspend_to_ram/README', 'samples/boards/st/power_mgmt/suspend_to_ram/README'),
    ('samples/boards/stm32/power_mgmt/wkup_pins/README', 'samples/boards/st/power_mgmt/wkup_pins/README'),
    ('samples/boards/stm32/sensortile_box/README', 'samples/boards/st/sensortile_box/README'),
    ('samples/boards/stm32/sensortile_box_pro/sensors-on-board/README', 'samples/boards/st/sensortile_box_pro/sensors-on-board/README'),
    ('samples/boards/stm32/steval_stwinbx1/sensors/README', 'samples/boards/st/steval_stwinbx1/sensors/README'),
    ('samples/drivers/adc/README', 'samples/drivers/adc/adc_dt/README'),
    ('samples/drivers/kscan_touch', 'samples/subsys/input/input'),
    ('samples/drivers/led_apa102/README', 'samples/drivers/led_strip/README'),
    ('samples/drivers/led_is31fl3194/README', 'samples/drivers/led/is31fl3194/README'),
    ('samples/drivers/led_is31fl3216a/README', 'samples/drivers/led/is31fl3216a/README'),
    ('samples/drivers/led_is31fl3733/README', 'samples/drivers/led/is31fl3733/README'),
    ('samples/drivers/led_lp3943/README', 'samples/drivers/led/lp3943/README'),
    ('samples/drivers/led_lp50xx/README', 'samples/drivers/led/lp50xx/README'),
    ('samples/drivers/led_lp5562/README', 'samples/drivers/led/lp5562/README'),
    ('samples/drivers/led_lp5569/README', 'samples/drivers/led/lp5569/README'),
    ('samples/drivers/led_lpd8806/README', 'samples/drivers/led_strip/README'),
    ('samples/drivers/led_pca9633/README', 'samples/drivers/led/pca9633/README'),
    ('samples/drivers/led_pwm/README', 'samples/drivers/led/pwm/README'),
    ('samples/drivers/led_strip/README', 'samples/drivers/led/led_strip/README'),
    ('samples/drivers/led_sx1509b_intensity/README', 'samples/drivers/led/sx1509b_intensity/README'),
    ('samples/drivers/led_ws2812/README', 'samples/drivers/led_strip/README'),
    ('samples/drivers/led_xec/README', 'samples/drivers/led/xec/README'),
    ('samples/net/cloud/google_iot_mqtt/README', 'samples/net/net'),
    ('samples/sensor/wsen_hids/README', 'samples/sensor/sensor'),
    ('samples/sensor/wsen_itds/README', 'samples/sensor/sensor'),
    ('samples/subsys/video/capture/README', 'samples/drivers/video/capture/README'),
    ('samples/subsys/video/tcpserversink/README', 'samples/drivers/video/tcpserversink/README'),
    ('samples/subsys/video/video', 'samples/drivers/video/video'),
    ('services/crypto/tinycrypt', 'services/crypto/psa_crypto'),
    ('services/portability/posix', 'services/portability/posix/index'),
    # zephyr-keep-sorted-stop
]
