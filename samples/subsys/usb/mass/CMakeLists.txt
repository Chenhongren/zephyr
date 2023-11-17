# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mass)

if((NOT CONFIG_DISK_DRIVER_FLASH) AND
   (NOT CONFIG_DISK_DRIVER_RAM) AND
   (NOT CONFIG_DISK_DRIVER_SDMMC))
	message( FATAL_ERROR "No disk access settings detected." )
endif()

set(SAMPLE_USBD_DIR ${ZEPHYR_BASE}/samples/subsys/usb/common)
FILE(GLOB app_sources src/*.c)

target_include_directories(app PRIVATE ${SAMPLE_USBD_DIR})

target_sources(app PRIVATE ${app_sources})

target_sources_ifdef(CONFIG_USB_DEVICE_STACK_NEXT app PRIVATE
	${SAMPLE_USBD_DIR}/sample_usbd_init.c
)

if(CONFIG_BUILD_WITH_TFM)
  target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
  )
endif()
