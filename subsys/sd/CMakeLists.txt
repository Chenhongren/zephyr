# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SD_STACK)
  zephyr_interface_library_named(SD)

  zephyr_library()
  zephyr_library_sources(sd.c sd_ops.c)
  zephyr_library_sources_ifdef(CONFIG_SDMMC_STACK sdmmc.c)
  zephyr_library_sources_ifdef(CONFIG_SDIO_STACK sdio.c)
  zephyr_library_sources_ifdef(CONFIG_MMC_STACK mmc.c)

endif()
