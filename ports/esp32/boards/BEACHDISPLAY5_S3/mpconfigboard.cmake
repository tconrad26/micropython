# v1.28.0 predates the mpconfigboard_esp32s3_common.cmake / sdkconfig.
# flash_qio_80m refactor that exists on later master -- inlined here to
# match what that shared file contained at this tag (see
# ESP32_GENERIC_S3/mpconfigboard.cmake for the same pattern). The QIO/80MHz
# flash mode keys that sdkconfig.flash_qio_80m would have supplied are in
# sdkconfig.board instead, for the same reason.
set(IDF_TARGET esp32s3)

if(NOT MICROPY_DIR)
    get_filename_component(MICROPY_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../.. ABSOLUTE)
endif()

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
    boards/sdkconfig.240mhz
    boards/sdkconfig.spiram_sx
    boards/sdkconfig.spiram_oct
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)
# sdkconfig.spiram_oct only overrides CONFIG_SPIRAM_MODE_QUAD/OCT -- it does
# NOT set CONFIG_SPIRAM=y itself (confirmed by reading it: just the mode
# line). It must be layered on top of a base spiram config that does.
# boards/sdkconfig.spiram_quad (used by master's mpconfigboard_esp32s3_
# common.cmake, which we inlined from) doesn't exist as a file at v1.28.0 --
# that name is S2-specific here. The real S3 base at this tag is
# sdkconfig.spiram_sx (CONFIG_SPIRAM=y, BOOT_INIT, USE_MALLOC, CLK/CS IO),
# and this exact spiram_sx + spiram_oct layering is what Espressif's own
# ESP32_GENERIC_S3 "Octal-SPIRAM" variant uses at this same tag (see
# mpconfigvariant_SPIRAM_OCT.cmake) -- confirmed matching, not guessed.
# Inlining only spiram_oct in the first version of this file silently left
# CONFIG_SPIRAM unset entirely (confirmed in build/sdkconfig 2026-08-03:
# "# CONFIG_SPIRAM is not set"), which is almost certainly the real cause
# of the MemoryError at main_0.py import (PSRAM heap never came online,
# leaving only internal SRAM).
# PSRAM: Octal SPI PSRAM (N8R8 module) -- overrides the common S3 board
# config's default sdkconfig.spiram_quad. Briefly second-guessed to Quad
# on 2026-08-02 based on a since-retracted correction; back to Octal per
# the user's own follow-up. Worth confirming decisively on first real boot
# of this build -- a PSRAM mode mismatch (quad firmware config against
# octal hardware, or vice versa) should show up fast as a boot failure/
# crash, not something that fails silently.

set(MICROPY_SOURCE_BOARD
    ${MICROPY_BOARD_DIR}/board_init.c
)
