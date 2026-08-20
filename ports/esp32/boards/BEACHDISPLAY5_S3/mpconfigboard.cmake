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

# boards/sdkconfig.ble REMOVED 2026-08-20 -- Beach Display doesn't use
# Bluetooth/NimBLE anywhere in the application; it was only here because
# this board's config was originally templated from one that had it on.
# Confirmed unused via a full grep of the application code before removing.
#
# Direct cause: bumping this project's pinned ESP-IDF from 5.5.2 to 5.5.3
# (needed for espressif/usb >=1.5.0's automatic root-port-suspend-before-
# light-sleep fix -- see main/idf_component.yml) pulled in a newer NimBLE
# submodule whose ble_hs_priv.h changed ble_hs_max_attrs/max_services/
# max_client_configs from plain globals to a ble_hs_state_ctx-> struct
# member. MicroPython's own extmod/nimble/modbluetooth_nimble.c (shared
# upstream code, not anything in this fork) wasn't updated to match, so
# it fails to compile against the newer NimBLE. Since nothing here uses
# BLE, removing sdkconfig.ble avoids compiling that file at all rather
# than patching shared upstream code for a feature this product doesn't
# use.
#
# If you ever want Bluetooth back on this board: re-add boards/sdkconfig.ble
# below, then either patch modbluetooth_nimble.c for the newer NimBLE
# macro form yourself, or check whether a newer MicroPython release has
# already fixed it upstream first -- that's the more likely place for
# this to get resolved than by hand here.
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
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

# usbhost: MicroPython-callable attach()/suspend()/resume() wrapping the
# same ESP-IDF usb_host calls board_init.c already uses at boot -- lets
# ModemTransport re-establish USB suspend after a runtime modem power-
# cycle instead of only once at ESP32 boot. Board-specific (only
# BEACHDISPLAY5_S3 has the modem wired to native USB), not a general
# ESP32 port module -- see usermods/usbhost/usbhost.c for the full
# rationale and the not-yet-hardware-verified caveats.
#
# Path built from CMAKE_CURRENT_LIST_DIR (this file's own directory,
# boards/BEACHDISPLAY5_S3/) rather than MICROPY_PORT_DIR -- that variable
# is set in esp32_common.cmake, and this file's processing order relative
# to that was not confirmed, so this avoids depending on it. Matches the
# same CMAKE_CURRENT_LIST_DIR-relative style already used for MICROPY_DIR
# at the top of this file.
set(USER_C_MODULES
    ${CMAKE_CURRENT_LIST_DIR}/../../usermods/usbhost
)
