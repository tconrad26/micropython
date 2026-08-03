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
    boards/sdkconfig.spiram_oct
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)
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
