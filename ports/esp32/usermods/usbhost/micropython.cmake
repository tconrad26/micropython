# usbhost -- MicroPython-callable wrapper around the ESP-IDF usb_host
# suspend/resume calls board_init.c already uses at boot. See usbhost.c
# for the full rationale; only included for BEACHDISPLAY5_S3 (see that
# board's mpconfigboard.cmake USER_C_MODULES line), not built for other
# boards/variants.

add_library(usermod_usbhost INTERFACE)

target_sources(usermod_usbhost INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/usbhost.c
)

target_include_directories(usermod_usbhost INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_usbhost)
