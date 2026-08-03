#ifndef MICROPY_HW_BOARD_NAME
#define MICROPY_HW_BOARD_NAME               "Beach Display v5 (board_v4)"
#endif
#define MICROPY_HW_MCU_NAME                 "ESP32-S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (1)

#define MICROPY_HW_I2C0_SCL                 (9)
#define MICROPY_HW_I2C0_SDA                 (8)

// Runs once at boot, before the MicroPython task (and therefore boot.py)
// starts. Puts the BG95-M3 modem's USB link into suspend via the ESP32-S3's
// own native USB acting as host, then hands off to normal startup. See
// board_init.c. Bounded timeout inside -- never blocks boot indefinitely if
// the modem is missing/unresponsive.
#define MICROPY_BOARD_STARTUP                BEACHDISPLAY5_S3_board_startup
void BEACHDISPLAY5_S3_board_startup(void);
