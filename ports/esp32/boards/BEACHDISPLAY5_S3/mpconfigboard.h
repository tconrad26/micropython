#ifndef MICROPY_HW_BOARD_NAME
#define MICROPY_HW_BOARD_NAME               "Beach Display v5 (board_v4)"
#endif
#define MICROPY_HW_MCU_NAME                 "ESP32-S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (1)

// Keep the single ESP32-S3 USB OTG PHY free for usb_host.
//
// 2026-08-08: usbhost.init() calling usb_host_install() from MicroPython
// failed with
//     E usb_phy: usb_new_phy(339): selected PHY is in use
//     E USB HOST: PHY install error for port 0: ESP_ERR_INVALID_STATE
// because MicroPython's TinyUSB device stack claims the PHY during
// startup, before any Python runs. That is why board_init.c has to be a
// MICROPY_BOARD_STARTUP hook -- it is the only code that runs earlier.
//
// BOTH of these are required. Disabling only USB_CDC turns SERIAL_JTAG on
// automatically (its default is `!MICROPY_HW_USB_CDC || SOC_USB_OTG_PERIPH_NUM > 1`,
// and SOC_USB_OTG_PERIPH_NUM is 1 here), which shares the same USB pads --
// swapping one squatter for another. The reverted 2026-08-02 experiment
// noted in sdkconfig.board disabled the secondary console while leaving
// USB_CDC on, which is probably why it regressed instead of helping.
//
// Cost: no native-USB CDC REPL on this board. MICROPY_HW_ENABLE_UART_REPL
// above already puts the REPL on UART0 (GPIO43/44), which is the FTDI path
// used for every deploy to BD00010. Boards provisioned over a native-USB
// COM port (BD00006/7/8/9) must NOT take this build until provisioning
// moves off the USB port.
// MICROPY_HW_ENABLE_USBDEV is the MASTER switch and must be 0 too.
// MICROPY_HW_USB_CDC=0 alone only removes the CDC serial *function* --
// the TinyUSB device stack is still compiled and initialised (it also
// backs machine.USBDevice), so it still claims the PHY. Confirmed on
// hardware 2026-08-08: with CDC and SERIAL_JTAG both 0 but USBDEV left
// at its default, usbhost.init() still failed with
//     E usb_phy: usb_new_phy(339): selected PHY is in use
// The 22 leftover tud_/tinyusb symbols in the ELF were the tell.
#define MICROPY_HW_ENABLE_USBDEV            (0)
#define MICROPY_HW_USB_CDC                  (0)
#define MICROPY_HW_ESP_USB_SERIAL_JTAG      (0)

#define MICROPY_HW_I2C0_SCL                 (9)
#define MICROPY_HW_I2C0_SDA                 (8)

// Runs once at boot, before the MicroPython task (and therefore boot.py)
// starts. Puts the BG95-M3 modem's USB link into suspend via the ESP32-S3's
// own native USB acting as host, then hands off to normal startup. See
// board_init.c. Bounded timeout inside -- never blocks boot indefinitely if
// the modem is missing/unresponsive.
#define MICROPY_BOARD_STARTUP                BEACHDISPLAY5_S3_board_startup
void BEACHDISPLAY5_S3_board_startup(void);
