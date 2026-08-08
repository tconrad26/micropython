// usbhost.c -- MicroPython-callable wrapper around the ESP-IDF usb_host
// calls BEACHDISPLAY5_S3's board_init.c already uses successfully at boot,
// exposed so ModemTransport can re-establish USB suspend after a RUNTIME
// modem power-cycle instead of only once at ESP32 boot.
//
// FIRST DRAFT, NOT YET BUILT OR TESTED ON HARDWARE (2026-08-06). Mirrors
// board_init.c's proven attach/suspend/cleanup sequence as closely as
// possible -- same client-register/wait-for-NEW_DEV/open/suspend/close/
// deregister pattern, same deliberate avoidance of usb_host_uninstall()
// (documented elsewhere in this project as unreliable on this hardware).
// See beachdisplay5's memory/project_modem_design.md, "Follow-up items
// still open" -- this is the structural fix recorded there.
//
// PRECONDITION: usb_host_install() must already have been called and its
// event-pump task already running -- board_init.c does exactly this at
// boot and deliberately never uninstalls it, so that precondition already
// holds by the time MicroPython/this module ever runs. This module does
// NOT call usb_host_install() itself and will fail if it somehow does
// not hold (e.g. this module used on a board without that boot hook).
//
// THREADING NOTE: attach()/suspend()/resume() block the calling
// MicroPython thread for up to their own timeout (attach() especially,
// default 10s) while usb_host_client_handle_events() pumps in a loop.
// The existing usb_lib_task (started by board_init.c, pinned to core 0)
// keeps running and servicing the host library throughout -- this
// module's blocking is on the CALLING task only, same as any other
// blocking MicroPython call (e.g. time.sleep()). Callers on the main
// event loop should account for this against their own watchdog-feed
// cadence; this module does not feed any watchdog itself.
//
// NOT YET VERIFIED ON HARDWARE:
//   - Whether registering a fresh client here, after board_init.c's own
//     boot-time client was already closed+deregistered, behaves
//     identically to board_init.c's own first registration -- should,
//     per ESP-IDF's usb_host design (multiple clients against one
//     installed library is supported), but genuinely untested.
//   - Whether attach() can succeed against a modem that was suspended
//     (not power-cycled) -- expected use is always after a power-cycle
//     (fresh USB enumeration), not against an already-suspended link.
//   - usb_host_lib_root_port_resume() is exposed but has never been
//     called anywhere in this project; board_init.c only ever suspends.

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include <string.h>   // MP_EIO -- missing here caused the first-ever compile of this file to fail (2026-08-08)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "usb/usb_host.h"

#define USBHOST_DEFAULT_ATTACH_TIMEOUT_MS 10000

static usb_host_client_handle_t s_client_hdl;
static usb_device_handle_t s_dev_hdl = NULL;
static uint8_t s_connected_addr = 0;
static volatile bool s_device_connected = false;
static bool s_have_open_device = false;

// DIAGNOSTIC (2026-08-08): the bool returns told us a call failed but not
// why. Record the last esp_err so usbhost.last_error() can name it.
static esp_err_t s_last_err = ESP_OK;

// Counters updated by board_init.c's usb_lib_task -- see the note there.
extern volatile uint32_t bd5_usb_lib_event_count;
extern volatile uint32_t bd5_usb_lib_last_flags;   // true between a successful attach() and the next suspend()/detach cleanup

static void usbhost_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        s_connected_addr = event_msg->new_dev.address;
        s_device_connected = true;
    }
}

// usbhost.attach(timeout_ms=10000) -> bool
// Registers a fresh USB host client, waits for the modem to enumerate
// (USB_HOST_CLIENT_EVENT_NEW_DEV), and opens it. Mirrors board_init.c's
// own attach sequence exactly. Returns True on success; on failure,
// cleans up any partially-registered client before returning False so a
// later retry starts from a known-clean state.
static mp_obj_t usbhost_attach(size_t n_args, const mp_obj_t *args) {
    mp_int_t timeout_ms = USBHOST_DEFAULT_ATTACH_TIMEOUT_MS;
    if (n_args >= 1) {
        timeout_ms = mp_obj_get_int(args[0]);
    }

    if (s_have_open_device) {
        // Already attached from a prior call this session -- treat as
        // success rather than silently leaking a second client.
        return mp_const_true;
    }

    s_device_connected = false;
    s_connected_addr = 0;

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usbhost_client_event_cb,
            .callback_arg = NULL,
        },
    };
    if (usb_host_client_register(&client_config, &s_client_hdl) != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }

    // A client is only sent USB_HOST_CLIENT_EVENT_NEW_DEV for devices that
    // enumerate AFTER it registers. board_init.c's boot-time client is long
    // gone, so a device enumerated before this call is invisible to the
    // NEW_DEV wait below -- usb_host_device_addr_list_fill() is the
    // documented way to discover it. Checked FIRST because it is instant
    // when it works and costs nothing when it doesn't.
    uint8_t addr_list[8];
    int num_devs = 0;
    esp_err_t fill_err = usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_devs);
    s_last_err = fill_err;
    if (fill_err == ESP_OK && num_devs > 0) {
        s_connected_addr = addr_list[0];
        s_device_connected = true;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (!s_device_connected && xTaskGetTickCount() < deadline) {
        usb_host_client_handle_events(s_client_hdl, pdMS_TO_TICKS(500));
    }

    if (!s_device_connected) {
        usb_host_client_deregister(s_client_hdl);
        return mp_const_false;
    }

    s_last_err = usb_host_device_open(s_client_hdl, s_connected_addr, &s_dev_hdl);
    if (s_last_err != ESP_OK) {
        usb_host_client_deregister(s_client_hdl);
        return mp_const_false;
    }

    s_have_open_device = true;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbhost_attach_obj, 0, 1, usbhost_attach);

// usbhost.suspend() -> bool
// Calls usb_host_lib_root_port_suspend() (same call board_init.c uses).
// If a device is currently open (via attach()), closes and deregisters
// the client afterward -- confirmed elsewhere in this project that
// close+deregister does not disturb an already-established suspend, and
// deliberately does NOT call usb_host_uninstall() (documented as
// unreliable on this hardware). Requires attach() to have succeeded
// first in this session; raises otherwise rather than silently no-op'ing
// against a device that was never opened.
static mp_obj_t usbhost_suspend(void) {
    if (!s_have_open_device) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("usbhost: attach() must succeed before suspend()"));
    }

    esp_err_t err = usb_host_lib_root_port_suspend();
    s_last_err = err;

    // Settle, then release the client the same way board_init.c does --
    // this is what leaves USB genuinely suspended rather than merely
    // "suspend requested but session still nominally open".
    vTaskDelay(pdMS_TO_TICKS(300));
    usb_host_device_close(s_client_hdl, s_dev_hdl);
    usb_host_client_deregister(s_client_hdl);
    s_have_open_device = false;

    return err == ESP_OK ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_suspend_obj, usbhost_suspend);

// usbhost.resume() -> bool
// Calls usb_host_lib_root_port_resume(). UNTESTED -- never exercised
// anywhere in this project; board_init.c only ever suspends. After a
// resume, the modem's USB link is presumably re-enumerable but this
// module has no open client/device session at that point (suspend()
// already closed it) -- call attach() again to get one.
static mp_obj_t usbhost_resume(void) {
    esp_err_t err = usb_host_lib_root_port_resume();
    s_last_err = err;
    return err == ESP_OK ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_resume_obj, usbhost_resume);

// usbhost.attached() -> bool
// True if attach() has succeeded and suspend() hasn't run yet (i.e. a
// device handle is currently open). Diagnostic/state-check helper --
// does NOT reflect the modem's own AT+QCFGEXT="usb/event" state, which
// must still be read separately over the AT-command UART if that's what
// the caller actually needs.
static mp_obj_t usbhost_attached(void) {
    return s_have_open_device ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_attached_obj, usbhost_attached);

// usbhost.devices() -> tuple of ints
// Addresses the host LIBRARY currently believes are enumerated, via
// usb_host_device_addr_list_fill(). Independent of whether this module has
// a client registered or a device open -- which is exactly the distinction
// needed to tell "the library never saw the device" from "the library saw
// it but our client wasn't told".
static mp_obj_t usbhost_devices(void) {
    uint8_t addr_list[8];
    int num_devs = 0;
    esp_err_t err = usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_devs);
    s_last_err = err;
    if (err != ESP_OK) {
        return mp_const_none;
    }
    mp_obj_t items[8];
    if (num_devs > 8) {
        num_devs = 8;
    }
    for (int i = 0; i < num_devs; i++) {
        items[i] = MP_OBJ_NEW_SMALL_INT(addr_list[i]);
    }
    return mp_obj_new_tuple(num_devs, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_devices_obj, usbhost_devices);

// usbhost.last_error() -> str
// Name of the last esp_err seen by this module. A bool return says a call
// failed; this says what the IDF actually reported.
static mp_obj_t usbhost_last_error(void) {
    const char *name = esp_err_to_name(s_last_err);
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_last_error_obj, usbhost_last_error);

// usbhost.lib_status() -> (event_count, last_flags)
// How many times board_init.c's usb_lib_task has returned from
// usb_host_lib_handle_events, and the flags from the most recent one. If
// this count does not move across a modem power-cycle, the host library is
// not observing the disconnect/connect at all -- which would place the
// fault below this module entirely.
static mp_obj_t usbhost_lib_status(void) {
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint(bd5_usb_lib_event_count),
        mp_obj_new_int_from_uint(bd5_usb_lib_last_flags),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_lib_status_obj, usbhost_lib_status);

static const mp_rom_map_elem_t usbhost_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_usbhost) },
    { MP_ROM_QSTR(MP_QSTR_attach),   MP_ROM_PTR(&usbhost_attach_obj) },
    { MP_ROM_QSTR(MP_QSTR_suspend),  MP_ROM_PTR(&usbhost_suspend_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),   MP_ROM_PTR(&usbhost_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_attached), MP_ROM_PTR(&usbhost_attached_obj) },
    { MP_ROM_QSTR(MP_QSTR_devices),    MP_ROM_PTR(&usbhost_devices_obj) },
    { MP_ROM_QSTR(MP_QSTR_last_error), MP_ROM_PTR(&usbhost_last_error_obj) },
    { MP_ROM_QSTR(MP_QSTR_lib_status), MP_ROM_PTR(&usbhost_lib_status_obj) },
};
static MP_DEFINE_CONST_DICT(usbhost_module_globals, usbhost_module_globals_table);

const mp_obj_module_t usbhost_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&usbhost_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_usbhost, usbhost_user_cmodule);
