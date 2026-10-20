// usbhost.c -- MicroPython-callable wrapper around the ESP-IDF usb_host
// calls BEACHDISPLAY5_S3's board_init.c uses at boot, exposed so
// ModemTransport can re-establish USB suspend after a RUNTIME modem
// power-cycle instead of only once at ESP32 boot.
//
// PERSISTENT-CLIENT REWRITE, 2026-08-08. The previous design mirrored
// board_init.c exactly -- register client, wait for NEW_DEV, open,
// suspend, close, deregister, per call. That is correct for a one-shot
// boot hook and provably wrong for runtime use. Measured on BD00010 with
// MODEM_PG (GPIO40) confirming the rail actually collapsed and the root
// port resumed BEFORE the power-cycle so the controller could see it:
//
//     MODEM_PG      1 -> 0 -> 1      real power cycle, verified
//     lib_events    17 -> 18         ONE event, not the expected two
//     devices       (1,) throughout  never cleared, before/during/after
//     attach()      True, ESP_OK     opened the STALE address 1
//     suspend()     True, ESP_OK
//     modem usb/event = 0            the modem never saw a suspend
//
// ESP-IDF frees a device only after a registered client is delivered
// USB_HOST_CLIENT_EVENT_DEV_GONE and releases its handle. With the client
// deregistered between calls there was nobody to deliver DEV_GONE to, so
// address 1 was never freed, the port never became re-enumerable, and
// suspend() succeeded against a zombie -- ESP_OK on a port with no live
// device behind it. Exactly the return-value-vs-reality trap this project
// already hit with disusb, which is why the tests validate over the AT
// interface and never trust a bool.
//
// So this version:
//   - registers ONE client on first use and keeps it forever;
//   - runs its own task pumping usb_host_client_handle_events, so
//     NEW_DEV and DEV_GONE are actually received;
//   - closes the device handle from the DEV_GONE callback and calls
//     usb_host_device_free_all(), which is what lets the library reclaim
//     the address;
//   - never deregisters, including in suspend().
//
// Python-side calls WAIT ON FLAGS and deliberately do not pump client
// events themselves -- two tasks calling usb_host_client_handle_events on
// one client is not safe.
//
// PRECONDITION: usb_host_install() must already have been called and its
// library event pump running. board_init.c does exactly that at boot and
// deliberately never uninstalls (usb_host_uninstall() is documented
// elsewhere in this project as unreliable on this hardware). This module
// does not install the library itself.

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCore (ESP-IDF 5.x)
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#define USBHOST_DEFAULT_ATTACH_TIMEOUT_MS 10000
// usb_host_lib_root_port_suspend()/_resume() are asynchronous -- ESP_OK from
// either call means "queued", not "done" (see their doc comments in
// usb_host.h). This bounds how long usbhost_suspend()/usbhost_resume() wait
// for the matching DEV_SUSPENDED/DEV_RESUMED client event that confirms the
// queued action actually completed, before giving up and returning False.
// Not a measured figure -- generous over the expected halt+flush+transition
// sequence, which the docs describe as a few endpoint operations, not
// anything that should take anywhere near this long in practice.
#define USBHOST_SUSPEND_RESUME_TIMEOUT_MS 1000

static const char *TAG = "usbhost";

static usb_host_client_handle_t s_client_hdl = NULL;
static TaskHandle_t s_client_task = NULL;
static usb_device_handle_t s_dev_hdl = NULL;

static volatile uint8_t s_connected_addr = 0;
static volatile bool s_device_present = false;   // library says a device is there
static volatile bool s_have_open_device = false; // we hold an open handle
static volatile uint32_t s_new_dev_count = 0;
static volatile uint32_t s_dev_gone_count = 0;

// Set by DEV_SUSPENDED/DEV_RESUMED client events -- the actual completion
// signal for usb_host_lib_root_port_suspend()/_resume(), which are
// themselves asynchronous. See USBHOST_SUSPEND_RESUME_TIMEOUT_MS.
static volatile bool s_dev_suspended = false;
static volatile bool s_dev_resumed = false;

static esp_err_t s_last_err = ESP_OK;

// True if THIS module called usb_host_install(), meaning we also own the
// library event pump. False when board_init.c installed it at boot (the
// historical arrangement) -- then its usb_lib_task is already pumping and
// we must not start a second one.
static bool s_lib_installed_by_us = false;
static TaskHandle_t s_lib_task = NULL;

// Counters updated by board_init.c's usb_lib_task.
extern volatile uint32_t bd5_usb_lib_event_count;
extern volatile uint32_t bd5_usb_lib_last_flags;

// ---------------------------------------------------------------------------

static void usbhost_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            s_connected_addr = event_msg->new_dev.address;
            s_device_present = true;
            s_new_dev_count++;
            ESP_LOGI(TAG, "NEW_DEV addr=%d", (int)s_connected_addr);
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            // THE POINT OF THE REWRITE. The library will not free the
            // device -- and the port will not become re-enumerable -- until
            // every client that has it open closes it in response to this
            // event. Previously no client existed here at all.
            s_dev_gone_count++;
            if (s_have_open_device) {
                usb_host_device_close(s_client_hdl, s_dev_hdl);
                s_have_open_device = false;
                s_dev_hdl = NULL;
            }
            s_device_present = false;
            s_connected_addr = 0;
            // FIXED 2026-09-18: neither flag was ever cleared here, so a
            // device that disconnected while suspended stayed reported as
            // dev_suspended=True (confirmed on hardware: client_status()
            // showed device_present=False, have_open_device=False, and
            // dev_suspended=True simultaneously, in tio_9_18_0.log). A
            // fresh DEV_GONE means whatever suspend/resume state existed
            // is gone with it -- both are stale from here regardless of
            // what happens next (a same-address recovery or a genuinely
            // new device at a new address).
            s_dev_suspended = false;
            s_dev_resumed = false;
            // Best-effort: may return ESP_ERR_NOT_FINISHED if another
            // client still holds something. Not fatal.
            usb_host_device_free_all();
            ESP_LOGI(TAG, "DEV_GONE handled, device released");
            break;

        case USB_HOST_CLIENT_EVENT_DEV_SUSPENDED:
            // THE completion signal for usb_host_lib_root_port_suspend() --
            // that call only queues the suspend; this is when it actually
            // happened (port transitioned, every endpoint of every open
            // device halted and flushed, clients notified). Previously
            // nothing here waited for this at all -- usbhost_suspend()
            // returned True as soon as the request was queued, which could
            // race the client task actually processing it.
            s_dev_suspended = true;
            s_dev_resumed = false;
            ESP_LOGI(TAG, "DEV_SUSPENDED");
            break;

        case USB_HOST_CLIENT_EVENT_DEV_RESUMED:
            // Same asynchrony on the resume side. This firing is what
            // actually means the device's endpoints are live again --
            // ESP_OK from usb_host_lib_root_port_resume() alone does not.
            s_dev_resumed = true;
            s_dev_suspended = false;
            ESP_LOGI(TAG, "DEV_RESUMED");
            break;

        default:
            break;
    }
}

// Owns client event pumping for the life of the system. Nothing else may
// call usb_host_client_handle_events on this client.
static void usbhost_client_task(void *arg) {
    while (1) {
        usb_host_client_handle_events(s_client_hdl, portMAX_DELAY);
    }
}

// Library-level event pump. Only started when this module installed the
// library itself; otherwise board_init.c's usb_lib_task owns this and
// starting a second pump would be wrong.
static void usbhost_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        bd5_usb_lib_event_count++;
        bd5_usb_lib_last_flags = event_flags;
    }
}

// Install the host library if nobody has. ESP_ERR_INVALID_STATE means it is
// already installed -- board_init.c does that at boot when the
// modem_usb_host_cable NVS flag is set -- which is success for our purposes.
static bool usbhost_ensure_lib(void) {
    if (s_lib_installed_by_us) {
        return true;
    }
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    esp_err_t err = usb_host_install(&host_config);
    s_last_err = err;
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "usb_host already installed (board_init.c); using it");
        return true;
    }
    if (err != ESP_OK) {
        return false;
    }
    if (xTaskCreatePinnedToCore(usbhost_lib_task, "usbhost_lib", 4096,
                                NULL, 5, &s_lib_task, 0) != pdPASS) {
        s_last_err = ESP_ERR_NO_MEM;
        return false;
    }
    s_lib_installed_by_us = true;
    ESP_LOGI(TAG, "usb_host installed by usbhost module");
    return true;
}

// Seed s_device_present from devices that enumerated before our client
// existed -- a client is only sent NEW_DEV for enumerations that happen
// after it registers.
static void usbhost_seed_from_addr_list(void) {
    uint8_t addr_list[8];
    int num_devs = 0;
    esp_err_t err = usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_devs);
    s_last_err = err;
    if (err == ESP_OK && num_devs > 0) {
        s_connected_addr = addr_list[0];
        s_device_present = true;
    }
}

static bool usbhost_ensure_client(void) {
    if (s_client_hdl != NULL) {
        return true;
    }
    if (!usbhost_ensure_lib()) {
        return false;
    }
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usbhost_client_event_cb,
            .callback_arg = NULL,
        },
    };
    esp_err_t err = usb_host_client_register(&client_config, &s_client_hdl);
    s_last_err = err;
    if (err != ESP_OK) {
        s_client_hdl = NULL;
        return false;
    }
    if (xTaskCreatePinnedToCore(usbhost_client_task, "usbhost_cli", 4096,
                                NULL, 5, &s_client_task, 0) != pdPASS) {
        usb_host_client_deregister(s_client_hdl);
        s_client_hdl = NULL;
        s_last_err = ESP_ERR_NO_MEM;
        return false;
    }
    usbhost_seed_from_addr_list();
    ESP_LOGI(TAG, "persistent client registered");
    return true;
}

// Is s_connected_addr still in the library's list? Guards against the
// stale-handle bug: attach() used to early-return True whenever
// s_have_open_device was set, which after a power-cycle meant "success"
// in 1ms for a device that no longer existed.
static bool usbhost_addr_still_listed(uint8_t addr) {
    uint8_t addr_list[8];
    int num_devs = 0;
    if (usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &num_devs) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < num_devs; i++) {
        if (addr_list[i] == addr) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------

// usbhost.init() -> bool
// Register the persistent client early, so DEV_GONE is received even for a
// disconnect that happens before any attach(). Call once at startup;
// attach() calls it implicitly, but by then a disconnect may already have
// been missed.
static mp_obj_t usbhost_init(void) {
    return usbhost_ensure_client() ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_init_obj, usbhost_init);

// usbhost.attach(timeout_ms=10000) -> bool
static mp_obj_t usbhost_attach(size_t n_args, const mp_obj_t *args) {
    mp_int_t timeout_ms = USBHOST_DEFAULT_ATTACH_TIMEOUT_MS;
    if (n_args >= 1) {
        timeout_ms = mp_obj_get_int(args[0]);
    }
    if (!usbhost_ensure_client()) {
        mp_raise_OSError(MP_EIO);
    }

    // Revalidate rather than trusting our own flag.
    if (s_have_open_device) {
        if (usbhost_addr_still_listed(s_connected_addr)) {
            return mp_const_true;
        }
        usb_host_device_close(s_client_hdl, s_dev_hdl);
        s_have_open_device = false;
        s_dev_hdl = NULL;
        s_device_present = false;
    }

    if (!s_device_present) {
        usbhost_seed_from_addr_list();
    }

    // Wait on the flag; the client task does the pumping.
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (!s_device_present && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_device_present) {
        // FIXED 2026-08-20: this path used to return False without
        // touching s_last_err at all, so usbhost.last_error() reported
        // whatever stale value happened to be left over from some
        // earlier, unrelated call -- confirmed misleading on real
        // hardware: a repeatable cold-boot enumeration failure (SET_
        // CONFIGURATION STALLing, per the ENUM: CHECK_CONFIG FAILED log
        // from espressif/usb's own enum.c) reported err=ESP_OK here,
        // looking like success despite attach() returning False. This
        // branch is reached specifically when enumeration never gets far
        // enough to deliver a NEW_DEV client event at all (e.g. it dies
        // during SET_CONFIG, before s_device_present would ever be set)
        // -- record that honestly instead of leaving the caller to infer
        // it from a stale/unrelated error code.
        s_last_err = ESP_ERR_TIMEOUT;
        return mp_const_false;
    }

    s_last_err = usb_host_device_open(s_client_hdl, s_connected_addr, &s_dev_hdl);
    if (s_last_err != ESP_OK) {
        s_dev_hdl = NULL;
        return mp_const_false;
    }
    s_have_open_device = true;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbhost_attach_obj, 0, 1, usbhost_attach);

// usbhost.suspend() -> bool
// Suspends the root port and, unlike the previous version, KEEPS the
// client registered and the device open. board_init.c closes only because
// it is about to exit; a runtime caller that closes has no way to be told
// the device later disappeared.
//
// usb_host_lib_root_port_suspend() is asynchronous (see usb_host.h's doc
// comment on it): ESP_OK means the suspend was queued, not that it
// happened yet -- the actual halt+flush+transition sequence runs later, in
// whatever task pumps usb_host_lib_handle_events() (usbhost_lib_task
// here). Returning True the instant the request is queued -- what this
// function used to do -- gives the caller no guarantee the port has
// actually finished suspending before it goes on to whatever comes next.
// Waits for the real completion signal (DEV_SUSPENDED, set in
// usbhost_client_event_cb()) instead.
static mp_obj_t usbhost_suspend(void) {
    if (!s_have_open_device) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("usbhost: attach() must succeed before suspend()"));
    }
    // Clear before issuing the request, not after -- a stale True left over
    // from a previous cycle must not cause an immediate false-positive return.
    s_dev_suspended = false;
    s_last_err = usb_host_lib_root_port_suspend();
    if (s_last_err != ESP_OK) {
        return mp_const_false;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(USBHOST_SUSPEND_RESUME_TIMEOUT_MS);
    while (!s_dev_suspended && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_dev_suspended) {
        ESP_LOGW(TAG, "suspend() queued but DEV_SUSPENDED not observed within %dms",
                 USBHOST_SUSPEND_RESUME_TIMEOUT_MS);
        return mp_const_false;
    }
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_suspend_obj, usbhost_suspend);

// usbhost.resume() -> bool
// ESP_ERR_NOT_ALLOWED here means the port was not suspended (may already
// be usable) -- exit_edrx() treats that case as non-fatal, see its own
// comment.
//
// Same asynchrony as usbhost_suspend() -- ESP_OK from
// usb_host_lib_root_port_resume() means "queued", not "the device's
// endpoints are live again". Waits for DEV_RESUMED instead of trusting
// the queue-request return value alone.
static mp_obj_t usbhost_resume(void) {
    s_dev_resumed = false;
    s_last_err = usb_host_lib_root_port_resume();
    if (s_last_err != ESP_OK) {
        return mp_const_false;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(USBHOST_SUSPEND_RESUME_TIMEOUT_MS);
    while (!s_dev_resumed && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_dev_resumed) {
        ESP_LOGW(TAG, "resume() queued but DEV_RESUMED not observed within %dms",
                 USBHOST_SUSPEND_RESUME_TIMEOUT_MS);
        return mp_const_false;
    }
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_resume_obj, usbhost_resume);

// usbhost.attached() -> bool
static mp_obj_t usbhost_attached(void) {
    return s_have_open_device ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_attached_obj, usbhost_attached);

// usbhost.devices() -> tuple of ints
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
static mp_obj_t usbhost_last_error(void) {
    const char *name = esp_err_to_name(s_last_err);
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_last_error_obj, usbhost_last_error);

// usbhost.lib_status() -> (event_count, last_flags)
static mp_obj_t usbhost_lib_status(void) {
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint(bd5_usb_lib_event_count),
        mp_obj_new_int_from_uint(bd5_usb_lib_last_flags),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_lib_status_obj, usbhost_lib_status);

// usbhost.client_status() -> (registered, new_dev_count, dev_gone_count,
//                             device_present, have_open_device, addr,
//                             lib_installed_by_us, dev_suspended, dev_resumed)
// dev_gone_count is the number that matters: if it stays 0 across a
// verified modem power-cycle, DEV_GONE is still not being delivered and
// this rewrite did not solve the problem.
// dev_suspended/dev_resumed added alongside the suspend()/resume() wait-
// for-completion fix -- lets a diagnostic confirm on real hardware that
// the events are actually arriving, not just that the timeout isn't firing.
static mp_obj_t usbhost_client_status(void) {
    mp_obj_t items[9] = {
        s_client_hdl != NULL ? mp_const_true : mp_const_false,
        mp_obj_new_int_from_uint(s_new_dev_count),
        mp_obj_new_int_from_uint(s_dev_gone_count),
        s_device_present ? mp_const_true : mp_const_false,
        s_have_open_device ? mp_const_true : mp_const_false,
        MP_OBJ_NEW_SMALL_INT(s_connected_addr),
        s_lib_installed_by_us ? mp_const_true : mp_const_false,
        s_dev_suspended ? mp_const_true : mp_const_false,
        s_dev_resumed ? mp_const_true : mp_const_false,
    };
    return mp_obj_new_tuple(9, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_client_status_obj, usbhost_client_status);

// ---------------------------------------------------------------------------
// Bulk transfers on the open device (2026-10-20).
//
// Why: when the BG95 crashes with AT+QCFGEXT="dump",1 it re-enumerates as the Qualcomm ram-dump device
// ("QUSB_BULK_SN:...", ONE interface, a bulk IN + a bulk OUT endpoint). Reading that dump over USB needs bulk
// transfers; until now this module only suspended/resumed. The ESP32-S3 host is full-speed only, so the dump
// runs at 12 Mbit/s here (the Pi's high-speed link failed with babble/protocol errors on this unit).
//
// API (all need a successful attach() first):
//   usbhost.dev_info()            -> (vid, pid, speed, manufacturer, product, serial) or None
//   usbhost.endpoints()           -> [(interface, ep_addr, bmAttributes, max_packet), ...]
//   usbhost.claim(iface[, alt])   -> bool       usbhost.release(iface) -> bool
//   usbhost.bulk_in(ep, buf, timeout_ms=1000)   fills `buf` (writable), returns bytes received
//   usbhost.bulk_out(ep, data, timeout_ms=1000) returns bytes sent
//   usbhost.xfer_status()         -> raw usb_transfer_status_t of the last transfer (0 = completed,
//                                    1 error, 2 timed out, 3 canceled, 4 stall, 5 OVERFLOW, 6 skipped, 7 no device)
// Failures set usbhost.last_error(); bulk_in/out return the bytes moved before the failure.
// ---------------------------------------------------------------------------

#define USBHOST_BULK_CHUNK 4096     // multiple of 512 so IN requests satisfy every max-packet size

static volatile bool s_xfer_done = false;
static volatile int s_xfer_status = 0;
static volatile int s_xfer_actual = 0;

static void usbhost_xfer_cb(usb_transfer_t *t) {
    s_xfer_status = (int)t->status;
    s_xfer_actual = t->actual_num_bytes;
    s_xfer_done = true;
}

// Submit one prepared transfer and wait for its callback. Polls like the rest of this module (no GIL games).
// Returns ESP_OK only for a COMPLETED transfer. On a timeout the endpoint is halted/flushed so the callback
// fires; if even that does not complete, *leaked is set and the caller must NOT free the transfer.
static esp_err_t usbhost_run_xfer(usb_transfer_t *x, uint8_t ep, size_t n, int timeout_ms, bool *leaked) {
    x->device_handle = s_dev_hdl;
    x->bEndpointAddress = ep;
    x->num_bytes = n;
    x->callback = usbhost_xfer_cb;
    x->context = NULL;
    x->timeout_ms = 0;
    s_xfer_done = false;
    s_xfer_status = 0;
    s_xfer_actual = 0;
    esp_err_t err = usb_host_transfer_submit(x);
    if (err != ESP_OK) {
        return err;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (!s_xfer_done && xTaskGetTickCount() < deadline) {
        vTaskDelay(1);
    }
    if (!s_xfer_done) {
        usb_host_endpoint_halt(s_dev_hdl, ep);
        usb_host_endpoint_flush(s_dev_hdl, ep);
        TickType_t d2 = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
        while (!s_xfer_done && xTaskGetTickCount() < d2) {
            vTaskDelay(1);
        }
        usb_host_endpoint_clear(s_dev_hdl, ep);
        if (!s_xfer_done) {
            *leaked = true;
        }
        return ESP_ERR_TIMEOUT;
    }
    return (s_xfer_status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
}

static void usbhost_require_open(void) {
    if (!s_have_open_device || s_dev_hdl == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("usbhost: attach() must succeed first"));
    }
}

static mp_obj_t usbhost_str_desc_obj(const usb_str_desc_t *s) {
    if (s == NULL || s->bLength < 2) {
        return mp_const_none;
    }
    char buf[64];
    int n = (s->bLength - 2) / 2;
    if (n > (int)sizeof(buf) - 1) {
        n = sizeof(buf) - 1;
    }
    for (int i = 0; i < n; i++) {
        buf[i] = (char)(s->wData[i] & 0x7f);
    }
    return mp_obj_new_str(buf, n);
}

static mp_obj_t usbhost_dev_info(void) {
    usbhost_require_open();
    usb_device_info_t info;
    s_last_err = usb_host_device_info(s_dev_hdl, &info);
    if (s_last_err != ESP_OK) {
        return mp_const_none;
    }
    const usb_device_desc_t *dd = NULL;
    usb_host_get_device_descriptor(s_dev_hdl, &dd);
    mp_obj_t t[6];
    t[0] = dd ? MP_OBJ_NEW_SMALL_INT(dd->idVendor) : mp_const_none;
    t[1] = dd ? MP_OBJ_NEW_SMALL_INT(dd->idProduct) : mp_const_none;
    t[2] = MP_OBJ_NEW_SMALL_INT((int)info.speed);        // 0 low, 1 full, 2 high
    t[3] = usbhost_str_desc_obj(info.str_desc_manufacturer);
    t[4] = usbhost_str_desc_obj(info.str_desc_product);
    t[5] = usbhost_str_desc_obj(info.str_desc_serial_num);
    return mp_obj_new_tuple(6, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_dev_info_obj, usbhost_dev_info);

static mp_obj_t usbhost_endpoints(void) {
    usbhost_require_open();
    const usb_config_desc_t *cfg = NULL;
    s_last_err = usb_host_get_active_config_descriptor(s_dev_hdl, &cfg);
    if (s_last_err != ESP_OK || cfg == NULL) {
        return mp_const_none;
    }
    const uint8_t *p = (const uint8_t *)cfg;
    int total = cfg->wTotalLength;
    int off = 0;
    int iface = -1;
    mp_obj_t list = mp_obj_new_list(0, NULL);
    while (off + 2 <= total) {
        uint8_t len = p[off];
        uint8_t type = p[off + 1];
        if (len < 2) {
            break;
        }
        if (type == 4 && off + 9 <= total) {                    // interface descriptor
            iface = p[off + 2];
        } else if (type == 5 && len >= 7 && off + 7 <= total) { // endpoint descriptor
            mp_obj_t t[4] = {
                MP_OBJ_NEW_SMALL_INT(iface),
                MP_OBJ_NEW_SMALL_INT(p[off + 2]),
                MP_OBJ_NEW_SMALL_INT(p[off + 3]),
                MP_OBJ_NEW_SMALL_INT(p[off + 4] | (p[off + 5] << 8)),
            };
            mp_obj_list_append(list, mp_obj_new_tuple(4, t));
        }
        off += len;
    }
    return list;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_endpoints_obj, usbhost_endpoints);

static mp_obj_t usbhost_claim(size_t n_args, const mp_obj_t *args) {
    usbhost_require_open();
    int iface = mp_obj_get_int(args[0]);
    int alt = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    s_last_err = usb_host_interface_claim(s_client_hdl, s_dev_hdl, iface, alt);
    return mp_obj_new_bool(s_last_err == ESP_OK);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbhost_claim_obj, 1, 2, usbhost_claim);

static mp_obj_t usbhost_release(mp_obj_t iface_in) {
    usbhost_require_open();
    s_last_err = usb_host_interface_release(s_client_hdl, s_dev_hdl, mp_obj_get_int(iface_in));
    return mp_obj_new_bool(s_last_err == ESP_OK);
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbhost_release_obj, usbhost_release);

static mp_obj_t usbhost_bulk_in(size_t n_args, const mp_obj_t *args) {
    usbhost_require_open();
    uint8_t ep = mp_obj_get_int(args[0]);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(args[1], &bi, MP_BUFFER_WRITE);
    int timeout_ms = (n_args > 2) ? mp_obj_get_int(args[2]) : 1000;
    usb_transfer_t *x = NULL;
    s_last_err = usb_host_transfer_alloc(USBHOST_BULK_CHUNK, 0, &x);
    if (s_last_err != ESP_OK) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    size_t got = 0;
    bool leaked = false;
    while (got < bi.len) {
        size_t rem = bi.len - got;
        size_t n = (rem + 511) & ~(size_t)511;               // IN requests must be a whole number of packets
        if (n > USBHOST_BULK_CHUNK) {
            n = USBHOST_BULK_CHUNK;
        }
        s_last_err = usbhost_run_xfer(x, ep, n, timeout_ms, &leaked);
        if (s_last_err != ESP_OK) {
            // a failed transfer can still have delivered some bytes before the error
            size_t part = (s_xfer_actual > 0) ? (size_t)s_xfer_actual : 0;
            if (part > rem) {
                part = rem;
            }
            if (part && !leaked) {
                memcpy((uint8_t *)bi.buf + got, x->data_buffer, part);
                got += part;
            }
            break;
        }
        size_t actual = (size_t)s_xfer_actual;
        size_t take = (actual > rem) ? rem : actual;
        memcpy((uint8_t *)bi.buf + got, x->data_buffer, take);
        got += take;
        if (actual < n) {
            break;                                           // short packet: the device has nothing more
        }
    }
    if (!leaked) {
        usb_host_transfer_free(x);
    }
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)got);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbhost_bulk_in_obj, 2, 3, usbhost_bulk_in);

static mp_obj_t usbhost_bulk_out(size_t n_args, const mp_obj_t *args) {
    usbhost_require_open();
    uint8_t ep = mp_obj_get_int(args[0]);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(args[1], &bi, MP_BUFFER_READ);
    int timeout_ms = (n_args > 2) ? mp_obj_get_int(args[2]) : 1000;
    usb_transfer_t *x = NULL;
    s_last_err = usb_host_transfer_alloc(USBHOST_BULK_CHUNK, 0, &x);
    if (s_last_err != ESP_OK) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    size_t sent = 0;
    bool leaked = false;
    while (sent < bi.len) {
        size_t n = bi.len - sent;
        if (n > USBHOST_BULK_CHUNK) {
            n = USBHOST_BULK_CHUNK;
        }
        memcpy(x->data_buffer, (const uint8_t *)bi.buf + sent, n);
        s_last_err = usbhost_run_xfer(x, ep, n, timeout_ms, &leaked);
        if (s_last_err != ESP_OK) {
            break;
        }
        sent += (size_t)s_xfer_actual;
    }
    if (!leaked) {
        usb_host_transfer_free(x);
    }
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)sent);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbhost_bulk_out_obj, 2, 3, usbhost_bulk_out);

static mp_obj_t usbhost_xfer_status(void) {
    return MP_OBJ_NEW_SMALL_INT(s_xfer_status);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbhost_xfer_status_obj, usbhost_xfer_status);

static const mp_rom_map_elem_t usbhost_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_usbhost) },
    { MP_ROM_QSTR(MP_QSTR_init),          MP_ROM_PTR(&usbhost_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_attach),        MP_ROM_PTR(&usbhost_attach_obj) },
    { MP_ROM_QSTR(MP_QSTR_suspend),       MP_ROM_PTR(&usbhost_suspend_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),        MP_ROM_PTR(&usbhost_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_attached),      MP_ROM_PTR(&usbhost_attached_obj) },
    { MP_ROM_QSTR(MP_QSTR_devices),       MP_ROM_PTR(&usbhost_devices_obj) },
    { MP_ROM_QSTR(MP_QSTR_last_error),    MP_ROM_PTR(&usbhost_last_error_obj) },
    { MP_ROM_QSTR(MP_QSTR_lib_status),    MP_ROM_PTR(&usbhost_lib_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_client_status), MP_ROM_PTR(&usbhost_client_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_info), MP_ROM_PTR(&usbhost_dev_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_endpoints), MP_ROM_PTR(&usbhost_endpoints_obj) },
    { MP_ROM_QSTR(MP_QSTR_claim), MP_ROM_PTR(&usbhost_claim_obj) },
    { MP_ROM_QSTR(MP_QSTR_release), MP_ROM_PTR(&usbhost_release_obj) },
    { MP_ROM_QSTR(MP_QSTR_bulk_in), MP_ROM_PTR(&usbhost_bulk_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_bulk_out), MP_ROM_PTR(&usbhost_bulk_out_obj) },
    { MP_ROM_QSTR(MP_QSTR_xfer_status), MP_ROM_PTR(&usbhost_xfer_status_obj) },
};
static MP_DEFINE_CONST_DICT(usbhost_module_globals, usbhost_module_globals_table);

const mp_obj_module_t usbhost_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&usbhost_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_usbhost, usbhost_user_cmodule);
