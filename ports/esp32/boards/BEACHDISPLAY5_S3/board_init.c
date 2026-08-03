/*
 * BEACHDISPLAY5_S3 board startup hook.
 *
 * Runs once, on every boot (power-on, soft reset, watchdog reset,
 * deep-sleep wake) via MICROPY_BOARD_STARTUP, BEFORE mp_task/boot.py
 * ever run. Job: get the BG95-M3 modem's USB link into genuine suspend by
 * having the ESP32-S3's own native USB (IO19/IO20) act as its host --
 * Quectel confirmed there is no other way to reach USB suspend, and
 * bench testing (beachdisplay5 memory/project_modem_design.md,
 * 2026-08-02) confirmed this approach reaches ~5mA (datasheet Sleep
 * tier), vs ~24-44mA otherwise.
 *
 * Never power-cycles the modem itself -- only ensures its power rail is
 * on (idempotent: harmless if already on from a prior boot, since the
 * FXL6408 GPIO expander's output latches persist independently of ESP32
 * resets). Bounded timeouts throughout: if the modem is missing,
 * unresponsive, or the USB enumeration fails, this gives up and lets
 * boot.py/main.py start anyway rather than blocking forever.
 *
 * IMPORTANT: boot.py must NOT unconditionally reset the fxl_main
 * (0x44) GPIO expander on every boot the way it currently does --
 * that undoes the power/DTR state this hook establishes. See the
 * beachdisplay5 boot.py fix that must ship alongside this.
 *
 * Full usb_host_uninstall() is deliberately skipped -- confirmed on
 * hardware (same memory doc) that the finer-grained close+deregister
 * sequence does not disturb an established suspend, while a clean
 * uninstall proved unreliable. The idle usb_host task's fixed RAM cost
 * is an accepted, measured tradeoff, not an oversight -- see the
 * heap-measurement note at the end of this file.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "nvs_flash.h"
#include "nvs.h"

// --- External hardware watchdog (TPS3435) keepalive ------------------------
// board_v4's TPS3435AFACADDFRQ1 needs a WDI feed pulse within ~10s of
// power-up just to exit its startup-delay window (tSD) at all -- miss that
// deadline and it resets, restarting boot and missing the deadline again
// (a ~10s reboot loop). Normally lib/boot_wdt.py handles this, started as
// the very first thing in boot.py -- but this C hook now runs BEFORE
// boot.py, and can legitimately take up to OVERALL_TIMEOUT_MS (28s) if the
// modem is slow to attach, which blows straight through that 10s deadline
// with nothing feeding the watchdog at all. Confirmed as a real risk
// 2026-08-02, caught before it caused a field problem -- not yet actually
// reproduced as a reset on this hardware, but the timing math is
// unambiguous, so this is fixed defensively rather than waiting to see it
// happen. Mirrors boot_wdt.py's pin/timing exactly (GPIO12, ~20ms pulse,
// ~1000ms period) so the later Python-level handoff sees a consistent,
// already-warm watchdog rather than conflicting settings.
#define WDT_FEED_PIN         GPIO_NUM_12
#define WDT_FEED_PULSE_MS    20
#define WDT_FEED_PERIOD_MS   1000

static volatile bool s_wdt_feed_stop = false;
static TaskHandle_t s_wdt_feed_task_hdl = NULL;

static void wdt_feed_task(void *arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << WDT_FEED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // idle-low bias, matches boot_wdt.py
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(WDT_FEED_PIN, 0);

    while (!s_wdt_feed_stop) {
        gpio_set_level(WDT_FEED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_PULSE_MS));
        gpio_set_level(WDT_FEED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_PERIOD_MS - WDT_FEED_PULSE_MS));
    }
    // Leave the pin idle-low for a clean handoff to boot_wdt.py's own
    // Pin() claim right after this hook returns.
    gpio_set_level(WDT_FEED_PIN, 0);
    s_wdt_feed_task_hdl = NULL;
    vTaskDelete(NULL);
}

static void wdt_feed_start(void) {
    s_wdt_feed_stop = false;
    xTaskCreatePinnedToCore(wdt_feed_task, "wdt_feed", 2048, NULL, 6, &s_wdt_feed_task_hdl, 0);
}

static void wdt_feed_stop_and_wait(void) {
    if (s_wdt_feed_task_hdl == NULL) {
        return;
    }
    s_wdt_feed_stop = true;
    // Give the task up to one full period to notice and exit cleanly.
    for (int i = 0; i < (WDT_FEED_PERIOD_MS / 50) + 2 && s_wdt_feed_task_hdl != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Mirrors what boot.py writes from config.json's hardwareconfig.modem_usb_host_cable
// (self-healing, rewritten every boot -- see boot.py's own comment on this).
// Namespace/key must match exactly. Absent key (e.g. very first boot on this
// firmware, before boot.py has ever run) defaults to "try anyway" -- costs
// one bounded delay at most, never repeats after boot.py writes the real value.
#define MODEM_CABLE_NVS_NAMESPACE  "bd5"
#define MODEM_CABLE_NVS_KEY        "usbcbl"

// board_v4's config jumper (GPIO13, active-low, pull-up) already tells
// main.py to launch webserver.py instead of normal operation -- see
// lib/early_boot.py's read_config_jumper(). Reusing the same physical
// jumper here for the same underlying purpose (fast, interactive
// provisioning access) rather than inventing a second signal: when
// asserted, skip the whole modem-suspend sequence entirely so native USB
// (IO19/IO20) is never claimed by usb_host, keeping mpremote/esptool
// access over native USB available immediately instead of waiting out
// (and permanently losing, since usb_host_uninstall() is deliberately
// never called) the modem-attach sequence on every boot.
#define CONFIG_JUMPER_PIN   GPIO_NUM_13

static bool config_jumper_asserted(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CONFIG_JUMPER_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    return gpio_get_level(CONFIG_JUMPER_PIN) == 0;
}

static bool modem_cable_present(void) {
    nvs_handle_t handle;
    if (nvs_open(MODEM_CABLE_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return true;  // no NVS entry yet -- try anyway (see comment above)
    }
    // MUST be i32, not u8: MicroPython's esp32.NVS class (boot.py, the
    // only writer of this key) only supports 32-bit signed ints and
    // blobs -- no u8 variant exists at all. A u8 read against a value
    // boot.py wrote as i32 hits ESP_ERR_NVS_TYPE_MISMATCH every time
    // (confirmed on hardware 2026-08-02, boot.py's own get_i32 failed
    // with AttributeError before this was even caught from the C side).
    int32_t val = 1;
    esp_err_t err = nvs_get_i32(handle, MODEM_CABLE_NVS_KEY, &val);
    nvs_close(handle);
    if (err != ESP_OK) {
        return true;  // key not set yet -- try anyway
    }
    return val != 0;
}

// From main.c -- the default MICROPY_BOARD_STARTUP implementation
// (NVS init, flash-size detection, vfs partition setup). We still need
// all of that; we're adding to it, not replacing it.
extern void boardctrl_startup(void);

static const char *TAG = "bd5_modem_usb";

// --- fxl_main (FXL6408 GPIO expander, I2C1 @ 0x44) -- modem power/reset/DTR ---
#define FXL_MAIN_I2C_PORT   I2C_NUM_1
#define FXL_MAIN_SDA        18
#define FXL_MAIN_SCL        21
#define FXL_MAIN_ADDR       0x44
#define FXL_MAIN_FREQ_HZ    100000

// Register offsets (FXL6408)
#define FXL_REG_DIR         0x03  // 1 = output
#define FXL_REG_OUT         0x05  // output value
#define FXL_REG_HIZ         0x07  // 0 = driven, 1 = Hi-Z

// Bit assignments (see beachdisplay5 board_v4.py / lib/drivers/fxl6408.py)
#define FXL_BIT_EN_5V       (1 << 2)
#define FXL_BIT_DTR         (1 << 3)
#define FXL_BIT_MODEM_RESET (1 << 6)  // active-low
#define FXL_BIT_MODEM_PWR   (1 << 7)  // active-low

// Modem attach timeout: how long we'll wait for it to enumerate before
// giving up and letting boot continue without a suspended modem.
// Bumped again 2026-08-02 (20000ms -> 30000ms): even 20s still timed out
// intermittently on hardware with the cable genuinely connected -- actual
// attach has ranged from ~400ms to a full timeout across repeated boots
// this session, wider variance than originally assumed. The watchdog feed
// task runs continuously for the whole duration regardless (see
// wdt_feed_task above), so a longer window here doesn't reintroduce the
// TPS3435 tSD risk this was already designed around.
#define MODEM_ATTACH_TIMEOUT_MS   30000
// Outer safety-net timeout on the whole sequence (task creation +
// power-on settle + attach wait + suspend). Must be comfortably larger
// than MODEM_POWERON_SETTLE_MS + MODEM_ATTACH_TIMEOUT_MS.
#define OVERALL_TIMEOUT_MS         40000
// Modem needs a moment after power-on before it presents on USB, same
// order of magnitude as the boot_wait_s used in tests/modem_at.py.
// Bumped 3000ms -> 5000ms alongside the attach-timeout increase above.
#define MODEM_POWERON_SETTLE_MS    5000

static i2c_master_bus_handle_t s_fxl_bus;
static i2c_master_dev_handle_t s_fxl_dev;

static bool fxl_main_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_fxl_dev, buf, sizeof(buf), 100) == ESP_OK;
}

// Ensures modem power is on and DTR/reset are in the correct idle state.
// Idempotent -- if the expander already has these bits set from a prior
// (non-power-cycled) boot, these writes are harmless no-ops in effect.
// Returns true if the I2C bus/device came up and writes succeeded.
static bool modem_power_ensure_on(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = FXL_MAIN_I2C_PORT,
        .sda_io_num = FXL_MAIN_SDA,
        .scl_io_num = FXL_MAIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_fxl_bus) != ESP_OK) {
        ESP_LOGW(TAG, "fxl_main: I2C bus init failed");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = FXL_MAIN_ADDR,
        .scl_speed_hz = FXL_MAIN_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(s_fxl_bus, &dev_cfg, &s_fxl_dev) != ESP_OK) {
        ESP_LOGW(TAG, "fxl_main: device add failed");
        return false;
    }

    // Direction: bits 2,3,6,7 as outputs (0xCC), rest inputs.
    bool ok = fxl_main_write_reg(FXL_REG_DIR, 0xCC);
    ok &= fxl_main_write_reg(FXL_REG_HIZ, 0x00);
    // #MODEM_RESET=1 (inactive), DTR=0 (wake), #MODEM_PWR=0 (on).
    //
    // DTR polarity matches modem_transport.py's QSCLK convention exactly
    // (see _FXL_MAIN_MODEM_DTR there): HIGH=sleep, LOW=wake. The FXL6408
    // is an I2C GPIO expander with its own power rail, independent of the
    // ESP32 -- it is NOT reset by an ESP32-only reset (watchdog, crash,
    // machine.reset()), only by a real power cycle. So on anything short
    // of a full power cycle, the modem inherits whatever DTR level the
    // *previous* boot left it in. If the previous session put the modem
    // into QSCLK sleep (normal end-of-cycle behavior, not an edge case),
    // DTR is already HIGH. Writing DTR=1 here (as this used to do, calling
    // it "released") was a no-op that left the modem asleep, and a
    // sleeping modem may not enumerate on USB -- this was the actual
    // cause of the intermittent 400ms-30s USB-attach variance, not a
    // connection problem (confirmed 2026-08-03). Driving DTR low
    // explicitly wakes it, same as _psm_wake()/QSCLK wake on the Python
    // side. Deliberately NOT touching EN_5V -- not required for modem
    // power (see project_modem_design.md "Modem_3V3 rail -- RESOLVED").
    uint8_t out_val = FXL_BIT_MODEM_RESET;  // DTR bit left 0 (wake), MODEM_PWR bit left 0 (on)
    ok &= fxl_main_write_reg(FXL_REG_OUT, out_val);

    if (!ok) {
        ESP_LOGW(TAG, "fxl_main: register writes failed");
    }
    return ok;
}

// --- USB host: enumerate the modem and suspend it ---

static usb_host_client_handle_t s_client_hdl;
static usb_device_handle_t s_dev_hdl = NULL;
static uint8_t s_connected_addr = 0;
static volatile bool s_device_connected = false;
static SemaphoreHandle_t s_done_sem;

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        s_connected_addr = event_msg->new_dev.address;
        s_device_connected = true;
    }
}

static void usb_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // No action needed on NO_CLIENTS/ALL_FREE -- we intentionally
        // leave the library installed (see file header).
    }
}

static void modem_suspend_task(void *arg) {
    size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    if (!modem_power_ensure_on()) {
        ESP_LOGW(TAG, "modem power-on sequence failed, skipping USB suspend");
        goto done;
    }
    ESP_LOGI(TAG, "modem power-on OK, settling %dms before USB host bring-up",
             MODEM_POWERON_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(MODEM_POWERON_SETTLE_MS));

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    if (usb_host_install(&host_config) != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_install failed, skipping USB suspend");
        goto done;
    }

    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 5, NULL, 0);

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    if (usb_host_client_register(&client_config, &s_client_hdl) != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_client_register failed, skipping USB suspend");
        goto done;
    }

    ESP_LOGI(TAG, "usb_host up, waiting up to %dms for modem to attach",
             MODEM_ATTACH_TIMEOUT_MS);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MODEM_ATTACH_TIMEOUT_MS);
    while (!s_device_connected && xTaskGetTickCount() < deadline) {
        usb_host_client_handle_events(s_client_hdl, pdMS_TO_TICKS(500));
    }

    if (!s_device_connected) {
        ESP_LOGW(TAG, "modem did not attach within %dms, continuing boot without suspend",
                 MODEM_ATTACH_TIMEOUT_MS);
        goto done;
    }

    if (usb_host_device_open(s_client_hdl, s_connected_addr, &s_dev_hdl) != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_device_open failed");
        goto done;
    }

    esp_err_t suspend_err = usb_host_lib_root_port_suspend();
    if (suspend_err == ESP_OK) {
        ESP_LOGI(TAG, "modem USB suspend requested OK");
    } else {
        ESP_LOGW(TAG, "usb_host_lib_root_port_suspend failed: %s", esp_err_to_name(suspend_err));
    }

    // Give the suspend transition a moment, then release our own client
    // handle. Confirmed on hardware: close + deregister does not disturb
    // an established suspend. Deliberately skip usb_host_uninstall() --
    // see file header.
    vTaskDelay(pdMS_TO_TICKS(300));
    usb_host_device_close(s_client_hdl, s_dev_hdl);
    usb_host_client_deregister(s_client_hdl);

done:
    {
        size_t heap_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "modem USB-suspend startup done, internal heap cost: %d bytes",
                 (int)(heap_before - heap_after));
    }
    xSemaphoreGive(s_done_sem);
    vTaskDelete(NULL);
}

void BEACHDISPLAY5_S3_board_startup(void) {
    // Feed the external TPS3435 watchdog from the very first instant,
    // before anything else in this function -- see the wdt_feed_task
    // comment above. Everything below this line can legitimately take up
    // to OVERALL_TIMEOUT_MS; boot_wdt.py won't take over feeding until
    // boot.py runs, well after this function returns.
    wdt_feed_start();

    // This board's default runtime log level is ERROR-only
    // (CONFIG_LOG_DEFAULT_LEVEL_ERROR), which would silently swallow every
    // INFO/WARN line in this file -- including the ESP_LOGW warnings that
    // matter for field diagnostics, not just this file's own INFO-level
    // progress logging. Raise it for just this tag rather than changing
    // the system-wide default (confirmed on hardware 2026-08-02: without
    // this, none of board_init.c's output was visible on the debug UART
    // at all, even though the strings were compiled into the binary).
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Preserve default startup behaviour (NVS init, flash size, vfs
    // partition setup) -- this is the same call every other board's
    // MICROPY_BOARD_STARTUP override makes. Must run first: it's what
    // brings NVS up, which modem_cable_present() below depends on.
    boardctrl_startup();

    if (config_jumper_asserted()) {
        ESP_LOGI(TAG, "config jumper asserted, skipping modem USB-suspend startup "
                      "entirely -- native USB stays free for provisioning");
        wdt_feed_stop_and_wait();
        return;
    }

    if (!modem_cable_present()) {
        ESP_LOGI(TAG, "modem_usb_host_cable not set for this unit (NVS), "
                      "skipping modem USB-suspend startup entirely");
        wdt_feed_stop_and_wait();
        return;
    }

    s_done_sem = xSemaphoreCreateBinary();
    if (s_done_sem == NULL) {
        ESP_LOGE(TAG, "semaphore create failed, skipping modem USB-suspend startup");
        wdt_feed_stop_and_wait();
        return;
    }

    xTaskCreatePinnedToCore(modem_suspend_task, "modem_suspend", 4096, NULL, 5, NULL, 0);

    // Outer safety net: if the task somehow hangs despite its own
    // internal timeouts, boot must still proceed.
    if (xSemaphoreTake(s_done_sem, pdMS_TO_TICKS(OVERALL_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "modem USB-suspend startup task did not complete within %dms, "
                      "continuing boot anyway", OVERALL_TIMEOUT_MS);
    }

    // Hand off watchdog feeding to boot_wdt.py, which runs immediately
    // after this function returns.
    wdt_feed_stop_and_wait();
}
