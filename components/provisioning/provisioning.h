#pragma once

#include <stdbool.h>
#include <stddef.h>

/* ── NVS layout (SDD §12) ────────────────────────────────────────────────── */

#define PROV_NVS_NAMESPACE  "cooking"
#define PROV_NVS_KEY_SSID   "wifi_ssid"
#define PROV_NVS_KEY_PASS   "wifi_pass"

/* Buffer sizes: SSID ≤ 32 chars + NUL, WPA2 password ≤ 63 chars + NUL */
#define PROV_SSID_MAX_LEN   33u
#define PROV_PASS_MAX_LEN   65u

/* Wi-Fi retries per boot before clearing credentials and restarting (FR-07) */
#define PROV_WIFI_MAX_RETRIES  3u

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Ensure Wi-Fi credentials are present in NVS (FR-07).
 *
 * - First boot (no credentials): initialises BT controller + Bluedroid,
 *   starts Blufi BLE advertising as "SmartCooking", blocks until the mobile
 *   app delivers SSID + password, saves them to NVS under namespace
 *   "cooking", then tears down the full BLE stack before returning.
 * - Subsequent boots: returns immediately if credentials already exist.
 *
 * Must be called before any Wi-Fi initialisation.
 * nvs_flash_init() must have been called before this function.
 */
void provisioning_ensure(void);

/**
 * Load SSID and password from NVS into caller-supplied buffers.
 * Buffers must be at least PROV_SSID_MAX_LEN and PROV_PASS_MAX_LEN bytes.
 * Returns true on success.
 * Call after provisioning_ensure() returns.
 */
bool provisioning_load_credentials(char *ssid, size_t ssid_len,
                                   char *pass, size_t pass_len);

/**
 * Report a Wi-Fi connection failure (FR-07).
 * Erases credentials from NVS and calls esp_restart() so that the next
 * boot enters Blufi provisioning mode.
 * Call this when all PROV_WIFI_MAX_RETRIES connection attempts have failed.
 * This function does not return.
 */
void provisioning_on_wifi_failure(void);

/**
 * Reset the Wi-Fi failure state. Call on successful connection.
 * Currently a no-op — reserved for future cross-boot failure counting.
 */
void provisioning_on_wifi_success(void);
