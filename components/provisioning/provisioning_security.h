#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── Blufi security helpers ──────────────────────────────────────────────────
 *
 * Implements the DH key-exchange + AES-256-CTR security layer for Blufi
 * provisioning.  Based on the ESP-IDF blufi example (examples/bluetooth/blufi).
 *
 * Call blufi_security_init() before esp_blufi_register_callbacks() and
 * blufi_security_deinit() after esp_blufi_profile_deinit().
 *
 * Wire the four functions below into esp_blufi_callbacks_t:
 *   .negotiate_data_handler = blufi_dh_negotiate_data_handler
 *   .encrypt_func           = blufi_aes_encrypt
 *   .decrypt_func           = blufi_aes_decrypt
 *   .checksum_func          = blufi_crc_checksum
 */

/**
 * Allocate and initialise the Blufi security context (DHM + AES).
 * Returns ESP_OK on success, ESP_FAIL if heap allocation fails.
 */
esp_err_t blufi_security_init(void);

/** Release the security context and clear key material. */
void blufi_security_deinit(void);

/**
 * negotiate_data_handler — performs the DH key exchange (PSA FFDH RFC7919,
 * 3072-bit) and derives an AES-256-CTR session key via SHA-256.
 */
void blufi_dh_negotiate_data_handler(uint8_t *data, int len,
                                     uint8_t **output_data, int *output_len,
                                     bool *need_free);

/** encrypt_func — AES-256-CTR in-place encryption. */
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);

/** decrypt_func — AES-256-CTR in-place decryption. */
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);

/** checksum_func — CRC-16/CCITT-BE checksum. */
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);

/* ── DH keypair pre-generation (SOCs without hardware MPI only) ─────────────
 * On ESP32 SOC_MPI_SUPPORTED=1 so these compile out.
 * Call blufi_dh_pregen_start() just after blufi_security_init() to overlap
 * the expensive G^X mod P computation with BLE advertising time.
 */
#include "soc/soc_caps.h"
#if !SOC_MPI_SUPPORTED
void blufi_dh_pregen_start(void);
void blufi_dh_pregen_start_with_cb(void (*done_cb)(void));
void blufi_dh_pregen_wait(void);
#endif
