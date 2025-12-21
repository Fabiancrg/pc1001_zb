/*
 * ESP Zigbee OTA Header
 * 
 * Handles Over-The-Air firmware updates via Zigbee network
 */

#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"

// OTA manufacturer and image type definitions
// Note: These should be defined in CMakeLists.txt and passed as compile definitions
// Fallback values only used if not defined by CMake
#ifndef OTA_UPGRADE_MANUFACTURER
#define OTA_UPGRADE_MANUFACTURER  0xFABC    // DIY manufacturer code
#endif

#ifndef OTA_UPGRADE_IMAGE_TYPE
#define OTA_UPGRADE_IMAGE_TYPE    0x1000    // Default image type
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize OTA functionality
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_zb_ota_init(void);

/**
 * @brief Register OTA callbacks
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_zb_ota_register_callbacks(void);

/**
 * @brief Get current OTA upgrade status
 * 
 * @return Current OTA status
 */
esp_zb_zcl_ota_upgrade_status_t esp_zb_ota_get_status(void);

/**
 * @brief Get current firmware version
 * 
 * @return Firmware version in format 0xMMmmpppp (major.minor.patch)
 */
uint32_t esp_zb_ota_get_fw_version(void);

/**
 * @brief OTA upgrade value callback handler
 * 
 * @param message OTA upgrade value message
 * @return ESP_OK on success
 */
esp_err_t zb_ota_upgrade_value_handler(esp_zb_zcl_ota_upgrade_value_message_t message);

/**
 * @brief OTA query image response callback handler
 * 
 * @param message OTA query image response message
 * @return ESP_OK on success
 */
esp_err_t zb_ota_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message);

#ifdef __cplusplus
}
#endif
