/*
 * PC1001 Pool Heat Pump Controller Driver
 * ESP32-C6 Manchester Protocol Implementation
 * 
 * This driver handles bidirectional Manchester-encoded communication
 * with Hayward/Majestic PC1001 heat pump controllers over a single GPIO line.
 */

#ifndef PC1001_DRIVER_H
#define PC1001_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PC1001 Operating Modes */
typedef enum {
    PC1001_MODE_COOL = 0x00,
    PC1001_MODE_HEAT = 0x08,
    PC1001_MODE_AUTO = 0x04,
    PC1001_MODE_UNKNOWN = 0xFF
} pc1001_mode_t;

/* PC1001 Status Structure */
typedef struct {
    float temp_in;          // Water inlet temperature (°C)
    float temp_out;         // Water outlet temperature (°C)
    float temp_prog;        // Programmed target temperature (°C)
    pc1001_mode_t mode;     // Operating mode
    bool power;             // Power state (on/off)
    bool valid;             // Data validity flag
} pc1001_status_t;

/* PC1001 Command Structure */
typedef struct {
    float temp;             // Target temperature (15-32°C, 0.5°C steps)
    pc1001_mode_t mode;     // Operating mode
    bool power;             // Power state
} pc1001_cmd_t;

/* Callback type for status updates */
typedef void (*pc1001_status_callback_t)(const pc1001_status_t *status);

/**
 * @brief Initialize PC1001 driver
 * 
 * @param gpio_num GPIO pin connected to PC1001 data line
 * @param callback Callback function for status updates (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t pc1001_driver_init(gpio_num_t gpio_num, pc1001_status_callback_t callback);

/**
 * @brief Deinitialize PC1001 driver
 * 
 * @return ESP_OK on success
 */
esp_err_t pc1001_driver_deinit(void);

/**
 * @brief Send command to PC1001
 * 
 * @param cmd Command structure with target settings
 * @return ESP_OK on success
 */
esp_err_t pc1001_send_command(const pc1001_cmd_t *cmd);

/**
 * @brief Get latest PC1001 status
 * 
 * @param status Pointer to status structure to fill
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no data received yet
 */
esp_err_t pc1001_get_status(pc1001_status_t *status);

/**
 * @brief Check if valid data has been received from PC1001
 * 
 * @return true if valid data available
 */
bool pc1001_has_valid_data(void);

/**
 * @brief Convert mode enum to string
 * 
 * @param mode Mode enum value
 * @return Mode string ("heat", "cool", "auto", "unknown")
 */
const char* pc1001_mode_to_string(pc1001_mode_t mode);

/**
 * @brief Convert string to mode enum
 * 
 * @param mode_str Mode string
 * @return Mode enum value
 */
pc1001_mode_t pc1001_string_to_mode(const char* mode_str);

#ifdef __cplusplus
}
#endif

#endif /* PC1001_DRIVER_H */
