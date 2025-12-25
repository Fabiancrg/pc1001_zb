/*
 * PC1001 Pool Heat Pump Controller Driver Implementation
 * 
 * Manchester protocol decoder/encoder for Hayward/Majestic PC1001
 * Single GPIO bidirectional communication
 */

#include "pc1001_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <string.h>

static const char *TAG = "PC1001_DRV";

/* GPIO and timing configuration */
static gpio_num_t data_gpio = GPIO_NUM_NC;
static pc1001_status_callback_t status_callback = NULL;

/* Protocol timing (in microseconds) */
#define TIMING_START_LOW    9000    // Start bit low time
#define TIMING_START_HIGH   5000    // Start bit high time
#define TIMING_BIT0_HIGH    1000    // Binary 0 high time
#define TIMING_BIT1_HIGH    3000    // Binary 1 high time
#define TIMING_BIT_LOW      1000    // All bits low time
#define TIMING_SPACE        100000  // Space between frame repetitions (100ms)
#define TIMING_SPACE_SHORT  125000  // Short spacing (125ms)
#define TIMING_SPACE_LONG   1000000 // Long spacing (1s) after frame groups
#define TIMING_GROUP_SPACE  2000000 // Space after 8 frames

/* Tolerance for timing detection (±40%) */
#define TIMING_TOLERANCE    0.4

/* Frame structure */
#define FRAME_SIZE_LONG     12      // 12 bytes for long frames
#define FRAME_SIZE_SHORT    9       // 9 bytes for short frames
#define FRAME_SIZE          12      // Keep for compatibility
#define FRAME_REPETITIONS   8       // Repeat frame 8 times (can be 16 for full cycle)
#define RX_BUFFER_SIZE      20      // Reception buffer size

/* Frame types - Extended protocol support */
#define FRAME_TYPE_TEMP_OUT     0x4B   // Water outlet temperature (12-byte)
#define FRAME_TYPE_TEMP_IN      0x8B   // Water inlet temperature (12-byte)
#define FRAME_TYPE_STATUS       0x81   // Status: temp, mode, power (12-byte)
#define FRAME_TYPE_CLOCK        0xD1   // Clock/time sync (12-byte)
#define FRAME_TYPE_CONFIG       0x82   // Configuration (12-byte)
#define FRAME_TYPE_COND_1       0x83   // Condition packet 1 (12-byte)
#define FRAME_TYPE_COND_2       0x84   // Condition packet 2 (9-byte or 12-byte)

/* Static command frame template */
static uint8_t cmd_frame[FRAME_SIZE] = {
    0x81,   // 0 - HEADER (129)
    0x8D,   // 1 - HEADER (141)
    0x60,   // 2 - POWER & MODE (96 default mask)
    0x06,   // 3 - (6)
    0x02,   // 4 - TEMP (2 default mask)
    0x1E,   // 5 - (30)
    0xBC,   // 6 - (188)
    0xBC,   // 7 - (188)
    0xBC,   // 8 - (188)
    0xBC,   // 9 - (188)
    0x00,   // 10 - (0)
    0x00    // 11 - CHECKSUM
};

/* Receiver state machine */
typedef enum {
    RX_STATE_IDLE = 0,
    RX_STATE_RECEIVING,
    RX_STATE_COMPLETE
} rx_state_t;

/* Current status from PC1001 */
static pc1001_status_t current_status = {
    .temp_in = 0.0f,
    .temp_out = 0.0f,
    .temp_prog = 0.0f,
    .mode = PC1001_MODE_UNKNOWN,
    .power = false,
    .valid = false
};

/* Receiver state */
static volatile rx_state_t rx_state = RX_STATE_IDLE;
static volatile uint32_t high_count = 0;
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_bit_buffer = 0;
static volatile uint8_t rx_bit_count = 0;
static volatile uint8_t rx_byte_count = 0;
static volatile int64_t last_edge_time = 0;

/* Processing flag to prevent command conflicts */
static volatile bool is_processing_cmd = false;

/* Task handle */
static TaskHandle_t receiver_task_handle = NULL;

/**
 * @brief Reverse bits in a byte (MSB <-> LSB)
 */
static uint8_t reverse_bits(uint8_t byte) {
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xAA);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xCC);
    byte = ((byte >> 4) & 0x0F) | ((byte << 4) & 0xF0);
    return byte;
}

/**
 * @brief Calculate checksum for frame (supports both 9 and 12-byte frames)
 */
static uint8_t calculate_checksum(const uint8_t *frame, uint8_t size) {
    uint32_t total = 0;
    for (int i = 0; i < size - 1; i++) {
        total += reverse_bits(frame[i]);
    }
    return total % 256;
}

/**
 * @brief Validate checksum for received frame
 */
static bool validate_checksum(const uint8_t *data, uint8_t len) {
    if (len != FRAME_SIZE_SHORT && len != FRAME_SIZE_LONG) {
        return false;
    }
    uint8_t calculated = calculate_checksum(data, len);
    uint8_t received_reversed = reverse_bits(data[len - 1]);
    return (calculated == received_reversed);
}

/**
 * @brief Extract temperature from frame byte
 */
static float extract_temperature(uint8_t temp_byte) {
    uint8_t temp = reverse_bits(temp_byte);
    temp &= 0x3E;  // Mask: 0b00111110
    temp = temp >> 1;
    temp = temp + 2;
    
    bool half_degree = (temp_byte & 0x80) >> 7;
    float result = (float)temp;
    if (half_degree) {
        result += 0.5f;
    }
    return result;
}

/**
 * @brief Process received frame (supports 9 and 12-byte frames with enhanced types)
 */
static void process_received_frame(const uint8_t *frame, uint8_t size) {
    // Support both 9-byte short frames and 12-byte long frames
    if (size != FRAME_SIZE_SHORT && size != FRAME_SIZE_LONG) {
        ESP_LOGI(TAG, "Invalid frame length: %d bytes (expected 9 or 12)", size);
        return;
    }
    
    if (!validate_checksum(frame, size)) {
        ESP_LOGI(TAG, "Invalid checksum for frame type 0x%02X", frame[0]);
        return;
    }
    
    uint8_t frame_type = frame[0];
    ESP_LOGI(TAG, "Valid %d-byte frame type 0x%02X", size, frame_type);
    
    switch (frame_type) {
        case 0x4B:  // TEMP OUT frame (0b01001011) - FRAME_TYPE_TEMP_OUT
            if (size >= FRAME_SIZE_LONG) {
                current_status.temp_out = extract_temperature(frame[4]);
                ESP_LOGI(TAG, "Temp OUT: %.1f°C", current_status.temp_out);
            }
            break;
            
        case 0x8B:  // TEMP IN frame (0b10001011) - FRAME_TYPE_TEMP_IN
            if (size >= FRAME_SIZE_SHORT && size >= 10) {
                current_status.temp_in = extract_temperature(frame[9]);
                ESP_LOGI(TAG, "Temp IN: %.1f°C", current_status.temp_in);
            }
            break;
            
        case 0x81:  // STATUS frame (0b10000001) - FRAME_TYPE_STATUS
            if (size >= FRAME_SIZE_LONG) {
                current_status.temp_prog = extract_temperature(frame[4]);
                current_status.power = (frame[2] & 0x80) >> 7;
                
                // Decode mode
                bool auto_mode = (frame[2] & 0x04) >> 2;
                if (auto_mode) {
                    current_status.mode = PC1001_MODE_AUTO;
                } else {
                    bool heat = (frame[2] & 0x08) >> 3;
                    current_status.mode = heat ? PC1001_MODE_HEAT : PC1001_MODE_COOL;
                }
                
                current_status.valid = true;
                
                ESP_LOGI(TAG, "Status: %.1f°C, %s, %s", 
                         current_status.temp_prog,
                         current_status.power ? "ON" : "OFF",
                         pc1001_mode_to_string(current_status.mode));
                
                // Notify callback
                if (status_callback && current_status.temp_out > 0) {
                    status_callback(&current_status);
                }
            }
            break;
            
        case 0xD1:  // CLOCK frame - FRAME_TYPE_CLOCK
            if (size >= FRAME_SIZE_LONG) {
                ESP_LOGD(TAG, "Clock/time sync frame received (not fully implemented)");
                // Clock data typically in bytes 1-6: hour, minute, second, day, month, year
                // Could be implemented in future for time synchronization
            }
            break;
            
        case 0x82:  // CONFIG frame - FRAME_TYPE_CONFIG
            if (size >= FRAME_SIZE_LONG) {
                ESP_LOGD(TAG, "Configuration frame received (not fully implemented)");
                // Configuration parameters for advanced heat pump settings
            }
            break;
            
        case 0x83:  // COND_1 frame - FRAME_TYPE_COND_1
            if (size >= FRAME_SIZE_LONG) {
                ESP_LOGD(TAG, "Condition 1 frame received (not fully implemented)");
                // Additional sensor data and heat pump conditions
            }
            break;
            
        case 0x84:  // COND_2 / COND_2B frame - FRAME_TYPE_COND_2
            if (size == FRAME_SIZE_SHORT) {
                ESP_LOGD(TAG, "Condition 2B short frame received (not fully implemented)");
                // Short 9-byte condition frame (flow meter, defrost, etc.)
            } else if (size == FRAME_SIZE_LONG) {
                ESP_LOGD(TAG, "Condition 2 long frame received (not fully implemented)");
                // Long 12-byte condition frame with extended diagnostics
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Unknown frame type: 0x%02X (len=%d)", frame_type, size);
            break;
    }
}

/**
 * @brief Receiver task - processes timing-based protocol decoding
 */
static void receiver_task(void *arg) {
    ESP_LOGI(TAG, "Receiver task started");
    
    int last_level = 1;
    int64_t last_time = esp_timer_get_time();
    uint32_t loop_counter = 0;
    
    // Activity monitoring
    uint32_t edge_count = 0;
    int64_t last_activity_report = esp_timer_get_time();
    const int64_t ACTIVITY_REPORT_INTERVAL = 10000000; // 10 seconds
    
    while (1) {
        ets_delay_us(200);  // 200µs polling to match Arduino timing
        
        // Feed watchdog every ~1ms (5 iterations x 200µs)
        // Use pdMS_TO_TICKS(1) to ensure proper yield
        loop_counter++;
        if (loop_counter >= 5) {
            vTaskDelay(pdMS_TO_TICKS(1));  // Yield for 1ms to reset watchdog
            loop_counter = 0;
        }
        
        int level = gpio_get_level(data_gpio);
        int64_t now = esp_timer_get_time();
        int64_t duration = now - last_time;
        
        if (level != last_level) {
            // Edge detected
            edge_count++;
            
            if (last_level == 1 && level == 0) {
                // Falling edge - end of HIGH pulse
                int64_t duration_us = duration;
                int64_t duration_ms = duration_us / 1000;
                
                // Log pulses that might be protocol-related (>500us)
                if (duration_us >= 500) {
                    ESP_LOGI(TAG, "Falling edge: HIGH was %lld us (%.1f ms)", 
                             duration_us, (float)duration_us / 1000.0f);
                }
                
                if (duration_ms >= 22 && duration_ms <= 28) {
                    // Start of frame (5ms ± tolerance)
                    rx_state = RX_STATE_RECEIVING;
                    rx_bit_count = 0;
                    rx_bit_buffer = 0;
                    rx_byte_count = 0;
                    ESP_LOGI(TAG, "Frame start detected (pulse: %lld ms)", duration_ms);
                    
                } else if (rx_state == RX_STATE_RECEIVING) {
                    if (duration_ms >= 12 && duration_ms <= 18) {
                        // Binary 1 (3ms)
                        rx_bit_buffer = rx_bit_buffer << 1;
                        rx_bit_buffer |= 1;
                        rx_bit_count++;
                    } else if (duration_ms >= 2 && duration_ms <= 8) {
                        // Binary 0 (1ms)
                        rx_bit_buffer = rx_bit_buffer << 1;
                        rx_bit_count++;
                    }
                    
                    if (rx_bit_count == 8) {
                        // Complete byte received
                        if (rx_byte_count < RX_BUFFER_SIZE) {
                            rx_buffer[rx_byte_count++] = rx_bit_buffer;
                        }
                        rx_bit_count = 0;
                        rx_bit_buffer = 0;
                    }
                }
            } else if (last_level == 0 && level == 1) {
                // Rising edge - end of LOW pulse
                int64_t duration_us = duration;
                
                // Log LOW pulses that might be protocol-related
                if (duration_us >= 500) {
                    ESP_LOGI(TAG, "Rising edge: LOW was %lld us (%.1f ms)", 
                             duration_us, (float)duration_us / 1000.0f);
                }
                
                // Check for end of frame
                if (rx_state == RX_STATE_RECEIVING && duration > 50000) {
                    // Long LOW > 50ms = end of frame
                    if (rx_byte_count > 0) {
                        uint8_t frame_copy[RX_BUFFER_SIZE];
                        memcpy(frame_copy, (void*)rx_buffer, rx_byte_count);
                        process_received_frame(frame_copy, rx_byte_count);
                    }
                    rx_state = RX_STATE_IDLE;
                }
            }
            
            last_time = now;
            last_level = level;
        }
        
        // Periodic activity report every 10 seconds
        if (now - last_activity_report >= ACTIVITY_REPORT_INTERVAL) {
            int current_gpio_level = gpio_get_level(data_gpio);
            
            ESP_LOGI(TAG, "=== GPIO Activity Report ===");
            ESP_LOGI(TAG, "  Current GPIO level: %d", current_gpio_level);
            ESP_LOGI(TAG, "  Total edges in last 10s: %lu", edge_count);
            ESP_LOGI(TAG, "  RX state: %s", 
                     rx_state == RX_STATE_IDLE ? "IDLE" : 
                     rx_state == RX_STATE_RECEIVING ? "RECEIVING" : "COMPLETE");
            ESP_LOGI(TAG, "  Valid status: %s", current_status.valid ? "YES" : "NO");
            
            if (edge_count == 0) {
                ESP_LOGW(TAG, "  WARNING: No activity detected - check:");
                ESP_LOGW(TAG, "    1. Heat pump is powered ON");
                ESP_LOGW(TAG, "    2. Wiring: GPIO2 <-> BSS138 <-> PC1001");
                ESP_LOGW(TAG, "    3. Common ground between ESP and PC1001");
            } else if (edge_count < 100 && !current_status.valid) {
                ESP_LOGW(TAG, "  Low activity but no valid frames - check protocol");
            }
            ESP_LOGI(TAG, "===========================");
            
            edge_count = 0;
            last_activity_report = now;
        }
    }
}

/**
 * @brief Send LOW signal
 */
static void send_low(uint32_t us) {
    gpio_set_level(data_gpio, 0);
    esp_rom_delay_us(us);
}

/**
 * @brief Send HIGH signal
 */
static void send_high(uint32_t us) {
    gpio_set_level(data_gpio, 1);
    esp_rom_delay_us(us);
}

/**
 * @brief Send binary 0
 */
static void send_bit_0(void) {
    send_low(TIMING_BIT_LOW);
    send_high(TIMING_BIT0_HIGH);
}

/**
 * @brief Send binary 1
 */
static void send_bit_1(void) {
    send_low(TIMING_BIT_LOW);
    send_high(TIMING_BIT1_HIGH);
}

/**
 * @brief Send frame header
 */
static void send_header(void) {
    send_low(TIMING_START_LOW);
    send_high(TIMING_START_HIGH);
}

/**
 * @brief Send space between frames (matches Arduino: 100ms between frames)
 */
static void send_frame_space(uint8_t cycle_pos) {
    send_low(TIMING_BIT_LOW);
    // Match Arduino: always 100ms between frame repetitions
    send_high(TIMING_SPACE);
}

/**
 * @brief Send space after frame group
 */
static void send_group_space(void) {
    send_low(TIMING_BIT_LOW);
    // Split into chunks to feed watchdog
    for (int i = 0; i < 4; i++) {
        send_high(500000);
        vTaskDelay(1);
    }
}

/**
 * @brief Transmit command frame with enhanced 16-frame cycle pattern
 */
static void transmit_command_frame(void) {
    // Switch to output mode
    gpio_set_direction(data_gpio, GPIO_MODE_OUTPUT);
    
    // Repeat frame 16 times (full protocol cycle) or 8 times for faster response
    // Using 8 for backward compatibility and faster command execution
    const int repetitions = 8;  // Can be changed to 16 for full cycle
    
    for (int rep = 0; rep < repetitions; rep++) {
        send_header();
        
        // Send all bytes (12-byte long frame)
        for (int byte_idx = 0; byte_idx < FRAME_SIZE_LONG; byte_idx++) {
            uint8_t byte = cmd_frame[byte_idx];
            
            // Send MSB first
            for (int bit = 7; bit >= 0; bit--) {
                if (byte & (1 << bit)) {
                    send_bit_1();
                } else {
                    send_bit_0();
                }
            }
        }
        
        // Space between repetitions (variable spacing based on cycle)
        if (rep < repetitions - 1) {
            send_frame_space(rep);
        } else {
            send_group_space();
        }
        
        vTaskDelay(1);  // Feed watchdog
    }
    
    // Back to input mode
    gpio_set_direction(data_gpio, GPIO_MODE_INPUT);
    
    ESP_LOGI(TAG, "Command frame transmitted (%d repetitions)", repetitions);
}

/* Public API Implementation */

esp_err_t pc1001_driver_init(gpio_num_t gpio_num, pc1001_status_callback_t callback) {
    if (gpio_num == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Invalid GPIO number");
        return ESP_ERR_INVALID_ARG;
    }
    
    data_gpio = gpio_num;
    status_callback = callback;
    
    // Configure GPIO as input without internal pull resistors
    // Hardware has external 10K pull-ups on both sides of BSS138 level shifter
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << data_gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read initial GPIO level for diagnostics
    int initial_level = gpio_get_level(data_gpio);
    ESP_LOGI(TAG, "GPIO%d initial level: %d (expect 1/HIGH when idle)", data_gpio, initial_level);
    
    // Create receiver task
    BaseType_t task_ret = xTaskCreate(receiver_task, "pc1001_rx", 4096, NULL, 5, &receiver_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receiver task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "PC1001 driver initialized on GPIO%d - listening for frames", data_gpio);
    ESP_LOGI(TAG, "Waiting for heat pump status frames (type 0x81)...");
    return ESP_OK;
}

esp_err_t pc1001_driver_deinit(void) {
    if (receiver_task_handle) {
        vTaskDelete(receiver_task_handle);
        receiver_task_handle = NULL;
    }
    return ESP_OK;
}

esp_err_t pc1001_send_command(const pc1001_cmd_t *cmd) {
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (cmd->temp < 15.0f || cmd->temp > 33.0f) {
        ESP_LOGE(TAG, "Temperature out of range: %.1f (must be 15-33°C)", cmd->temp);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!current_status.valid) {
        ESP_LOGW(TAG, "No valid status received yet, cannot send command");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (is_processing_cmd) {
        ESP_LOGW(TAG, "Command already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    is_processing_cmd = true;
    
    // Reset frame to defaults
    cmd_frame[2] = 0x60;  // Power/mode mask
    cmd_frame[4] = 0x02;  // Temp mask
    
    // Set power bit
    if (cmd->power) {
        cmd_frame[2] |= 0x80;
    }
    
    // Set mode bits
    cmd_frame[2] |= (uint8_t)cmd->mode;
    
    // Set temperature
    uint8_t temp_int = (uint8_t)cmd->temp;
    bool half_degree = ((int)(cmd->temp * 10) - (temp_int * 10)) > 0;
    
    uint8_t temp_value = temp_int - 2;
    temp_value = reverse_bits(temp_value);
    temp_value = temp_value >> 1;
    cmd_frame[4] |= temp_value;
    
    if (half_degree) {
        cmd_frame[4] |= 0x80;
    }
    
    // Generate checksum
    uint8_t checksum = calculate_checksum(cmd_frame, FRAME_SIZE_LONG);
    cmd_frame[11] = reverse_bits(checksum);
    
    ESP_LOGI(TAG, "Sending command: %.1f°C, %s, %s", 
             cmd->temp, 
             cmd->power ? "ON" : "OFF",
             pc1001_mode_to_string(cmd->mode));
    
    // Transmit
    transmit_command_frame();
    
    // Reset receiver state
    rx_state = RX_STATE_IDLE;
    rx_byte_count = 0;
    
    is_processing_cmd = false;
    
    return ESP_OK;
}

esp_err_t pc1001_get_status(pc1001_status_t *status) {
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!current_status.valid) {
        return ESP_ERR_NOT_FOUND;
    }
    
    memcpy(status, &current_status, sizeof(pc1001_status_t));
    return ESP_OK;
}

bool pc1001_has_valid_data(void) {
    return current_status.valid;
}

const char* pc1001_mode_to_string(pc1001_mode_t mode) {
    switch (mode) {
        case PC1001_MODE_HEAT: return "heat";
        case PC1001_MODE_COOL: return "cool";
        case PC1001_MODE_AUTO: return "auto";
        default: return "unknown";
    }
}

pc1001_mode_t pc1001_string_to_mode(const char* mode_str) {
    if (strcmp(mode_str, "heat") == 0) return PC1001_MODE_HEAT;
    if (strcmp(mode_str, "cool") == 0) return PC1001_MODE_COOL;
    if (strcmp(mode_str, "auto") == 0) return PC1001_MODE_AUTO;
    return PC1001_MODE_UNKNOWN;
}
