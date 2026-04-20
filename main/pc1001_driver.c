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

/* TX protocol timing (microseconds) */
#define TIMING_START_LOW    9000    // Start bit low
#define TIMING_START_HIGH   5000    // Start bit high
#define TIMING_BIT0_HIGH    1000    // Binary 0 high
#define TIMING_BIT1_HIGH    3000    // Binary 1 high
#define TIMING_BIT_LOW      1000    // All bits low
#define TIMING_SPACE        100000  // Space between frame repetitions (100ms)

/* RX classification windows (microseconds).
 * The protocol is asymmetric: controller-to-pump encodes bit-1 as a long
 * (~3ms) HIGH pulse and bit-0 as a short (~1ms) HIGH. The heat pump's
 * broadcasts use the OPPOSITE mapping - a long HIGH means bit 0 and a
 * short HIGH means bit 1. The windows below describe HIGH pulse durations;
 * the bit value they decode to is assigned in receiver_task.
 * Wide windows absorb scheduling jitter but stay mutually exclusive. */
#define RX_START_MIN_US     4000     // 5ms header HIGH
#define RX_START_MAX_US     6000
#define RX_LONG_MIN_US      2200     // ~3ms HIGH (bit 0 from pump)
#define RX_LONG_MAX_US      3800
#define RX_SHORT_MIN_US      400     // ~1ms HIGH (bit 1 from pump)
#define RX_SHORT_MAX_US     1600
#define RX_EOF_MIN_US      50000     // HIGH >50ms between frames marks end of frame

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

/* Receiver state (owned by receiver_task) */
static rx_state_t rx_state = RX_STATE_IDLE;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_bit_buffer = 0;
static uint8_t rx_bit_count = 0;
static uint8_t rx_byte_count = 0;

/* Edge events pushed from ISR to receiver_task */
typedef struct {
    int64_t timestamp;
    uint8_t level;      // level AFTER the edge
} edge_event_t;

#define EDGE_QUEUE_DEPTH    256
static QueueHandle_t edge_queue = NULL;

/* ISR enable flag - disabled during TX so we don't self-capture */
static volatile bool rx_isr_enabled = false;

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
 * @brief GPIO edge ISR - timestamps edges and queues them for the decoder task.
 * Dropped silently if the queue is full or the ISR is gated during TX.
 */
static void IRAM_ATTR data_gpio_isr(void *arg) {
    if (!rx_isr_enabled) {
        return;
    }
    edge_event_t evt = {
        .timestamp = esp_timer_get_time(),
        .level = (uint8_t)gpio_get_level(data_gpio),
    };
    BaseType_t hp_woken = pdFALSE;
    xQueueSendFromISR(edge_queue, &evt, &hp_woken);
    if (hp_woken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Reset the RX decoder state.
 */
static void rx_reset(void) {
    rx_state = RX_STATE_IDLE;
    rx_bit_count = 0;
    rx_bit_buffer = 0;
    rx_byte_count = 0;
}

/**
 * @brief Receiver task - drains edge queue, decodes Manchester frames.
 *
 * Classification is done on the duration of HIGH pulses (falling edges).
 * A long idle HIGH (>50ms) between frame groups terminates the current frame.
 */
static void receiver_task(void *arg) {
    ESP_LOGI(TAG, "Receiver task started (ISR-driven)");

    int64_t last_time = esp_timer_get_time();
    uint8_t last_level = (uint8_t)gpio_get_level(data_gpio);

    uint32_t edge_count = 0;
    uint32_t start_count = 0;
    uint32_t frame_count = 0;
    int64_t last_report = esp_timer_get_time();
    const int64_t REPORT_INTERVAL_US = 10 * 1000 * 1000;

    edge_event_t evt;
    while (1) {
        if (xQueueReceive(edge_queue, &evt, pdMS_TO_TICKS(1000)) == pdTRUE) {
            int64_t duration = evt.timestamp - last_time;
            edge_count++;

            if (last_level == 1 && evt.level == 0) {
                // Falling edge - HIGH pulse just ended, classify by duration
                if (duration > RX_EOF_MIN_US) {
                    // Long idle HIGH (>50ms) = end of previous frame
                    if (rx_state == RX_STATE_RECEIVING && rx_byte_count > 0) {
                        uint8_t frame_copy[RX_BUFFER_SIZE];
                        memcpy(frame_copy, rx_buffer, rx_byte_count);
                        process_received_frame(frame_copy, rx_byte_count);
                        frame_count++;
                    }
                    rx_reset();
                } else if (duration >= RX_START_MIN_US && duration <= RX_START_MAX_US) {
                    rx_state = RX_STATE_RECEIVING;
                    rx_bit_count = 0;
                    rx_bit_buffer = 0;
                    rx_byte_count = 0;
                    start_count++;
                } else if (rx_state == RX_STATE_RECEIVING) {
                    // Heat pump: long HIGH = bit 0, short HIGH = bit 1
                    if (duration >= RX_LONG_MIN_US && duration <= RX_LONG_MAX_US) {
                        rx_bit_buffer = (uint8_t)(rx_bit_buffer << 1);
                        rx_bit_count++;
                    } else if (duration >= RX_SHORT_MIN_US && duration <= RX_SHORT_MAX_US) {
                        rx_bit_buffer = (uint8_t)((rx_bit_buffer << 1) | 1);
                        rx_bit_count++;
                    } else {
                        // Out-of-spec pulse mid-frame - abort
                        ESP_LOGD(TAG, "Abort frame: out-of-range HIGH %lld us", duration);
                        rx_reset();
                    }

                    if (rx_bit_count == 8) {
                        if (rx_byte_count < RX_BUFFER_SIZE) {
                            rx_buffer[rx_byte_count++] = rx_bit_buffer;
                        }
                        rx_bit_count = 0;
                        rx_bit_buffer = 0;
                    }
                }
            }
            // Rising edges are not used - EOF is detected on the falling edge
            // following a long idle HIGH period

            last_time = evt.timestamp;
            last_level = evt.level;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_report >= REPORT_INTERVAL_US) {
            ESP_LOGI(TAG, "RX 10s: %lu edges, %lu frame-starts, %lu decoded | LEVEL=%d valid=%s",
                     edge_count, start_count, frame_count,
                     gpio_get_level(data_gpio),
                     current_status.valid ? "YES" : "NO");
            if (edge_count == 0) {
                ESP_LOGW(TAG, "No GPIO activity - check heat pump power, wiring, ground");
            }
            edge_count = 0;
            start_count = 0;
            frame_count = 0;
            last_report = now;
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
    // Gate the RX ISR so we don't capture our own transmission as edges,
    // then switch to output mode.
    rx_isr_enabled = false;
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
    
    // Back to input mode. Drain any edge events queued during direction-switch
    // settling, then re-enable the RX ISR.
    gpio_set_direction(data_gpio, GPIO_MODE_INPUT);
    xQueueReset(edge_queue);
    rx_isr_enabled = true;

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

    // Configure GPIO as input with both-edge interrupt.
    // External 10K pull-ups on both sides of the BSS138 level shifter - no internal pulls.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
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

    int initial_level = gpio_get_level(data_gpio);
    ESP_LOGI(TAG, "GPIO%d initial level: %d (expect 1/HIGH when idle)", data_gpio, initial_level);

    edge_queue = xQueueCreate(EDGE_QUEUE_DEPTH, sizeof(edge_event_t));
    if (!edge_queue) {
        ESP_LOGE(TAG, "Failed to create edge queue");
        return ESP_ERR_NO_MEM;
    }

    // The ISR service may already be installed (e.g. by the boot-button init).
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(data_gpio, data_gpio_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    rx_isr_enabled = true;

    BaseType_t task_ret = xTaskCreate(receiver_task, "pc1001_rx", 4096, NULL,
                                      configMAX_PRIORITIES - 3, &receiver_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receiver task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PC1001 driver initialized on GPIO%d (ISR-driven)", data_gpio);
    ESP_LOGI(TAG, "Waiting for heat pump status frames (type 0x81)...");
    return ESP_OK;
}

esp_err_t pc1001_driver_deinit(void) {
    rx_isr_enabled = false;
    if (data_gpio != GPIO_NUM_NC) {
        gpio_isr_handler_remove(data_gpio);
    }
    if (receiver_task_handle) {
        vTaskDelete(receiver_task_handle);
        receiver_task_handle = NULL;
    }
    if (edge_queue) {
        vQueueDelete(edge_queue);
        edge_queue = NULL;
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
        ESP_LOGW(TAG, "No heat pump status received yet - sending blind command");
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
    
    // Transmit (gates RX ISR internally)
    transmit_command_frame();

    // Reset receiver state - a new frame may arrive any moment now
    rx_reset();

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
