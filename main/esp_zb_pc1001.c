/*
 * ESP32-C6 Zigbee PC1001 Pool Heat Pump Controller
 * 
 * Zigbee Router device with Thermostat cluster
 * Controls Hayward/Majestic PC1001 heat pump via Manchester protocol
 * Mains powered - no sleep mode
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "pc1001_driver.h"
#include "esp_zb_ota.h"
#include "version.h"

static const char *TAG = "PC1001_ZB";

/* GPIO Configuration */
#define PC1001_DATA_GPIO        GPIO_NUM_2      // GPIO2 for PC1001 data line

/* Boot button configuration for factory reset */
#define BOOT_BUTTON_GPIO        GPIO_NUM_9
#define BUTTON_LONG_PRESS_TIME_MS   5000

/* Zigbee Channel Configuration */
#define ESP_ZB_PRIMARY_CHANNEL_MASK    ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  // Scan all channels

/* Zigbee Endpoints */
#define HA_THERMOSTAT_ENDPOINT      1
#define HA_TEMP_OUTPUT_ENDPOINT     2       // Temperature output sensor (TEMP_OUT)

/* Manufacturer Information - Zigbee string format (first byte = length) */
#define MANUFACTURER_NAME       "\x14""Custom devices (DiY)"        // Length: 20 bytes
#define MODEL_IDENTIFIER        "\x09""PC1001_ZB"                   // Length: 9 bytes

/* Temperature conversion */
#define TEMP_TO_ZCL(temp_c)     ((int16_t)((temp_c) * 100))  // °C to centidegrees
#define ZCL_TO_TEMP(zcl_val)    ((float)(zcl_val) / 100.0f)  // centidegrees to °C

/* System mode mapping */
#define ZCL_SYSTEM_MODE_OFF     0x00
#define ZCL_SYSTEM_MODE_AUTO    0x01
#define ZCL_SYSTEM_MODE_COOL    0x03
#define ZCL_SYSTEM_MODE_HEAT    0x04

/* Network connection tracking */
static bool zigbee_connected = false;

/* Deferred update flag */
static bool pending_update = false;
static pc1001_cmd_t pending_cmd;

/* Boot button state */
static QueueHandle_t button_evt_queue = NULL;

/********************* Function Declarations **************************/
static esp_err_t button_init(void);
static void button_task(void *arg);
static void factory_reset_device(uint8_t param);

/* Factory reset function */
static void factory_reset_device(uint8_t param)
{
    ESP_LOGW(TAG, "[RESET] Performing factory reset...");
    esp_zb_factory_reset();
    ESP_LOGI(TAG, "[RESET] Factory reset successful - device will restart");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/* Boot button ISR handler */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t gpio_num = BOOT_BUTTON_GPIO;
    xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
}

/* Button monitoring task */
static void button_task(void *arg)
{
    uint32_t io_num;
    TickType_t press_start_time = 0;
    bool long_press_triggered = false;
    const TickType_t LONG_PRESS_DURATION = pdMS_TO_TICKS(BUTTON_LONG_PRESS_TIME_MS);
    const TickType_t DEBOUNCE_TIME = pdMS_TO_TICKS(50);  // 50ms debounce
    
    ESP_LOGI(TAG, "[BUTTON] Task started - waiting for button events");
    
    for (;;) {
        if (xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            // Debounce
            vTaskDelay(DEBOUNCE_TIME);
            
            gpio_intr_disable(BOOT_BUTTON_GPIO);
            int button_level = gpio_get_level(BOOT_BUTTON_GPIO);
            
            if (button_level == 0) {  // Button pressed
                press_start_time = xTaskGetTickCount();
                long_press_triggered = false;
                ESP_LOGI(TAG, "[BUTTON] Pressed - hold 5 sec for factory reset");
                
                // Poll button state with timeout to prevent infinite loop
                TickType_t poll_count = 0;
                const TickType_t MAX_POLL_TIME = pdMS_TO_TICKS(10000);  // 10s max
                
                while (gpio_get_level(BOOT_BUTTON_GPIO) == 0 && poll_count < MAX_POLL_TIME) {
                    TickType_t current_time = xTaskGetTickCount();
                    if ((current_time - press_start_time) >= LONG_PRESS_DURATION && !long_press_triggered) {
                        long_press_triggered = true;
                        ESP_LOGW(TAG, "[BUTTON] Long press detected! Triggering factory reset...");
                        esp_zb_scheduler_alarm((esp_zb_callback_t)factory_reset_device, 0, 100);
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                    poll_count += pdMS_TO_TICKS(100);
                }
                
                if (!long_press_triggered) {
                    uint32_t press_duration = pdTICKS_TO_MS(xTaskGetTickCount() - press_start_time);
                    ESP_LOGI(TAG, "[BUTTON] Released (held for %lu ms)", press_duration);
                }
            }
            
            // Re-enable interrupt after debounce
            vTaskDelay(DEBOUNCE_TIME);
            gpio_intr_enable(BOOT_BUTTON_GPIO);
        }
    }
}

/* Initialize boot button */
static esp_err_t button_init(void)
{
    ESP_LOGI(TAG, "[INIT] Initializing boot button on GPIO%d", BOOT_BUTTON_GPIO);
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!button_evt_queue) {
        ESP_LOGE(TAG, "[ERROR] Failed to create event queue");
        return ESP_FAIL;
    }
    
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "[ERROR] Failed to install ISR service: %s", esp_err_to_name(isr_ret));
        return isr_ret;
    }
    
    ret = gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    BaseType_t task_ret = xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "[ERROR] Failed to create button task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "[OK] Boot button initialization complete");
    return ESP_OK;
}

/**
 * @brief Map PC1001 mode to Zigbee system mode
 */
static uint8_t pc1001_mode_to_zcl(pc1001_mode_t mode, bool power) {
    if (!power) {
        return ZCL_SYSTEM_MODE_OFF;
    }
    
    switch (mode) {
        case PC1001_MODE_HEAT:
            return ZCL_SYSTEM_MODE_HEAT;
        case PC1001_MODE_COOL:
            return ZCL_SYSTEM_MODE_COOL;
        case PC1001_MODE_AUTO:
            return ZCL_SYSTEM_MODE_AUTO;
        default:
            return ZCL_SYSTEM_MODE_OFF;
    }
}

/**
 * @brief Map Zigbee system mode to PC1001 mode and power
 */
static void zcl_to_pc1001_mode(uint8_t system_mode, pc1001_mode_t *mode, bool *power) {
    switch (system_mode) {
        case ZCL_SYSTEM_MODE_HEAT:
            *mode = PC1001_MODE_HEAT;
            *power = true;
            break;
        case ZCL_SYSTEM_MODE_COOL:
            *mode = PC1001_MODE_COOL;
            *power = true;
            break;
        case ZCL_SYSTEM_MODE_AUTO:
            *mode = PC1001_MODE_AUTO;
            *power = true;
            break;
        case ZCL_SYSTEM_MODE_OFF:
        default:
            *mode = PC1001_MODE_COOL;  // Default to cool when off
            *power = false;
            break;
    }
}

/**
 * @brief Update Zigbee attributes from PC1001 status
 */
static void update_zigbee_attributes(const pc1001_status_t *status) {
    if (!zigbee_connected || !status->valid) {
        return;
    }
    
    ESP_LOGI(TAG, "Updating Zigbee attributes from PC1001 status");
    
    // Update local temperature (water inlet)
    int16_t local_temp = TEMP_TO_ZCL(status->temp_in);
    esp_zb_zcl_set_attribute_val(HA_THERMOSTAT_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                  &local_temp,
                                  false);
    
    // Update occupied heating setpoint
    int16_t heat_setpoint = TEMP_TO_ZCL(status->temp_prog);
    esp_zb_zcl_set_attribute_val(HA_THERMOSTAT_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                  &heat_setpoint,
                                  false);
    
    // Update occupied cooling setpoint
    int16_t cool_setpoint = TEMP_TO_ZCL(status->temp_prog);
    esp_zb_zcl_set_attribute_val(HA_THERMOSTAT_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                                  &cool_setpoint,
                                  false);
    
    // Update system mode
    uint8_t system_mode = pc1001_mode_to_zcl(status->mode, status->power);
    esp_zb_zcl_set_attribute_val(HA_THERMOSTAT_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                  &system_mode,
                                  false);
    
    // Update temperature output sensor (EP2) - water outlet temperature
    int16_t temp_out = TEMP_TO_ZCL(status->temp_out);
    esp_zb_zcl_set_attribute_val(HA_TEMP_OUTPUT_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                  &temp_out,
                                  false);
    
    ESP_LOGI(TAG, "Attributes updated: %.1f°C (in), %.1f°C (out), %.1f°C (prog), mode=%s",
             status->temp_in, status->temp_out, status->temp_prog, pc1001_mode_to_string(status->mode));
}

/**
 * @brief PC1001 status callback
 */
static void pc1001_status_callback(const pc1001_status_t *status) {
    ESP_LOGI(TAG, "PC1001 status update received");
    update_zigbee_attributes(status);
}

/**
 * @brief Wrapper for commissioning that matches callback signature
 */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

/**
 * @brief Send pending PC1001 command
 */
static void send_pending_command(uint8_t param) {
    (void)param;
    
    if (!pending_update) {
        return;
    }
    
    ESP_LOGI(TAG, "Sending pending command to PC1001");
    esp_err_t ret = pc1001_send_command(&pending_cmd);
    if (ret == ESP_OK) {
        pending_update = false;
    } else {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Zigbee attribute write handler
 */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    esp_err_t ret = ESP_OK;
    
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, 
                        TAG, "Received message: error status(%d)", message->info.status);
    
    ESP_LOGI(TAG, "Received ZCL message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
             message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    
    if (message->info.dst_endpoint != HA_THERMOSTAT_ENDPOINT) {
        return ESP_OK;
    }
    
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        pc1001_status_t current_status;
        if (pc1001_get_status(&current_status) != ESP_OK) {
            ESP_LOGW(TAG, "No PC1001 status available yet");
            return ESP_OK;
        }
        
        // Prepare command with current values
        pending_cmd.temp = current_status.temp_prog;
        pending_cmd.mode = current_status.mode;
        pending_cmd.power = current_status.power;
        
        switch (message->attribute.id) {
            case ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
            case ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID: {
                int16_t setpoint = message->attribute.data.value ? 
                                  *(int16_t *)message->attribute.data.value : 0;
                float temp = ZCL_TO_TEMP(setpoint);
                
                ESP_LOGI(TAG, "Setpoint change: %.1f°C", temp);
                pending_cmd.temp = temp;
                pending_update = true;
                break;
            }
            
            case ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID: {
                uint8_t system_mode = message->attribute.data.value ? 
                                     *(uint8_t *)message->attribute.data.value : 0;
                
                ESP_LOGI(TAG, "System mode change: %d", system_mode);
                zcl_to_pc1001_mode(system_mode, &pending_cmd.mode, &pending_cmd.power);
                pending_update = true;
                break;
            }
            
            default:
                ESP_LOGD(TAG, "Unhandled attribute: 0x%x", message->attribute.id);
                break;
        }
        
        // Schedule command transmission
        if (pending_update) {
            esp_zb_scheduler_alarm((esp_zb_callback_t)send_pending_command, 0, 100);
        }
    }
    
    return ret;
}

/**
 * @brief Zigbee action callback handler
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    esp_err_t ret = ESP_OK;
    
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
            break;
        
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            ESP_LOGD(TAG, "[OTA] Upgrade value callback triggered");
            ret = zb_ota_upgrade_value_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
            break;
        
        case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
            ESP_LOGI(TAG, "[OTA] Query image response callback triggered");
            ret = zb_ota_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
            break;
            
        default:
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
    }
    
    return ret;
}

/**
 * @brief Zigbee signal handler
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "End Device started up in %s factory-reset mode", 
                         esp_zb_bdb_is_factory_new() ? "" : "non");
                
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering (joining)");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted - rejoining network");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                }
            } else {
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", 
                         esp_err_to_name(err_status));
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                
                zigbee_connected = true;
                
                // Initial status read
                pc1001_status_t status;
                if (pc1001_get_status(&status) == ESP_OK) {
                    update_zigbee_attributes(&status);
                }
            } else {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", 
                         esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                      ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
            
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", 
                     esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

/**
 * @brief Zigbee task
 */
static void esp_zb_task(void *pvParameters) {
    // Initialize Zigbee stack as End Device (mains powered, no sleep)
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
                .keep_alive = 3000,  // Keep-alive interval in milliseconds (3 seconds)
            },
        },
    };
    
    esp_zb_init(&zb_nwk_cfg);
    
    // Set rx_on_when_idle to true (device always listening - mains powered)
    esp_zb_set_rx_on_when_idle(true);
    
    // Create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    
    // Create thermostat cluster list
    esp_zb_cluster_list_t *esp_zb_thermostat_clusters = esp_zb_zcl_cluster_list_create();
    
    // Basic cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,  // Mains powered
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    
    // Add manufacturer info (already in Zigbee string format with length byte)
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 
                                  (void *)MANUFACTURER_NAME);
    
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 
                                  (void *)MODEL_IDENTIFIER);
    
    // Add optional Basic cluster attributes for version info
    #ifdef FW_VERSION_MAJOR
    // Application version (encode as major.minor in hex)
    uint8_t app_version = (FW_VERSION_MAJOR << 4) | FW_VERSION_MINOR;  // e.g., 1.0 = 0x10
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, 
                                  &app_version);
    #endif
    
    // Stack version (Zigbee 3.0 = 0x03)
    #ifdef ZB_STACK_VERSION
    uint8_t stack_version = ZB_STACK_VERSION & 0xFF;
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, 
                                  &stack_version);
    #endif
    
    // Hardware version
    uint8_t hw_version = 1;
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, 
                                  &hw_version);
    
    // Date code (format: YYYYMMDD)
    #ifdef FW_DATE_CODE
    static char date_code[17];  // 16 chars max + null terminator
    const char *date_str = FW_DATE_CODE;
    size_t date_len = strlen(date_str);
    if (date_len > 16) date_len = 16;
    date_code[0] = date_len;  // First byte is length
    memcpy(&date_code[1], date_str, date_len);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, 
                                  date_code);
    #endif
    
    // Software build ID (firmware version string)
    #ifdef FW_VERSION
    static char sw_build_id[17];  // 16 chars max + null terminator
    const char *version_str = FW_VERSION;
    size_t version_len = strlen(version_str);
    if (version_len > 16) version_len = 16;
    sw_build_id[0] = version_len;  // First byte is length
    memcpy(&sw_build_id[1], version_str, version_len);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, 
                                  sw_build_id);
    #endif
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_thermostat_clusters, 
                                                          esp_zb_basic_cluster, 
                                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Thermostat cluster with REPORTING flag for attribute persistence
    // According to ESP Zigbee SDK: ESP_ZB_ZCL_ATTR_ACCESS_REPORTING flag enables
    // reporting configuration to persist across reboots in zb_storage partition
    int16_t local_temp = TEMP_TO_ZCL(20.0f);
    int16_t cool_setpoint = TEMP_TO_ZCL(26.0f);
    int16_t heat_setpoint = TEMP_TO_ZCL(20.0f);
    uint8_t control_seq = 0x04;  // Cooling and heating
    uint8_t sys_mode = ZCL_SYSTEM_MODE_OFF;
    
    esp_zb_attribute_list_t *esp_zb_thermostat_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    
    // Local temperature (read-only, reportable)
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &local_temp));
    
    // Cooling setpoint (read-write, reportable)
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &cool_setpoint));
    
    // Heating setpoint (read-write, reportable)
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &heat_setpoint));
    
    // Control sequence of operation
    ESP_ERROR_CHECK(esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_cluster,
                                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID, &control_seq));
    
    // System mode (read-write, reportable)
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &sys_mode));
    
    // Add min/max setpoint limit attributes
    int16_t min_setpoint = TEMP_TO_ZCL(15.0f);
    int16_t max_setpoint = TEMP_TO_ZCL(32.0f);
    ESP_ERROR_CHECK(esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &min_setpoint));
    ESP_ERROR_CHECK(esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &max_setpoint));
    ESP_ERROR_CHECK(esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID, &min_setpoint));
    ESP_ERROR_CHECK(esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID, &max_setpoint));
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_thermostat_cluster(esp_zb_thermostat_clusters, 
                                                               esp_zb_thermostat_cluster, 
                                                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Identify cluster
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_thermostat_clusters, 
                                                             esp_zb_identify_cluster_create(&identify_cfg), 
                                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Add OTA cluster for firmware updates
    ESP_LOGI(TAG, "  [+] Adding OTA cluster (0x0019)...");
    uint32_t fw_version = esp_zb_ota_get_fw_version();
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = fw_version,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
        .ota_upgrade_downloaded_file_ver = 0xFFFFFFFF,
    };
    esp_zb_attribute_list_t *esp_zb_ota_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    
    // Add OTA cluster attributes
    uint32_t current_file_version = ota_cluster_cfg.ota_upgrade_file_version;
    esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, 0x0003, &current_file_version);
    
    // Add client-specific OTA attributes
    esp_zb_zcl_ota_upgrade_client_variable_t client_vars = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = 0x0101,
        .max_data_size = 223,
    };
    esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &client_vars);
    
    uint16_t server_addr = 0xffff;
    esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &server_addr);
    
    uint8_t server_ep = 0xff;
    esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &server_ep);
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(esp_zb_thermostat_clusters, esp_zb_ota_cluster,
                                                        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_LOGI(TAG, "  [OK] OTA cluster added (FW version: 0x%08lX)", ota_cluster_cfg.ota_upgrade_file_version);
    
    // Create endpoint
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_THERMOSTAT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
        .app_device_version = 0
    };
    
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_thermostat_clusters, endpoint_config));
    
    /* Create temperature output sensor endpoint (EP2)
     * Basic cluster intentionally omitted - only on primary endpoint (EP1)
     * This endpoint reports TEMP_OUT from PC1001 (water outlet temperature) */
    esp_zb_cluster_list_t *esp_zb_temp_out_clusters = esp_zb_zcl_cluster_list_create();
    
    // Temperature measurement cluster with REPORTING flag
    int16_t temp_out_value = 0x8000;  // Invalid/unknown initially
    int16_t temp_out_min = -40 * 100;  // -40°C in centidegrees
    int16_t temp_out_max = 60 * 100;   // 60°C in centidegrees (pool water range)
    esp_zb_attribute_list_t *esp_zb_temp_out_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_temp_out_cluster, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp_out_value));
    ESP_ERROR_CHECK(esp_zb_temperature_meas_cluster_add_attr(esp_zb_temp_out_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &temp_out_min));
    ESP_ERROR_CHECK(esp_zb_temperature_meas_cluster_add_attr(esp_zb_temp_out_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &temp_out_max));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_temp_out_clusters, esp_zb_temp_out_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Identify cluster for temperature output endpoint
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_temp_out_clusters,
                                                             esp_zb_identify_cluster_create(&identify_cfg),
                                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t temp_out_endpoint_config = {
        .endpoint = HA_TEMP_OUTPUT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_temp_out_clusters, temp_out_endpoint_config));
    
    // Register device
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    
    // Set primary channel
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));
    
    // Start Zigbee stack
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    ESP_LOGI(TAG, "Zigbee stack started");
    
    // Main loop
    esp_zb_stack_main_loop();
}

void app_main(void) {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Check OTA status and validate firmware
    ESP_LOGI(TAG, "=== OTA Validation Check ===");
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "OTA firmware pending verification - validating...");
            // Firmware successfully booted after OTA update
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA firmware validated successfully!");
        } else if (ota_state == ESP_OTA_IMG_VALID) {
            ESP_LOGI(TAG, "Running validated firmware");
        } else if (ota_state == ESP_OTA_IMG_INVALID || ota_state == ESP_OTA_IMG_ABORTED) {
            ESP_LOGW(TAG, "Running firmware in invalid/aborted state");
        }
    }
    ESP_LOGI(TAG, "Running partition: %s at 0x%lx", running->label, running->address);
    
    // Initialize OTA
    ESP_ERROR_CHECK(esp_zb_ota_init());
    
    // Configure external antenna for ESP32-C6
    // GPIO3 controls antenna switch (HIGH = external, LOW = internal)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_3, 1);  // Enable external antenna
    ESP_LOGI(TAG, "[INIT] External antenna enabled on GPIO3");
    
    ESP_LOGI(TAG, "ESP32-C6 PC1001 Pool Heat Pump Controller");
    ESP_LOGI(TAG, "Version: %s", FW_VERSION);
    ESP_LOGI(TAG, "Zigbee: End Device (mains powered, no sleep)");
    
    // Initialize boot button for factory reset
    ESP_LOGI(TAG, "[INIT] Initializing boot button...");
    esp_err_t button_ret = button_init();
    if (button_ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Button initialization failed");
        // Continue anyway - button is optional
    }
    
    // Initialize PC1001 driver
    ESP_ERROR_CHECK(pc1001_driver_init(PC1001_DATA_GPIO, pc1001_status_callback));
    
    // Configure ESP-IDF platform
    esp_zb_platform_config_t config = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    // Start Zigbee task
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
