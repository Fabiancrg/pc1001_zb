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
#include "ha/esp_zigbee_ha_standard.h"
#include "pc1001_driver.h"
#include "version.h"

static const char *TAG = "PC1001_ZB";

/* GPIO Configuration */
#define PC1001_DATA_GPIO        GPIO_NUM_2      // GPIO2 for PC1001 data line

/* Zigbee Channel Configuration */
#define ESP_ZB_PRIMARY_CHANNEL_MASK    (1 << 15)  // Channel 15

/* Zigbee Endpoints */
#define HA_THERMOSTAT_ENDPOINT      1
#define HA_TEMP_OUTPUT_ENDPOINT     2       // Temperature output sensor (TEMP_OUT)

/* Manufacturer Information */
#define MANUFACTURER_NAME       "Fabiancrg"
#define MODEL_IDENTIFIER        "PC1001_ZB_v1"

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
                .keep_alive = 0,  // 0 = always awake (mains powered)
            },
        },
    };
    
    esp_zb_init(&zb_nwk_cfg);
    
    // Set rx_on_when_idle to true (device never sleeps - mains powered)
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
    
    // Add manufacturer info
    uint8_t manufacturer_name[sizeof(MANUFACTURER_NAME)] = {sizeof(MANUFACTURER_NAME) - 1};
    memcpy(&manufacturer_name[1], MANUFACTURER_NAME, sizeof(MANUFACTURER_NAME) - 1);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 
                                  manufacturer_name);
    
    uint8_t model_identifier[sizeof(MODEL_IDENTIFIER)] = {sizeof(MODEL_IDENTIFIER) - 1};
    memcpy(&model_identifier[1], MODEL_IDENTIFIER, sizeof(MODEL_IDENTIFIER) - 1);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
                                  ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 
                                  model_identifier);
    
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
    
    ESP_LOGI(TAG, "ESP32-C6 PC1001 Pool Heat Pump Controller");
    ESP_LOGI(TAG, "Version: %s", FW_VERSION);
    ESP_LOGI(TAG, "Zigbee: End Device (mains powered, no sleep)");
    
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
