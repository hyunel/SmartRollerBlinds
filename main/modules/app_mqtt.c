#include "app_mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdlib.h>

#include "esp_event.h"

static const char *TAG = "APP_MQTT";
static esp_mqtt_client_handle_t client = NULL;
static motor_handle_t mqtt_motor_handle = NULL;
static TaskHandle_t status_publish_task_handle = NULL;
static EventGroupHandle_t status_task_event_group = NULL;

// Event group bits
static const int MOTOR_IS_MOVING_BIT = BIT0;
static const int MQTT_STOPPING_BIT   = BIT1;

static char mqtt_base_topic[64]; // e.g., "blinds/living_room"
static char mqtt_command_topic[128];
static char mqtt_position_set_topic[128];
static char mqtt_status_topic[128];

// Variables to hold the last known state
static int last_position = -1;
static int last_target_position = -1;
static motor_state_t last_state = -1; // Use an invalid initial value
static bool last_calibrated_state = false;

static void publish_state_value(const char* sub_topic, const char* value, int qos, bool retain) {
    if (client && strlen(mqtt_base_topic) > 0) {
        char full_topic[128];
        snprintf(full_topic, sizeof(full_topic), "%s/%s", mqtt_base_topic, sub_topic);
        esp_mqtt_client_publish(client, full_topic, value, 0, qos, retain);
        ESP_LOGD(TAG, "Published to %s: %s (QoS %d)", full_topic, value, qos);
    }
}

static void check_and_publish_status_updates(int qos_position) {
    if (!client || !mqtt_motor_handle) return;

    // 1. Check and publish position
    int current_position = motor_get_position(mqtt_motor_handle);
    if (current_position != last_position) {
        char pos_str[16];
        snprintf(pos_str, sizeof(pos_str), "%d", current_position);
        publish_state_value("position", pos_str, qos_position, true);
        last_position = current_position;
    }

    // 2. Check and publish target position
    int current_target = motor_get_target_position(mqtt_motor_handle);
    if (current_target != last_target_position) {
        char target_str[16];
        snprintf(target_str, sizeof(target_str), "%d", current_target);
        publish_state_value("target", target_str, 1, true);
        last_target_position = current_target;
    }

    // 3. Check and publish state
    motor_state_t current_state = motor_get_state(mqtt_motor_handle);
    if (current_state != last_state) {
        const char *state_str;
        switch (current_state) {
            case MOTOR_STATE_IDLE: state_str = "idle"; break;
            case MOTOR_STATE_MOVING: state_str = "moving"; break;
            case MOTOR_STATE_CALIBRATING: state_str = "calibrating"; break;
            case MOTOR_STATE_HOMING: state_str = "homing"; break;
            case MOTOR_STATE_ERROR: state_str = "error"; break;
            default: state_str = "unknown"; break;
        }
        publish_state_value("state", state_str, 2, true);
        last_state = current_state;
    }

    // 4. Check and publish calibrated status
    bool is_calibrated = motor_is_calibrated(mqtt_motor_handle);
    // Publish initial calibrated state or if it changes
    if (last_state == -1 || is_calibrated != last_calibrated_state) {
        publish_state_value("calibrated", is_calibrated ? "true" : "false", 2, true);
        last_calibrated_state = is_calibrated;
    }
}

static void status_publish_task(void *pvParameters) {
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            status_task_event_group,
            MOTOR_IS_MOVING_BIT | MQTT_STOPPING_BIT,
            pdFALSE, // Don't clear on exit, we check the state below
            pdFALSE, // Wait for any bit
            portMAX_DELAY);

        // Check for shutdown signal
        if (bits & MQTT_STOPPING_BIT) {
            break; // Exit main loop
        }

        // Check if motor is moving
        if (bits & MOTOR_IS_MOVING_BIT) {
            // Loop for periodic updates while motor is moving
            while (xEventGroupGetBits(status_task_event_group) & MOTOR_IS_MOVING_BIT) {
                check_and_publish_status_updates(0); // QoS 0 for position during movement

                // Wait for the update interval, but wake up immediately if the state changes
                xEventGroupWaitBits(
                    status_task_event_group,
                    MQTT_STOPPING_BIT, // We only wait for the stop bit here
                    pdFALSE, // Don't clear on exit
                    pdFALSE, // Wait for any bit
                    pdMS_TO_TICKS(CONFIG_MQTT_STATUS_UPDATE_INTERVAL_MS));

                // Check for shutdown signal inside the loop
                if (xEventGroupGetBits(status_task_event_group) & MQTT_STOPPING_BIT) {
                    break;
                }
            }
            // Publish final state one last time after stopping
            check_and_publish_status_updates(2); // QoS 2 for final position
        }
    }

    // Cleanup and self-delete
    status_publish_task_handle = NULL; // Signal that we have exited
    vTaskDelete(NULL);
}

static void motor_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    motor_state_t new_state = *(motor_state_t *)event_data;

    // The status_publish_task is responsible for publishing all state changes.
    // We just signal the event here. The task will wake up and publish the new state.
    // This prevents message flooding if events arrive in quick succession.

    bool is_moving = (new_state == MOTOR_STATE_MOVING || new_state == MOTOR_STATE_CALIBRATING || new_state == MOTOR_STATE_HOMING);

    if (status_task_event_group != NULL) {
        if (is_moving) {
            xEventGroupSetBits(status_task_event_group, MOTOR_IS_MOVING_BIT);
        } else {
            xEventGroupClearBits(status_task_event_group, MOTOR_IS_MOVING_BIT);
        }
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            if (strlen(mqtt_command_topic) > 0) {
                msg_id = esp_mqtt_client_subscribe(client, mqtt_command_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", mqtt_command_topic, msg_id);
            }
            if (strlen(mqtt_position_set_topic) > 0) {
                msg_id = esp_mqtt_client_subscribe(client, mqtt_position_set_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", mqtt_position_set_topic, msg_id);
            }
            
            // Reset last known states to force a full update on connection
            last_position = -1;
            last_target_position = -1;
            last_state = -1;
            // last_calibrated_state does not need reset, it will be published if different

            // Publish initial status on connection
            check_and_publish_status_updates(2); // QoS 2 for initial position
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);

            // Null-terminate the data to treat it as a C string
            char* data_str = strndup(event->data, event->data_len);
            if (!data_str) {
                ESP_LOGE(TAG, "Failed to allocate memory for data string");
                break;
            }

            if (strncmp(event->topic, mqtt_position_set_topic, event->topic_len) == 0) {
                int percentage = atoi(data_str);
                ESP_LOGI(TAG, "Received position set command: %d%%", percentage);
                motor_go_to_percentage(mqtt_motor_handle, percentage, CONFIG_DEFAULT_MOVE_SPEED);

            } else if (strncmp(event->topic, mqtt_command_topic, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Received command: %s", data_str);
                if (strcmp(data_str, "SET_FULLY_CLOSED") == 0) {
                    motor_set_fully_closed_position(mqtt_motor_handle);
                } else if (strcmp(data_str, "CALIBRATE") == 0) {
                    motor_calibrate(mqtt_motor_handle, CONFIG_DEFAULT_MOVE_SPEED);
                } else if (strcmp(data_str, "GO_TO_ZERO") == 0) {
                    motor_go_to_zero(mqtt_motor_handle, CONFIG_DEFAULT_MOVE_SPEED);
                } else {
                    ESP_LOGW(TAG, "Unknown command received on command topic: %s", data_str);
                }
            }

            free(data_str);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    mqtt_event_handler_cb(event_data);
}

esp_err_t app_mqtt_start(motor_handle_t motor_handle) {
    mqtt_motor_handle = motor_handle;

    nvs_handle_t nvs_handle;
    char mqtt_server[64] = {0};
    char mqtt_port_str[6] = {0};
    char mqtt_user[32] = {0};
    char mqtt_pass[64] = {0};
    
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return err;
    }

    size_t required_size;
    nvs_get_str(nvs_handle, "mqtt_server", NULL, &required_size);
    nvs_get_str(nvs_handle, "mqtt_server", mqtt_server, &required_size);
    nvs_get_str(nvs_handle, "mqtt_port", NULL, &required_size);
    nvs_get_str(nvs_handle, "mqtt_port", mqtt_port_str, &required_size);
    nvs_get_str(nvs_handle, "mqtt_user", NULL, &required_size);
    nvs_get_str(nvs_handle, "mqtt_user", mqtt_user, &required_size);
    nvs_get_str(nvs_handle, "mqtt_pass", NULL, &required_size);
    nvs_get_str(nvs_handle, "mqtt_pass", mqtt_pass, &required_size);

    // Read base topic from NVS
    memset(mqtt_base_topic, 0, sizeof(mqtt_base_topic));
    size_t base_topic_len;
    if (nvs_get_str(nvs_handle, "mqtt_base_topic", NULL, &base_topic_len) == ESP_OK) {
        nvs_get_str(nvs_handle, "mqtt_base_topic", mqtt_base_topic, &base_topic_len);
    } else {
        ESP_LOGE(TAG, "MQTT base_topic not configured in NVS. Cannot start.");
        nvs_close(nvs_handle);
        return ESP_FAIL;
    }

    // Construct full topic strings
    snprintf(mqtt_command_topic, sizeof(mqtt_command_topic), "%s/command", mqtt_base_topic);
    snprintf(mqtt_position_set_topic, sizeof(mqtt_position_set_topic), "%s/position/set", mqtt_base_topic);
    // The status topic is now the base topic itself, sub-topics are added in publish_state_value
    snprintf(mqtt_status_topic, sizeof(mqtt_status_topic), "%s/status", mqtt_base_topic);

    nvs_close(nvs_handle);

    if (strlen(mqtt_server) == 0) {
        ESP_LOGE(TAG, "MQTT server not configured.");
        return ESP_FAIL;
    }

    char uri[128];
    snprintf(uri, sizeof(uri), "mqtt://%s:%s", mqtt_server, mqtt_port_str);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
    };
    if (strlen(mqtt_user) > 0) {
        mqtt_cfg.credentials.username = mqtt_user;
    }
    if (strlen(mqtt_pass) > 0) {
        mqtt_cfg.credentials.authentication.password = mqtt_pass;
    }

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    // Register for motor events on the default event loop
    esp_event_handler_register(MOTOR_EVENTS, MOTOR_EVENT_STATE_CHANGED, motor_event_handler, NULL);

    // Create the event group and task for status publishing
    status_task_event_group = xEventGroupCreate();
    if (status_task_event_group != NULL) {
        xTaskCreate(status_publish_task, "mqtt_status_task", 4096, NULL, 5, &status_publish_task_handle);
    } else {
        ESP_LOGE(TAG, "Failed to create status task event group");
    }

    return ESP_OK;
}

esp_err_t app_mqtt_stop(void) {
    if (client) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
    }
    // Unregister from motor events on the default event loop
    esp_event_handler_unregister(MOTOR_EVENTS, MOTOR_EVENT_STATE_CHANGED, motor_event_handler);

    // Signal the status task to stop and wait for it to terminate
    if (status_task_event_group != NULL) {
        ESP_LOGI(TAG, "Signaling status task to stop.");
        xEventGroupSetBits(status_task_event_group, MQTT_STOPPING_BIT);

        // Wait for the task to delete itself
        while (status_publish_task_handle != NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        ESP_LOGI(TAG, "Status task has stopped.");

        vEventGroupDelete(status_task_event_group);
        status_task_event_group = NULL;
    }

    return ESP_OK;
}
