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
#include "esp_mac.h"
#include "esp_event.h"
#include "modules/command_handler.h"
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
static char mqtt_response_topic[128];
static char mqtt_set_topic[128];
static char mqtt_set_position_topic[128];

// Variables to hold the last known state
static int last_position = -1;
static motor_state_t last_state = -1;
static motor_direction_t last_direction = -1;
static bool last_calibrated_state = false;
static float last_speed = -1.0f;

static void publish_state_value(const char* sub_topic, const char* value, int qos, bool retain) {
    if (client && strlen(mqtt_base_topic) > 0) {
        char full_topic[128];
        snprintf(full_topic, sizeof(full_topic), "%s/%s", mqtt_base_topic, sub_topic);
        esp_mqtt_client_publish(client, full_topic, value, 0, qos, retain);
        ESP_LOGD(TAG, "Published to %s: %s (QoS %d)", full_topic, value, qos);
    }
}

static void publish_ha_discovery_message(void) {
    if (!client) return;

    char unique_id[32];
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(unique_id, sizeof(unique_id), "smartblinds_%02x%02x%02x", mac[3], mac[4], mac[5]);

    char discovery_topic[128];
    snprintf(discovery_topic, sizeof(discovery_topic), "homeassistant/cover/%s/config", unique_id);

    // Construct the individual topics that will be used in the payload
    char state_topic[128];
    char position_topic[128];
    snprintf(state_topic, sizeof(state_topic), "%s/state", mqtt_base_topic);
    snprintf(position_topic, sizeof(position_topic), "%s/position", mqtt_base_topic);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", unique_id);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "cmd_t", mqtt_set_topic);
    cJSON_AddStringToObject(root, "stat_t", state_topic);
    cJSON_AddStringToObject(root, "pos_t", position_topic);
    cJSON_AddStringToObject(root, "set_pos_t", mqtt_set_position_topic);
    cJSON_AddStringToObject(root, "pl_open", "OPEN");
    cJSON_AddStringToObject(root, "pl_cls", "CLOSE");
    cJSON_AddStringToObject(root, "pl_stop", "STOP");
    cJSON_AddStringToObject(root, "stat_open", "open");
    cJSON_AddStringToObject(root, "stat_closed", "closed");
    cJSON_AddStringToObject(root, "stat_opening", "opening");
    cJSON_AddStringToObject(root, "stat_closing", "closing");
    // Invert for Home Assistant: 100 is open, 0 is closed.
    cJSON_AddNumberToObject(root, "pos_open", 100);
    cJSON_AddNumberToObject(root, "pos_closed", 0);
    cJSON_AddTrueToObject(root, "opt"); // Optimistic mode for smoother UI

    cJSON *device = cJSON_CreateObject();
    cJSON_AddStringToObject(device, "ids", unique_id);
    cJSON_AddStringToObject(device, "name", unique_id);
    cJSON_AddStringToObject(device, "mf", "Hyun");
    cJSON_AddStringToObject(device, "mdl", CONFIG_DEVICE_NAME);
    cJSON_AddItemToObject(root, "device", device);

    char *json_payload = cJSON_PrintUnformatted(root);
    if (json_payload) {
        // A retained message of an empty payload will clear the discovery message
        esp_mqtt_client_publish(client, discovery_topic, "", 0, 1, true);
        // Publish the new discovery message
        esp_mqtt_client_publish(client, discovery_topic, json_payload, 0, 1, true);
        ESP_LOGI(TAG, "Published HA discovery message to %s", discovery_topic);
        free(json_payload);
    }
    cJSON_Delete(root);
}

static void check_and_publish_status_updates(int qos_position) {
    if (!client || !mqtt_motor_handle) return;

    // Determine if this is the first publish after connecting
    bool is_initial_publish = (last_state == -1);

    // 1. Get current values
    int32_t current_raw_pos;
    motor_get_position(mqtt_motor_handle, &current_raw_pos, NULL);
    motor_state_t current_state = motor_get_state(mqtt_motor_handle);
    motor_direction_t current_direction = motor_get_direction(mqtt_motor_handle);
    bool is_calibrated = motor_is_calibrated(mqtt_motor_handle);
    float current_speed = motor_get_default_speed(mqtt_motor_handle);
    int32_t fully_closed_pos = motor_get_fully_closed_position(mqtt_motor_handle);

    // 2. Check and publish position (as a percentage)
    int current_percentage = 0;
    if (is_calibrated && fully_closed_pos > 0) {
        // Calculate the percentage, ensuring it's clamped between 0 and 100.
        current_percentage = (int)(100.0f * (float)current_raw_pos / (float)fully_closed_pos);
        if (current_percentage < 0) current_percentage = 0;
        if (current_percentage > 100) current_percentage = 100;
    }

    if (is_initial_publish || current_percentage != last_position) {
        char pos_str[16];
        // The position topic for HA should be the percentage value.
        // Let's report the inverted value to HA, as it expects 100=open, 0=closed.
        // Our internal logic remains 0=open, 100=closed.
        snprintf(pos_str, sizeof(pos_str), "%d", 100 - current_percentage);
        publish_state_value("position", pos_str, qos_position, true);
        last_position = current_percentage; // We still store the non-inverted value as last_position
    }

    // 3. Check and publish state
    if (is_initial_publish || current_state != last_state || current_direction != last_direction) {
        const char *state_str = (current_percentage >= 99) ? "closed" : "open";

        if (current_state == MOTOR_STATE_MOVING || current_state == MOTOR_STATE_CALIBRATING) {
            if (current_direction == MOTOR_DIRECTION_UP) {
                state_str = "opening";
            } else if (current_direction == MOTOR_DIRECTION_DOWN) {
                state_str = "closing";
            }
        } else if (current_state == MOTOR_STATE_ERROR) {
            state_str = "error"; // Custom state
        }

        publish_state_value("state", state_str, 2, true);
        last_state = current_state;
        last_direction = current_direction;
    }

    // 5. Check and publish calibrated status
    if (is_initial_publish || is_calibrated != last_calibrated_state) {
        publish_state_value("calibrated", is_calibrated ? "true" : "false", 2, true);
        last_calibrated_state = is_calibrated;
    }

    // 6. Check and publish speed
    if (is_initial_publish || current_speed != last_speed) {
        char speed_str[16];
        snprintf(speed_str, sizeof(speed_str), "%.2f", current_speed);
        publish_state_value("speed", speed_str, 1, true);
        last_speed = current_speed;
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

    bool is_moving = (new_state == MOTOR_STATE_MOVING || new_state == MOTOR_STATE_CALIBRATING);

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
            if (strlen(mqtt_set_topic) > 0) {
                msg_id = esp_mqtt_client_subscribe(client, mqtt_set_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", mqtt_set_topic, msg_id);
            }
            if (strlen(mqtt_set_position_topic) > 0) {
                msg_id = esp_mqtt_client_subscribe(client, mqtt_set_position_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", mqtt_set_position_topic, msg_id);
            }
            
            // Reset last known states to force a full update on connection
            last_position = -1;
            last_state = -1;
            last_direction = -1;
            last_speed = -1.0f;
            // last_calibrated_state does not need reset, it will be published if different

            // Publish initial status on connection
            check_and_publish_status_updates(2); // QoS 2 for initial position

            // Publish Home Assistant discovery message
            publish_ha_discovery_message();
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

            // Null-terminate the topic and data to treat them as C strings
            char* topic_str = strndup(event->topic, event->topic_len);
            char* data_str = strndup(event->data, event->data_len);
            if (!topic_str || !data_str) {
                ESP_LOGE(TAG, "Failed to allocate memory for topic or data string");
                free(topic_str);
                free(data_str);
                break;
            }

            if (strcmp(topic_str, mqtt_set_position_topic) == 0) {
                char *endptr;
                long percentage = strtol(data_str, &endptr, 10);

                // Check if the conversion was successful and the entire string was a valid number
                if (endptr != data_str && *endptr == '\0') {
                    // HA now sends a value where 100=open, 0=closed.
                    // We convert it back to our internal format (0=open, 100=closed).
                    motor_go_to_percentage(mqtt_motor_handle, 100 - (int)percentage, 0);
                } else {
                    ESP_LOGW(TAG, "Received invalid position value: %s", data_str);
                }
            } else if (strcmp(topic_str, mqtt_set_topic) == 0) {
                // Handle Home Assistant specific commands
                // The position received from HA's slider is now inverted (0-100, closed-open).
                // We need to convert it back to our internal representation (0-100, open-closed).
                if (strcmp(data_str, "OPEN") == 0) {
                    motor_go_to_percentage(mqtt_motor_handle, 0, 0); // Internal: 0% is open
                } else if (strcmp(data_str, "CLOSE") == 0) {
                    motor_go_to_percentage(mqtt_motor_handle, 100, 0); // Internal: 100% is closed
                } else if (strcmp(data_str, "STOP") == 0) {
                    motor_stop(mqtt_motor_handle);
                }
            } else if (strcmp(topic_str, mqtt_command_topic) == 0) {
                // Handle generic commands and publish a response
                char response_buffer[256] = {0};
                cmd_handle_command_string(data_str, mqtt_motor_handle, response_buffer, sizeof(response_buffer));
                if (strlen(response_buffer) > 0) {
                    esp_mqtt_client_publish(client, mqtt_response_topic, response_buffer, 0, 0, false);
                    ESP_LOGI(TAG, "Published response to %s: %s", mqtt_response_topic, response_buffer);
                }
            }

            free(topic_str);
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
    // HA-compliant topics
    snprintf(mqtt_set_topic, sizeof(mqtt_set_topic), "%s/set", mqtt_base_topic);
    snprintf(mqtt_set_position_topic, sizeof(mqtt_set_position_topic), "%s/set_position", mqtt_base_topic);
    // Custom topics for other controls
    snprintf(mqtt_command_topic, sizeof(mqtt_command_topic), "%s/command", mqtt_base_topic);
    snprintf(mqtt_response_topic, sizeof(mqtt_response_topic), "%s/command/response", mqtt_base_topic);

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
