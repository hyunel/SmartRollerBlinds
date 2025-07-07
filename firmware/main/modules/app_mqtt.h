#ifndef APP_MQTT_H
#define APP_MQTT_H

#include "esp_err.h"
#include "modules/motor.h"

/**
 * @brief Starts the MQTT client.
 *
 * Reads configuration from NVS, connects to the MQTT broker, and starts
 * the tasks for publishing status and handling incoming commands.
 *
 * @param motor_handle Handle to the motor instance for status and control.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_mqtt_start(motor_handle_t motor_handle);

/**
 * @brief Stops the MQTT client.
 *
 * Disconnects from the broker and stops the related tasks.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_mqtt_stop(void);

#endif // APP_MQTT_H
