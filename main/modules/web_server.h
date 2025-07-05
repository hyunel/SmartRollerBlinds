#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

/**
 * @brief Starts the web server for configuration.
 *
 * This function initializes and starts the HTTP server, which provides a web
 * interface for configuring Wi-Fi and MQTT settings.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t web_server_start(void);

/**
 * @brief Stops the web server.
 *
 * This function stops the HTTP server.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t web_server_stop(void);

#endif // WEB_SERVER_H
