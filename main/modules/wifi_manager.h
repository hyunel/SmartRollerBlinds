#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "modules/motor.h"
#include "esp_netif.h"

extern esp_netif_t *ap_netif;

/**
 * @brief Initializes the WiFi manager.
 *
 * This function initializes Wi-Fi, attempts to connect to a previously configured
 * network, and if that fails, starts a configuration access point.
 *
 * @param motor_handle Handle to the motor instance, to be passed to other modules.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_manager_init(motor_handle_t motor_handle);

/**
 * @brief Clears the stored Wi-Fi configuration from NVS and reboots the device.
 *
 * This function is typically called when a user wants to reset the Wi-Fi settings,
 * for example, by long-pressing a button.
 */
void wifi_manager_clear_config_and_reboot(void);

#endif // WIFI_MANAGER_H
