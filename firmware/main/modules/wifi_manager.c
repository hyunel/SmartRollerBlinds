#include "wifi_manager.h"
#include "web_server.h"
#include "app_mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdlib.h>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "WIFI_MANAGER";

esp_netif_t *ap_netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
static motor_handle_t s_motor_handle = NULL;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from AP, attempting to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

static void start_web_config_mode(void) {
    ESP_LOGI(TAG, "Starting AP mode for web configuration.");

    ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    
    uint8_t mac[6];
    char ap_ssid[32];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
    snprintf(ap_ssid, sizeof(ap_ssid), CONFIG_DEVICE_NAME "-%02X%02X", mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ap_ssid),
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        },
    };
    strcpy((char *)wifi_config.ap.ssid, ap_ssid);

    if (strlen((char*)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started. SSID:%s", ap_ssid);

    web_server_start();
}

void wifi_manager_clear_config_and_reboot(void) {
    ESP_LOGI(TAG, "Clearing Wi-Fi configuration and rebooting.");
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_restart();
}

esp_err_t wifi_manager_init(motor_handle_t motor_handle) {
    s_motor_handle = motor_handle;
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t stored_config;
    if (esp_wifi_get_config(WIFI_IF_STA, &stored_config) == ESP_OK && strlen((const char*)stored_config.sta.ssid)) {
        ESP_LOGI(TAG, "Found stored credentials. Connecting to %s", stored_config.sta.ssid);
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &stored_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_connect();

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                WIFI_CONNECTED_BIT,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Successfully connected to AP");
            app_mqtt_start(s_motor_handle);
        }
    } else {
        ESP_LOGI(TAG, "No stored credentials found. Starting configuration mode.");
        start_web_config_mode();
    }

    return ESP_OK;
}
