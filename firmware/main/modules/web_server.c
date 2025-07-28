#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <string.h>
#include <sys/param.h>
#include "web_server_html.h"

static const char *TAG = "WEB_SERVER";

// Helper function to convert a hex character to its integer value
static int hex_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

// Helper function to decode a URL-encoded string in place
static void url_decode(char *str) {
    char *p = str;
    char *q = str;
    while (*p) {
        if (*p == '%' && p[1] && p[2]) {
            *q++ = (char)(hex_to_int(p[1]) * 16 + hex_to_int(p[2]));
            p += 3;
        } else if (*p == '+') {
            *q++ = ' ';
            p++;
        } else {
            *q++ = *p++;
        }
    }
    *q = '\0';
}
static httpd_handle_t server = NULL;

// HTTP GET Handler for the root page
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, config_page_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler to redirect captive portal checks to the root page
static esp_err_t redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS and other systems require content in the response to detect a captive portal.
    const char* resp_str = "Redirecting to captive portal";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t wifi_scan_handler(httpd_req_t *req) {
    uint16_t number = 20;
    wifi_ap_record_t ap_info[number];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    // Use a specific scan configuration
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };

    ESP_LOGI(TAG, "Starting WiFi scan...");
    
    if (esp_wifi_scan_start(&scan_config, true) == ESP_OK) {
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
        ESP_LOGI(TAG, "Total APs scanned: %u", ap_count);
    } else {
        ESP_LOGE(TAG, "WiFi scan failed to start.");
    }

    httpd_resp_set_type(req, "application/json");
    char* json_buf = malloc(2048); 
    if (json_buf == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *p = json_buf;
    p += sprintf(p, "[");
    for (int i = 0; (i < number) && (i < ap_count); i++) {
        if (i > 0) {
            p += sprintf(p, ",");
        }
        char escaped_ssid[65];
        const char *ssid_ptr = (const char *)ap_info[i].ssid;
        char *esc_ptr = escaped_ssid;
        while(*ssid_ptr) {
            if (*ssid_ptr == '"' || *ssid_ptr == '\\') {
                *esc_ptr++ = '\\';
            }
            *esc_ptr++ = *ssid_ptr++;
        }
        *esc_ptr = '\0';

        p += sprintf(p, "{\"ssid\":\"%s\",\"rssi\":%d,\"authmode\":%d}", escaped_ssid, ap_info[i].rssi, ap_info[i].authmode);
    }
    p += sprintf(p, "]");
    
    httpd_resp_send(req, json_buf, strlen(json_buf));
    free(json_buf);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[512];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) -1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    char ssid[32] = {0};
    char pass[64] = {0};
    char mqtt_server[64] = {0};
    char mqtt_port[6] = {0};
    char mqtt_user[32] = {0};
    char mqtt_pass[64] = {0};
    char mqtt_base_topic[64] = {0};

    httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid));
    httpd_query_key_value(buf, "pass", pass, sizeof(pass));
    httpd_query_key_value(buf, "mqtt_server", mqtt_server, sizeof(mqtt_server));
    httpd_query_key_value(buf, "mqtt_port", mqtt_port, sizeof(mqtt_port));
    httpd_query_key_value(buf, "mqtt_user", mqtt_user, sizeof(mqtt_user));
    httpd_query_key_value(buf, "mqtt_pass", mqtt_pass, sizeof(mqtt_pass));
    httpd_query_key_value(buf, "mqtt_base_topic", mqtt_base_topic, sizeof(mqtt_base_topic));

    // Decode URL-encoded strings
    url_decode(ssid);
    url_decode(pass);
    url_decode(mqtt_server);
    url_decode(mqtt_user);
    url_decode(mqtt_pass);
    url_decode(mqtt_base_topic);

    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_server", mqtt_server));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_port", mqtt_port));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_user", mqtt_user));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_pass", mqtt_pass));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_base_topic", mqtt_base_topic));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, pass);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_LOGI(TAG, "Saved Wi-Fi credentials. SSID: %s", ssid);
    ESP_LOGI(TAG, "Saved MQTT config. Server: %s", mqtt_server);

    const char* resp_str = "<h1>Configuration Saved!</h1><p>The device will now reboot and try to connect to the new network.</p>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

esp_err_t web_server_start(void) {
    if (server) {
        return ESP_OK;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 16; // Increase the number of URI handlers

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Main page
        const httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);

        // API endpoints
        const httpd_uri_t wifi_scan_uri = { .uri = "/api/scanwifi", .method = HTTP_GET, .handler = wifi_scan_handler };
        httpd_register_uri_handler(server, &wifi_scan_uri);
        const httpd_uri_t save_uri = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(server, &save_uri);

        // Captive portal detection URIs
        const httpd_uri_t redirect_uri = { .method = HTTP_GET, .handler = redirect_handler };
        const char* redirect_uris[] = {
            "/generate_204", "/gen_204", "/hotspot-detect.html",
            "/mobile/status.php", "/library/test/success.html", "/success.html",
            "/ncsi.txt", "/check_network_status.txt"
        };
        for (size_t i = 0; i < sizeof(redirect_uris) / sizeof(redirect_uris[0]); i++) {
            httpd_uri_t temp_uri = redirect_uri;
            temp_uri.uri = redirect_uris[i];
            httpd_register_uri_handler(server, &temp_uri);
        }
        
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return ESP_FAIL;
}

esp_err_t web_server_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}
