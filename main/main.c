#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "modules/motor.h"
#include "modules/button.h"
#include "modules/led_control.h"
#include "modules/wifi_manager.h"
#include "modules/dns_server.h"
#include "modules/command_handler.h" // Add command handler
#include "sdkconfig.h"
#include "esp_wifi.h"

#define TAG "MAIN"
#define UART_NUM UART_NUM_0
#define UART_BUF_SIZE (1024)
#define TASK_STACK_SIZE 2048

static motor_handle_t main_motor_handle;

static void process_command(char *cmd_line) {
    if (cmd_line == NULL || strlen(cmd_line) == 0) {
        return;
    }
    
    // Pass all commands directly to the central handler.
    char response_buffer[512]; // Increased buffer size for help text
    cmd_handle_command_string(cmd_line, main_motor_handle, response_buffer, sizeof(response_buffer));
    
    if (strlen(response_buffer) > 0) {
        printf("%s\n", response_buffer);
    }
}

// --- UART Task ---

static void uart_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t* data = (uint8_t*) malloc(UART_BUF_SIZE);
    char line_buffer[256];
    int line_pos = 0;

    printf("\n--- Serial Console Initialized ---\n");
    printf("Type 'help' for a list of commands.\n");
    printf("esp32> ");
    fflush(stdout);


    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                if (c == '\r' || c == '\n') {
                    // Echo the newline
                    uart_write_bytes(UART_NUM, "\r\n", 2);

                    line_buffer[line_pos] = '\0'; // Null-terminate the string
                    if (line_pos > 0) {
                        process_command(line_buffer);
                    }
                    line_pos = 0; // Reset for next command
                    printf("esp32> ");
                    fflush(stdout);
                } else if (c == '\b' || c == 127) { // Handle backspace
                    if (line_pos > 0) {
                        line_pos--;
                        // Erase character on the terminal
                        uart_write_bytes(UART_NUM, "\b \b", 3);
                    }
                } else if (line_pos < (sizeof(line_buffer) - 1)) {
                    // Echo character
                    uart_write_bytes(UART_NUM, (const char*)&c, 1);
                    line_buffer[line_pos++] = c;
                }
            }
        }
    }
    free(data);
}


void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize Motor
    ESP_ERROR_CHECK(motor_init(&main_motor_handle));

    // Initialize Buttons
    button_module_init(main_motor_handle);

    // Initialize LED
    led_control_init();

    // Start the console task
    xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE * 2, NULL, 10, NULL);

    // Initialize WiFi Manager
    ESP_ERROR_CHECK(wifi_manager_init(main_motor_handle));

    // Start DNS server if in AP mode
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
        start_dns_server(&config);
    }
}
