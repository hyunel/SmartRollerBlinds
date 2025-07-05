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
#include "sdkconfig.h"
#include "esp_wifi.h"

#define TAG "MAIN"
#define UART_NUM UART_NUM_0
#define UART_BUF_SIZE (1024)
#define TASK_STACK_SIZE 2048

static motor_handle_t main_motor_handle;

static void process_command(char *cmd_line) {
    char *cmd;
    char *saveptr;

    // Get the command which is the first word
    cmd = strtok_r(cmd_line, " \n\r", &saveptr);

    if (cmd == NULL) {
        return; // Empty line
    }

    if (strcmp(cmd, "help") == 0) {
        printf("Available commands:\n");
        printf("  help - Show this message\n");
        printf("  status - Get current motor status\n");
        printf("  calibrate - Start motor calibration\n");
        printf("  zero - Go to zero position to re-sync\n");
        printf("  move <position> [speed] - Move motor to a specific position\n");
        printf("  set_pid <kp> <ki> <kd> - Set PID gains\n");
    } else if (strcmp(cmd, "status") == 0) {
        if (main_motor_handle) {
            int32_t pos = motor_get_position(main_motor_handle);
            int32_t target_pos = motor_get_target_position(main_motor_handle);
            motor_state_t state = motor_get_state(main_motor_handle);
            bool calibrated = motor_is_calibrated(main_motor_handle);
            const char* state_str = "Unknown";
            switch(state) {
                case MOTOR_STATE_IDLE: state_str = "Idle"; break;
                case MOTOR_STATE_MOVING: state_str = "Moving"; break;
                case MOTOR_STATE_CALIBRATING: state_str = "Calibrating"; break;
                case MOTOR_STATE_HOMING: state_str = "Homing"; break;
                case MOTOR_STATE_ERROR: state_str = "Error"; break;
            }
            // Log with ESP_LOGI for standardized output format
            printf("Pos:%" PRIi32 " -> Tar:%" PRIi32 ", State:%s, Calib:%s\n", pos, target_pos, state_str, calibrated ? "Y" : "N");
        } else {
            ESP_LOGE(TAG, "Motor not initialized!");
        }
    } else if (strcmp(cmd, "calibrate") == 0) {
        if (main_motor_handle) {
            motor_calibrate(main_motor_handle, 100.0f);
            printf("Calibration command sent.\n");
        } else {
            ESP_LOGE(TAG, "Motor not initialized!");
        }
    } else if (strcmp(cmd, "zero") == 0) {
        if (main_motor_handle) {
            motor_go_to_zero(main_motor_handle, 100.0f);
            printf("Go to zero command sent.\n");
        } else {
            ESP_LOGE(TAG, "Motor not initialized!");
        }
    } else if (strcmp(cmd, "move") == 0) {
        char *pos_str = strtok_r(NULL, " \n\r", &saveptr);
        char *speed_str = strtok_r(NULL, " \n\r", &saveptr); // Optional speed
        if (pos_str) {
            int32_t position = atoi(pos_str);
            float speed = CONFIG_DEFAULT_MOVE_SPEED;
            if (speed_str) {
                speed = atof(speed_str);
            }
            if (main_motor_handle) {
                motor_set_target(main_motor_handle, position, speed);
                printf("Move command sent to pos %" PRIi32 " with speed %.2f.\n", position, speed);
            } else {
                ESP_LOGE(TAG, "Motor not initialized!");
            }
        } else {
            printf("Error: 'move' command requires a position argument.\n");
        }
    } else if (strcmp(cmd, "set_pid") == 0) {
        char *kp_str = strtok_r(NULL, " \n\r", &saveptr);
        char *ki_str = strtok_r(NULL, " \n\r", &saveptr);
        char *kd_str = strtok_r(NULL, " \n\r", &saveptr);

        if (kp_str && ki_str && kd_str) {
            float kp = atof(kp_str);
            float ki = atof(ki_str);
            float kd = atof(kd_str);
            if (main_motor_handle) {
                motor_update_pid_params(main_motor_handle, kp, ki, kd);
                printf("PID params updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
            } else {
                ESP_LOGE(TAG, "Motor not initialized!");
            }
        } else {
            printf("Error: 'set_pid' requires kp, ki, and kd arguments.\n");
        }
    } else {
        printf("Unknown command: %s\n", cmd);
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
