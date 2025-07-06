#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "sdkconfig.h"
#include "motor.h"
#include "button.h"
#include "wifi_manager.h"

#define TAG "BUTTON"

static motor_handle_t s_motor_handle;


// --- Button Callbacks ---

static void button_up_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Long Press - Start moving up for calibration.");
    if (s_motor_handle) {
        motor_start_calibration(s_motor_handle, true, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_up_single_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Single Click");
    if (s_motor_handle) {
        motor_state_t current_state = motor_get_state(s_motor_handle);
        motor_direction_t current_direction = motor_get_direction(s_motor_handle);

        if (current_direction == MOTOR_DIRECTION_UP) {
            ESP_LOGI(TAG, "Motor is already moving up. No action.");
            return;
        }

        if (current_state == MOTOR_STATE_MOVING) {
            motor_stop(s_motor_handle);
        } else if (current_state == MOTOR_STATE_CALIBRATING) {
            motor_stop(s_motor_handle);
        } else {
            ESP_LOGI(TAG, "Go to Zero");
            motor_set_target(s_motor_handle, 0, 0);
        }
    }
}

static void button_up_double_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Double Click - Move Up %d steps", CONFIG_BUTTON_DOUBLE_CLICK_STEPS);
    if (s_motor_handle) {
        motor_move_relative(s_motor_handle, -CONFIG_BUTTON_DOUBLE_CLICK_STEPS, 0);
    }
}

static void button_down_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Long Press - Start moving down for calibration.");
    if (s_motor_handle) {
        motor_start_calibration(s_motor_handle, false, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_down_single_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Single Click");
    if (s_motor_handle) {
        motor_state_t current_state = motor_get_state(s_motor_handle);
        motor_direction_t current_direction = motor_get_direction(s_motor_handle);

        if (current_direction == MOTOR_DIRECTION_DOWN) {
            ESP_LOGI(TAG, "Motor is already moving down. No action.");
            return;
        }

        if (current_state == MOTOR_STATE_MOVING) {
            motor_stop(s_motor_handle);
        } else if (current_state == MOTOR_STATE_CALIBRATING) {
            motor_stop(s_motor_handle);
        } else {
            ESP_LOGI(TAG, "Move to 100%% (fully closed)");
            motor_go_to_percentage(s_motor_handle, 100, 0);
        }
    }
}

static void button_down_double_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Double Click - Move Down %d steps", CONFIG_BUTTON_DOUBLE_CLICK_STEPS);
    if (s_motor_handle) {
        motor_move_relative(s_motor_handle, CONFIG_BUTTON_DOUBLE_CLICK_STEPS, 0);
    }
}

static void button_boot_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "BOOT BUTTON: Long Press - Clearing WiFi config and rebooting.");
    wifi_manager_clear_config_and_reboot();
}

// --- Button Initialization ---
void button_module_init(motor_handle_t motor_handle)
{
    s_motor_handle = motor_handle;

    button_handle_t btn_up_handle, btn_down_handle, btn_boot_handle;

    // Boot Button (for WiFi reset)
    button_config_t btn_boot_cfg = {
        .long_press_time = 5000, // 5 seconds for reset
        .short_press_time = 0, // Not used
    };
    button_gpio_config_t gpio_btn_boot_cfg = {
        .gpio_num = CONFIG_PIN_BUTTON_BOOT,
        .active_level = 0,
    };
    iot_button_new_gpio_device(&btn_boot_cfg, &gpio_btn_boot_cfg, &btn_boot_handle);
    if (btn_boot_handle) {
        ESP_LOGI(TAG, "Boot button initialized on GPIO %d", CONFIG_PIN_BUTTON_BOOT);
        iot_button_register_cb(btn_boot_handle, BUTTON_LONG_PRESS_START, NULL, button_boot_long_press_cb, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to initialize BOOT button");
    }

    // Button Up
    button_config_t btn_up_cfg = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };
    button_gpio_config_t gpio_btn_up_cfg = {
        .gpio_num = CONFIG_PIN_BUTTON_UP,
        .active_level = 0,
    };
    iot_button_new_gpio_device(&btn_up_cfg, &gpio_btn_up_cfg, &btn_up_handle);
    if (btn_up_handle) {
        ESP_LOGI(TAG, "Button UP initialized on GPIO %d", CONFIG_PIN_BUTTON_UP);
        iot_button_register_cb(btn_up_handle, BUTTON_LONG_PRESS_START, NULL, button_up_long_press_cb, NULL);
        iot_button_register_cb(btn_up_handle, BUTTON_SINGLE_CLICK, NULL, button_up_single_click_cb, NULL);
        iot_button_register_cb(btn_up_handle, BUTTON_DOUBLE_CLICK, NULL, button_up_double_click_cb, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to initialize UP button");
    }

    // Button Down
    button_config_t btn_down_cfg = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };
    button_gpio_config_t gpio_btn_down_cfg = {
        .gpio_num = CONFIG_PIN_BUTTON_DOWN,
        .active_level = 0,
    };
    iot_button_new_gpio_device(&btn_down_cfg, &gpio_btn_down_cfg, &btn_down_handle);
    if (btn_down_handle) {
        ESP_LOGI(TAG, "Button DOWN initialized on GPIO %d", CONFIG_PIN_BUTTON_DOWN);
        iot_button_register_cb(btn_down_handle, BUTTON_LONG_PRESS_START, NULL, button_down_long_press_cb, NULL);
        iot_button_register_cb(btn_down_handle, BUTTON_SINGLE_CLICK, NULL, button_down_single_click_cb, NULL);
        iot_button_register_cb(btn_down_handle, BUTTON_DOUBLE_CLICK, NULL, button_down_double_click_cb, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to initialize DOWN button");
    }
}
