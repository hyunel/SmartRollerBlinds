#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "sdkconfig.h"
#include "motor.h"
#include "button.h"

#define TAG "BUTTON"

static motor_handle_t s_motor_handle;
static int32_t g_preset_pos = 0;
static const char* NVS_KEY_PRESET = "preset_pos";

static void save_preset_position() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(nvs_handle, NVS_KEY_PRESET, g_preset_pos);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) writing to NVS!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Successfully saved preset position: %" PRIi32, g_preset_pos);
    }

    err = nvs_commit(nvs_handle);
     if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed!");
    }

    nvs_close(nvs_handle);
}

static void load_preset_position() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error (%s) opening NVS handle for reading!", esp_err_to_name(err));
        return;
    }

    err = nvs_get_i32(nvs_handle, NVS_KEY_PRESET, &g_preset_pos);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Successfully loaded preset position: %" PRIi32, g_preset_pos);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "Preset position not found in NVS. Initializing to 0.");
            g_preset_pos = 0; // Default value
            break;
        default :
            ESP_LOGE(TAG, "Error (%s) reading NVS!", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}


// --- Button Callbacks ---

static void button_up_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Long Press - Starting Calibration");
    if (s_motor_handle) {
        motor_calibrate(s_motor_handle, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_up_single_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Single Click - Go to Zero");
    if (s_motor_handle) {
        motor_go_to_zero(s_motor_handle, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_up_double_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "UP BUTTON: Double Click - Move Up 1000 steps");
    if (s_motor_handle) {
        int32_t current_pos = motor_get_position(s_motor_handle);
        motor_set_target(s_motor_handle, current_pos - 1000, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_down_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Long Press - Set Preset Position");
    if (s_motor_handle) {
        g_preset_pos = motor_get_position(s_motor_handle);
        ESP_LOGI(TAG, "New preset position set to %" PRIi32, g_preset_pos);
        save_preset_position(); // Save to NVS
    }
}

static void button_down_single_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Single Click - Move to Preset Position");
    if (s_motor_handle) {
        motor_set_target(s_motor_handle, g_preset_pos, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

static void button_down_double_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "DOWN BUTTON: Double Click - Move Down 1000 steps");
    if (s_motor_handle) {
        int32_t current_pos = motor_get_position(s_motor_handle);
        motor_set_target(s_motor_handle, current_pos + 1000, CONFIG_DEFAULT_MOVE_SPEED);
    }
}

// --- Button Initialization ---
void button_module_init(motor_handle_t motor_handle)
{
    s_motor_handle = motor_handle;
    load_preset_position();

    button_handle_t btn_up_handle, btn_down_handle;

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
