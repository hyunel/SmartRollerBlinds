#include "led_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <math.h>
#include "esp_event.h"
#include "esp_wifi.h"
#include "motor.h"

#define TAG "LED_CONTROL"

// LEDC configuration for the status LED
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // 13-bit resolution (0-8191) for smooth breathing
#define LEDC_FREQUENCY          (5000) // 5 kHz frequency

#define BREATHING_PERIOD_MS     3000 // Total time for one breath (in and out)
#define BREATHING_STEPS         100  // Number of steps for the animation

static led_state_t g_current_led_state = LED_STATE_OFF;
static SemaphoreHandle_t g_state_mutex;
static TaskHandle_t g_led_task_handle = NULL;

static void status_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START || event_id == WIFI_EVENT_STA_DISCONNECTED) {
            // Wi-Fi Connecting State
            led_control_set_state(LED_STATE_BREATHING);
        } else if (event_id == WIFI_EVENT_AP_START) {
            // Wi-Fi Provisioning (AP Mode)
            led_control_set_state(LED_STATE_BLINK_FAST);
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            // Wi-Fi Connected and Idle
            led_control_set_state(LED_STATE_ON);
        }
    } else if (event_base == MOTOR_EVENTS) {
        if (event_id == MOTOR_EVENT_STATE_CHANGED) {
            motor_state_t new_state = *(motor_state_t*)event_data;
            switch (new_state) {
                case MOTOR_STATE_MOVING:
                case MOTOR_STATE_CALIBRATING:
                    // Motor is running
                    led_control_set_state(LED_STATE_BLINK_SLOW);
                    break;
                case MOTOR_STATE_IDLE:
                    // Motor is idle, revert to Wi-Fi status
                    {
                        wifi_ap_record_t ap_info;
                        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                            // Connected and Idle
                            led_control_set_state(LED_STATE_ON);
                        } else {
                            // Disconnected and Idle
                            led_control_set_state(LED_STATE_BREATHING);
                        }
                    }
                    break;
                case MOTOR_STATE_ERROR:
                    // Motor Error
                    led_control_set_state(LED_STATE_BLINK_FAST);
                    break;
            }
        }
    }
}

static void led_control_task(void *pvParameters);

void led_control_init(void) {
    // 1. Configure LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configure LEDC Channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CONFIG_PIN_LED,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 3. Create Mutex and Task
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 5, &g_led_task_handle);
    ESP_LOGI(TAG, "LED control task started");

    // Register the event handler to listen for system-wide status changes
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &status_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &status_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MOTOR_EVENTS, ESP_EVENT_ANY_ID, &status_event_handler, NULL));
}

void led_control_set_state(led_state_t new_state) {
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_current_led_state != new_state) {
            g_current_led_state = new_state;
            // Notify the task that the state has changed
            if (g_led_task_handle != NULL) {
                xTaskNotifyGive(g_led_task_handle);
            }
        }
        xSemaphoreGive(g_state_mutex);
    }
}

static void set_led_duty(uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void led_control_task(void *pvParameters) {
    led_state_t local_state = LED_STATE_OFF;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    TickType_t wait_time = portMAX_DELAY;

    while (1) {
        // Wait for a notification, but with a timeout for blinking states
        if (ulTaskNotifyTake(pdTRUE, wait_time)) {
            // If we got a notification, a state change occurred.
            // Fetch the new state.
            if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
                local_state = g_current_led_state;
                xSemaphoreGive(g_state_mutex);
            }
        }

        // Reset wait time for the next loop iteration
        wait_time = portMAX_DELAY;

        switch (local_state) {
            case LED_STATE_OFF:
                set_led_duty(0);
                break;

            case LED_STATE_ON:
                set_led_duty(max_duty);
                break;

            case LED_STATE_BLINK_SLOW:
                // Toggle state
                set_led_duty(ledc_get_duty(LEDC_MODE, LEDC_CHANNEL) == 0 ? max_duty : 0);
                wait_time = pdMS_TO_TICKS(500); // Set timeout for next toggle
                break;

            case LED_STATE_BLINK_FAST:
                // Toggle state
                set_led_duty(ledc_get_duty(LEDC_MODE, LEDC_CHANNEL) == 0 ? max_duty : 0);
                wait_time = pdMS_TO_TICKS(100); // Set timeout for next toggle
                break;

            case LED_STATE_BREATHING:
                // This state is more complex and the original logic was fine for it,
                // but let's adapt it to the new single-loop model.
                // Fade in
                for (int i = 0; i <= BREATHING_STEPS; i++) {
                    if(ulTaskNotifyTake(pdTRUE, 0)) { // Check for state change during animation
                        if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
                            local_state = g_current_led_state;
                            xSemaphoreGive(g_state_mutex);
                        }
                        goto next_cycle; // Break out of the for loop and switch
                    }
                    float rad = (float)i / BREATHING_STEPS * M_PI;
                    float multiplier = (1 - cos(rad)) / 2.0f;
                    set_led_duty((uint32_t)(max_duty * multiplier));
                    vTaskDelay(pdMS_TO_TICKS(BREATHING_PERIOD_MS / 2 / BREATHING_STEPS));
                }
                // Fade out
                for (int i = 0; i <= BREATHING_STEPS; i++) {
                    if(ulTaskNotifyTake(pdTRUE, 0)) { // Check for state change during animation
                         if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
                            local_state = g_current_led_state;
                            xSemaphoreGive(g_state_mutex);
                        }
                        goto next_cycle; // Break out of the for loop and switch
                    }
                    float rad = M_PI + (float)i / BREATHING_STEPS * M_PI;
                    float multiplier = (1 - cos(rad)) / 2.0f;
                    set_led_duty((uint32_t)(max_duty * multiplier));
                    vTaskDelay(pdMS_TO_TICKS(BREATHING_PERIOD_MS / 2 / BREATHING_STEPS));
                }
                break;
        }
        next_cycle:; // Label to jump to for the next iteration
    }
}
