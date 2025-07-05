#include "motor.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "pid_ctrl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"
#include <math.h>
#include <string.h>

#define TAG "MOTOR"

ESP_EVENT_DEFINE_BASE(MOTOR_EVENTS);

// --- Configuration Constants ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY          (5000)

#define PID_MAX_OUTPUT          100.0f
#define PID_MIN_OUTPUT         -100.0f
#define PID_CONTROL_PERIOD_MS   10

#define NVS_NAMESPACE           "motor"
#define NVS_POSITION_KEY        "position"
#define NVS_FULLY_CLOSED_KEY    "fully_closed"

#define MOTOR_CMD_QUEUE_LEN     10

// --- Type Definitions ---

typedef enum {
    CMD_SET_TARGET,
    CMD_CALIBRATE,
    CMD_GO_TO_ZERO,
} motor_command_type_t;

typedef struct {
    motor_command_type_t type;
    int32_t position;
    float max_speed_pps;
} motor_command_t;

// The main struct holding the state of a motor instance
struct motor_control_t {
    pid_ctrl_block_handle_t pid_controller;
    pid_ctrl_parameter_t pid_params;
    QueueHandle_t cmd_queue;
    nvs_handle_t nvs_handle;

    // Volatile state updated by ISR or multiple tasks
    volatile int32_t current_position;
    volatile int8_t last_hall_state;

    // State controlled by the PID task
    int32_t target_position;
    int32_t fully_closed_position;
    float max_speed_pps;
    motor_state_t current_state;
    bool is_calibrated;
};

// --- Forward Declarations ---
static void motor_pid_control_task(void *pvParameters);
static esp_err_t motor_set_pwm_duty(float duty_percent);
static void IRAM_ATTR hall_encoder_isr_handler(void* arg);
static void process_command(motor_handle_t handle, const motor_command_t *cmd);
static void motor_set_state(motor_handle_t handle, motor_state_t new_state);
static esp_err_t motor_save_nvs_value(const char* key, int32_t value);
static esp_err_t motor_load_nvs_value(const char* key, int32_t* value);

// --- Public API Implementation ---

esp_err_t motor_init(motor_handle_t *motor_handle) {
    ESP_LOGI(TAG, "Initializing new motor instance...");

    // Allocate memory for the motor control struct
    motor_handle_t handle = (motor_handle_t)calloc(1, sizeof(struct motor_control_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for motor handle");
        return ESP_ERR_NO_MEM;
    }

    // Hardware Init: LEDC PWM (assuming single motor for now, could be parameterized)
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER, .duty_resolution = LEDC_DUTY_RES, .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel_a = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_A, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = CONFIG_PIN_MOTOR_A, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));
    ledc_channel_config_t ledc_channel_b = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_B, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = CONFIG_PIN_MOTOR_B, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));

    // Hardware Init: Hall Sensor GPIO Interrupts
    gpio_config_t io_conf = { .pin_bit_mask = (1ULL << CONFIG_PIN_MOTOR_HA) | (1ULL << CONFIG_PIN_MOTOR_HB), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // Pass the handle to the ISR
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_PIN_MOTOR_HA, hall_encoder_isr_handler, handle));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_PIN_MOTOR_HB, hall_encoder_isr_handler, handle));

    // Software Init: PID Controller
    handle->pid_params = (pid_ctrl_parameter_t) {
        .kp = atof(CONFIG_MOTOR_PID_KP),
        .ki = atof(CONFIG_MOTOR_PID_KI),
        .kd = atof(CONFIG_MOTOR_PID_KD),
        .min_output = PID_MIN_OUTPUT,
        .max_output = PID_MAX_OUTPUT,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
    };
    pid_ctrl_config_t pid_config = { .init_param = handle->pid_params };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &handle->pid_controller));

    // Software Init: Command Queue
    handle->cmd_queue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(motor_command_t));
    if (!handle->cmd_queue) {
        ESP_LOGE(TAG, "Failed to create command queue");
        free(handle);
        return ESP_ERR_NO_MEM;
    }

    // State Init: Load from NVS
    if (motor_load_nvs_value(NVS_POSITION_KEY, (int32_t*)&handle->current_position) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded position %" PRId32 " from NVS.", handle->current_position);
        handle->target_position = handle->current_position;
        handle->is_calibrated = true; // Assume calibrated if position is saved
    } else {
        ESP_LOGW(TAG, "Could not load position from NVS. Motor needs calibration.");
        handle->is_calibrated = false;
    }

    if (motor_load_nvs_value(NVS_FULLY_CLOSED_KEY, &handle->fully_closed_position) != ESP_OK) {
        handle->fully_closed_position = 0; // Default to 0 if not found
        ESP_LOGW(TAG, "Fully closed position not set. Percentage moves will not work.");
    } else {
        ESP_LOGI(TAG, "Loaded fully closed position %" PRId32 " from NVS.", handle->fully_closed_position);
    }

    motor_set_state(handle, MOTOR_STATE_IDLE);

    // Create and start the control task, passing the handle as the parameter
    xTaskCreate(motor_pid_control_task, "motor_pid_task", 4096, handle, 5, NULL);
    
    *motor_handle = handle;
    ESP_LOGI(TAG, "Motor instance initialized successfully.");
    return ESP_OK;
}

esp_err_t motor_calibrate(motor_handle_t handle, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_command_t cmd = {
        .type = CMD_CALIBRATE,
        .max_speed_pps = max_speed_pps
    };
    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send calibrate command to queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t motor_go_to_zero(motor_handle_t handle, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_command_t cmd = {
        .type = CMD_GO_TO_ZERO,
        .max_speed_pps = max_speed_pps
    };
    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send go_to_zero command to queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t motor_set_target(motor_handle_t handle, int32_t position, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_command_t cmd = {
        .type = CMD_SET_TARGET,
        .position = position,
        .max_speed_pps = max_speed_pps
    };
    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send set_target command to queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

int32_t motor_get_position(motor_handle_t handle) {
    return handle ? handle->current_position : 0;
}

int32_t motor_get_target_position(motor_handle_t handle) {
    return handle ? handle->target_position : 0;
}

motor_state_t motor_get_state(motor_handle_t handle) {
    return handle ? handle->current_state : MOTOR_STATE_ERROR;
}

bool motor_is_calibrated(motor_handle_t handle) {
    return handle ? handle->is_calibrated : false;
}

esp_err_t motor_update_pid_params(motor_handle_t handle, float kp, float ki, float kd) {
    if (!handle || !handle->pid_controller) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->pid_params.kp = kp;
    handle->pid_params.ki = ki;
    handle->pid_params.kd = kd;
    // Note: pid_update_parameters is the correct function from pid_ctrl.h
    esp_err_t err = pid_update_parameters(handle->pid_controller, &handle->pid_params);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Updated PID gains to: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
    } else {
        ESP_LOGE(TAG, "Failed to update PID parameters");
    }
    return err;
}

// --- Private Helper Functions ---

static void motor_pid_control_task(void *pvParameters) {
    motor_handle_t handle = (motor_handle_t)pvParameters;
    motor_command_t received_cmd;

    int64_t move_start_time_ms = 0;

    while (1) {
        // 1. Check for incoming commands (non-blocking)
        if (xQueueReceive(handle->cmd_queue, &received_cmd, 0) == pdPASS) {
            process_command(handle, &received_cmd);
            move_start_time_ms = 0; // Reset timer on new command
        }

        // 2. Update state machine
        int64_t current_time_ms = esp_log_timestamp();
        switch (handle->current_state) {
            case MOTOR_STATE_IDLE:
                break; // PID loop below will hold position

            case MOTOR_STATE_MOVING:
            case MOTOR_STATE_CALIBRATING:
            case MOTOR_STATE_HOMING:
                if (move_start_time_ms == 0) move_start_time_ms = current_time_ms;

                // Timeout check
                if (current_time_ms - move_start_time_ms > CONFIG_PID_FAILURE_TIMEOUT_MS) {
                    ESP_LOGE(TAG, "PID failure: Timed out in state %d", handle->current_state);
                    motor_set_state(handle, MOTOR_STATE_ERROR);
                    motor_set_pwm_duty(0);
                    continue; // Skip PID calculation for this cycle
                }

                // Check for target reached
                if (abs(handle->target_position - handle->current_position) <= CONFIG_POSITION_DEADBAND) {
                    ESP_LOGI(TAG, "Movement finished. Final position: %" PRId32, handle->current_position);
                    if (handle->current_state == MOTOR_STATE_CALIBRATING) {
                        ESP_LOGI(TAG, "Calibration finished. Setting zero point.");
                        handle->current_position = 0;
                        handle->target_position = 0;
                        handle->is_calibrated = true;
                    } else if (handle->current_state == MOTOR_STATE_HOMING) {
                        ESP_LOGI(TAG, "Homing finished. Resetting zero point.");
                        handle->current_position = 0;
                        handle->target_position = 0;
                    }
                    motor_set_state(handle, MOTOR_STATE_IDLE);
                    motor_save_nvs_value(NVS_POSITION_KEY, handle->current_position);
                }
                break;

            case MOTOR_STATE_ERROR:
                vTaskDelay(pdMS_TO_TICKS(100));
                continue; // Skip PID calculation
        }

        // 3. Core PID Calculation
        float error = (float)(handle->target_position - handle->current_position);
        float pid_output;
        pid_compute(handle->pid_controller, error, &pid_output);

        // Apply deadband to prevent noise and oscillation when holding position.
        // If the motor is in IDLE state and the position error is within the deadband,
        // force the output to zero. This stops the motor from making small, noisy
        // adjustments when it's supposed to be stationary.
        if (handle->current_state == MOTOR_STATE_IDLE && fabsf(error) <= CONFIG_POSITION_DEADBAND) {
            pid_output = 0;
        }
        
        // 4. Apply motor power
        motor_set_pwm_duty(pid_output);
        
        vTaskDelay(pdMS_TO_TICKS(PID_CONTROL_PERIOD_MS));
    }
}

static void process_command(motor_handle_t handle, const motor_command_t *cmd) {
    switch (cmd->type) {
        case CMD_SET_TARGET:
            if (!handle->is_calibrated) {
                ESP_LOGE(TAG, "Cannot set target, motor not calibrated.");
                return;
            }
            if (handle->current_state == MOTOR_STATE_ERROR) {
                ESP_LOGE(TAG, "Cannot set target, motor is in an error state.");
                return;
            }

            int32_t new_target = cmd->position;
            // Constrain target to valid range if fully_closed_position is set
            if (handle->fully_closed_position > 0) {
                if (new_target < 0) new_target = 0;
                if (new_target > handle->fully_closed_position) new_target = handle->fully_closed_position;
            }


            ESP_LOGI(TAG, "CMD: Set target=%" PRId32 " (constrained to %" PRId32 "), speed=%.2f", cmd->position, new_target, cmd->max_speed_pps);

            // Update the speed limit in the PID controller
            handle->max_speed_pps = fabsf(cmd->max_speed_pps);
            handle->pid_params.max_output = handle->max_speed_pps;
            handle->pid_params.min_output = -handle->max_speed_pps;
            pid_update_parameters(handle->pid_controller, &handle->pid_params);

            pid_reset_ctrl_block(handle->pid_controller); // Reset PID before a new move
            handle->target_position = new_target;
            motor_set_state(handle, MOTOR_STATE_MOVING);
            break;

        case CMD_CALIBRATE:
            if (handle->current_state == MOTOR_STATE_ERROR) return;
            ESP_LOGI(TAG, "CMD: Calibrate, speed=%.2f", cmd->max_speed_pps);
            handle->is_calibrated = false;
            pid_reset_ctrl_block(handle->pid_controller);
            handle->max_speed_pps = fabsf(cmd->max_speed_pps);
            handle->pid_params.max_output = handle->max_speed_pps;
            handle->pid_params.min_output = -handle->max_speed_pps;
            pid_update_parameters(handle->pid_controller, &handle->pid_params);
            handle->target_position = handle->current_position - CONFIG_CALIBRATION_OVERRUN_PULSES; // Move up
            motor_set_state(handle, MOTOR_STATE_CALIBRATING);
            break;

        case CMD_GO_TO_ZERO:
            if (!handle->is_calibrated) {
                ESP_LOGE(TAG, "Cannot go to zero, motor not calibrated.");
                return;
            }
            if (handle->current_state == MOTOR_STATE_ERROR) return;
            ESP_LOGI(TAG, "CMD: Go to zero, speed=%.2f", cmd->max_speed_pps);
            pid_reset_ctrl_block(handle->pid_controller);
            handle->max_speed_pps = fabsf(cmd->max_speed_pps);
            handle->pid_params.max_output = handle->max_speed_pps;
            handle->pid_params.min_output = -handle->max_speed_pps;
            pid_update_parameters(handle->pid_controller, &handle->pid_params);
            handle->target_position = 0 - CONFIG_HOMING_OVERRUN_PULSES; // Move to zero plus overrun
            motor_set_state(handle, MOTOR_STATE_HOMING);
            break;
    }
}

static esp_err_t motor_set_pwm_duty(float duty_percent) {
    duty_percent = fmaxf(PID_MIN_OUTPUT, fminf(PID_MAX_OUTPUT, duty_percent));
    uint32_t duty_val = (uint32_t)(fabsf(duty_percent) / 100.0f * (1 << LEDC_DUTY_RES));
    
    if (duty_percent > 0) { // Move down
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty_val);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    } else { // Move up
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, duty_val);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    }
    return ESP_OK;
}

static void hall_encoder_isr_handler(void* arg) {
    motor_handle_t handle = (motor_handle_t)arg;
    uint8_t hall_a = gpio_get_level(CONFIG_PIN_MOTOR_HA);
    uint8_t hall_b = gpio_get_level(CONFIG_PIN_MOTOR_HB);
    uint8_t current_state = (hall_a << 1) | hall_b;
    static const int8_t QEM[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};
    int8_t transition = QEM[(handle->last_hall_state << 2) | current_state];
    
    if (transition == 1) {
        handle->current_position++;
    } else if (transition == -1) {
        handle->current_position--;
    }
    
    handle->last_hall_state = current_state;
}

static void motor_set_state(motor_handle_t handle, motor_state_t new_state) {
    if (handle->current_state != new_state) {
        handle->current_state = new_state;
        esp_event_post(MOTOR_EVENTS, MOTOR_EVENT_STATE_CHANGED, &new_state, sizeof(new_state), pdMS_TO_TICKS(10));
    }
}

static esp_err_t motor_save_nvs_value(const char* key, int32_t value) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for writing!", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_i32(nvs_handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS commit failed for key '%s'!", key);
        }
    } else {
        ESP_LOGE(TAG, "Error (%s) writing key '%s' to NVS!", esp_err_to_name(err), key);
    }
    nvs_close(nvs_handle);
    return err;
}

static esp_err_t motor_load_nvs_value(const char* key, int32_t* value) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        // This can happen on first boot, not necessarily an error.
        return err;
    }
    err = nvs_get_i32(nvs_handle, key, value);
    nvs_close(nvs_handle);
    return err;
}

// --- New Public API Functions ---

esp_err_t motor_set_fully_closed_position(motor_handle_t handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    handle->fully_closed_position = handle->current_position;
    ESP_LOGI(TAG, "New fully closed position set to %" PRId32, handle->fully_closed_position);
    return motor_save_nvs_value(NVS_FULLY_CLOSED_KEY, handle->fully_closed_position);
}

esp_err_t motor_go_to_percentage(motor_handle_t handle, uint8_t percentage, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (percentage > 100) percentage = 100;

    if (!handle->is_calibrated || handle->fully_closed_position <= 0) {
        ESP_LOGE(TAG, "Cannot go to percentage. Motor not calibrated or fully closed position not set.");
        return ESP_ERR_INVALID_STATE;
    }

    int32_t target_pos = (int32_t)(((float)percentage / 100.0f) * handle->fully_closed_position);
    ESP_LOGI(TAG, "Moving to %d%%, which is position %" PRId32, percentage, target_pos);

    return motor_set_target(handle, target_pos, max_speed_pps);
}
