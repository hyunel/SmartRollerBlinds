#include "motor.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
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
#define LEDC_FREQUENCY          (20000)

#define PID_MAX_OUTPUT          100.0f
#define PID_MIN_OUTPUT         -100.0f
#define PID_CONTROL_PERIOD_MS   10

#define NVS_NAMESPACE           "motor"
#define NVS_POSITION_KEY        "position"
#define NVS_FULLY_CLOSED_KEY    "fully_closed"
#define NVS_SPEED_KEY           "def_speed"

#define MOTOR_CMD_QUEUE_LEN     10

// --- Type Definitions ---

typedef enum {
    CMD_SET_TARGET,
    CMD_MOVE_RELATIVE,
    CMD_STOP,
    CMD_START_CALIBRATION_UP,
    CMD_START_CALIBRATION_DOWN,
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
    SemaphoreHandle_t lock;

    // Volatile state updated by ISR or multiple tasks
    volatile int32_t current_position;
    volatile int8_t last_hall_state;

    // State controlled by the PID task
    int32_t target_position;
    int32_t fully_closed_position;
    float default_speed_pps;
    motor_state_t current_state;
    motor_direction_t current_direction;
    bool is_calibrated;
};

// --- Forward Declarations ---
static void motor_pid_control_task(void *pvParameters);
static esp_err_t motor_set_pwm_duty(float duty_percent);
static void IRAM_ATTR hall_encoder_isr_handler(void* arg);
static void process_command(motor_handle_t handle, const motor_command_t *cmd);
static void motor_set_state(motor_handle_t handle, motor_state_t new_state);
static void motor_initiate_movement(motor_handle_t handle, int32_t target, float speed, motor_state_t state);

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

    // Software Init: Mutex for thread safety
    handle->lock = xSemaphoreCreateMutex();
    if (!handle->lock) {
        ESP_LOGE(TAG, "Failed to create mutex");
        vQueueDelete(handle->cmd_queue);
        free(handle);
        return ESP_ERR_NO_MEM;
    }

    // State Init: Load from NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        if (nvs_get_i32(nvs_handle, NVS_POSITION_KEY, (int32_t*)&handle->current_position) == ESP_OK) {
            ESP_LOGI(TAG, "Loaded position %" PRId32 " from NVS.", handle->current_position);
            handle->target_position = handle->current_position;
            handle->is_calibrated = true;
        } else {
            ESP_LOGW(TAG, "Could not load position from NVS. Motor needs calibration.");
            handle->is_calibrated = false;
        }

        if (nvs_get_i32(nvs_handle, NVS_FULLY_CLOSED_KEY, &handle->fully_closed_position) == ESP_OK) {
            ESP_LOGI(TAG, "Loaded fully closed position %" PRId32 " from NVS.", handle->fully_closed_position);
        } else {
            handle->fully_closed_position = 0;
            ESP_LOGW(TAG, "Fully closed position not set. Percentage moves will not work.");
        }

        char speed_str[16] = {0};
        size_t speed_len = sizeof(speed_str);
        if (nvs_get_str(nvs_handle, NVS_SPEED_KEY, speed_str, &speed_len) == ESP_OK) {
            handle->default_speed_pps = atof(speed_str);
            ESP_LOGI(TAG, "Loaded default speed %.2f from NVS.", handle->default_speed_pps);
        } else {
            handle->default_speed_pps = CONFIG_DEFAULT_MOVE_SPEED;
            ESP_LOGI(TAG, "Default speed not in NVS, using default %.2f.", handle->default_speed_pps);
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "Could not open NVS. Using default values.");
        handle->is_calibrated = false;
        handle->fully_closed_position = 0;
        handle->default_speed_pps = CONFIG_DEFAULT_MOVE_SPEED;
    }

    motor_set_state(handle, MOTOR_STATE_IDLE);

    // Create and start the control task, passing the handle as the parameter
    xTaskCreate(motor_pid_control_task, "motor_pid_task", 4096, handle, 5, NULL);
    
    *motor_handle = handle;
    ESP_LOGI(TAG, "Motor instance initialized successfully.");
    return ESP_OK;
}

esp_err_t motor_start_calibration(motor_handle_t handle, bool move_up, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    motor_command_t cmd = {
        .type = move_up ? CMD_START_CALIBRATION_UP : CMD_START_CALIBRATION_DOWN,
        .max_speed_pps = max_speed_pps
    };
    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send start_calibration command to queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t motor_stop(motor_handle_t handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;

    // If the motor is already stopped, do nothing.
    if (motor_get_state(handle) == MOTOR_STATE_IDLE) {
        ESP_LOGI(TAG, "Motor is already idle, nothing to stop.");
        return ESP_OK;
    }

    // For any active state (MOVING or CALIBRATING), just queue a generic STOP command.
    // The logic to handle the context (e.g., setting calibration points) is in process_command.
    ESP_LOGI(TAG, "Queueing stop command.");
    motor_command_t cmd = { .type = CMD_STOP };
    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send stop command to queue");
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

void motor_get_position(motor_handle_t handle, int32_t *current_pos, int32_t *target_pos) {
    if (!handle) {
        if (current_pos) *current_pos = 0;
        if (target_pos) *target_pos = 0;
        return;
    }
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    if (current_pos) *current_pos = handle->current_position;
    if (target_pos) *target_pos = handle->target_position;
    xSemaphoreGive(handle->lock);
}

motor_state_t motor_get_state(motor_handle_t handle) {
    if (!handle) return MOTOR_STATE_ERROR;
    motor_state_t state;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    state = handle->current_state;
    xSemaphoreGive(handle->lock);
    return state;
}

motor_direction_t motor_get_direction(motor_handle_t handle) {
    if (!handle) return MOTOR_DIRECTION_STOP;
    motor_direction_t dir;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    dir = handle->current_direction;
    xSemaphoreGive(handle->lock);
    return dir;
}

bool motor_is_calibrated(motor_handle_t handle) {
    if (!handle) return false;
    bool calibrated;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    calibrated = handle->is_calibrated;
    xSemaphoreGive(handle->lock);
    return calibrated;
}

esp_err_t motor_update_pid_params(motor_handle_t handle, float kp, float ki, float kd) {
    if (!handle || !handle->pid_controller) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->pid_params.kp = kp;
    handle->pid_params.ki = ki;
    handle->pid_params.kd = kd;
    err = pid_update_parameters(handle->pid_controller, &handle->pid_params);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Updated PID gains to: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
    } else {
        ESP_LOGE(TAG, "Failed to update PID parameters");
    }
    xSemaphoreGive(handle->lock);
    return err;
}

// --- Private Helper Functions ---

/**
 * @brief Configures PID parameters and sets the motor state to initiate a movement.
 *
 * @param handle Motor control handle.
 * @param target The target position for the movement.
 * @param speed The maximum speed for the movement (in pulses per second).
 * @param state The new motor state (e.g., MOVING or CALIBRATING).
 */
static void motor_initiate_movement(motor_handle_t handle, int32_t target, float speed, motor_state_t state) {
    ESP_LOGI(TAG, "CMD: Move %ld -> %ld, speed=%.2f, state=%d", handle->current_position, target, speed, state);
    
    // Reset PID controller for a new, distinct movement.
    pid_reset_ctrl_block(handle->pid_controller);

    // Update speed limits in the PID controller
    handle->pid_params.max_output = fabsf(speed);
    handle->pid_params.min_output = -fabsf(speed);
    pid_update_parameters(handle->pid_controller, &handle->pid_params);
    
    handle->target_position = target;
    motor_set_state(handle, state);
}


static void motor_pid_control_task(void *pvParameters) {
    motor_handle_t handle = (motor_handle_t)pvParameters;
    motor_command_t received_cmd;
    int64_t move_start_time_ms = 0;

    while (1) {
        if (xQueueReceive(handle->cmd_queue, &received_cmd, 0) == pdPASS) {
            xSemaphoreTake(handle->lock, portMAX_DELAY);
            process_command(handle, &received_cmd);
            move_start_time_ms = 0;
            xSemaphoreGive(handle->lock);
        }

        xSemaphoreTake(handle->lock, portMAX_DELAY);
        int64_t current_time_ms = esp_log_timestamp();
        motor_state_t current_state = handle->current_state;
        int32_t target_pos = handle->target_position;
        int32_t current_pos = handle->current_position;

        switch (current_state) {
            case MOTOR_STATE_IDLE:
                break;
            case MOTOR_STATE_MOVING:
            case MOTOR_STATE_CALIBRATING:
                if (move_start_time_ms == 0) move_start_time_ms = current_time_ms;
                if (current_time_ms - move_start_time_ms > CONFIG_PID_FAILURE_TIMEOUT_MS) {
                    ESP_LOGE(TAG, "PID failure: Timed out in state %d", current_state);
                    motor_set_state(handle, MOTOR_STATE_ERROR);
                    motor_set_pwm_duty(0);
                } else if (current_state == MOTOR_STATE_MOVING && abs(target_pos - current_pos) <= CONFIG_POSITION_DEADBAND) {
                    ESP_LOGI(TAG, "Movement finished. Final position: %" PRId32, current_pos);
                    motor_set_state(handle, MOTOR_STATE_IDLE);
                    nvs_handle_t nvs_handle;
                    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
                        nvs_set_i32(nvs_handle, NVS_POSITION_KEY, current_pos);
                        nvs_commit(nvs_handle);
                        nvs_close(nvs_handle);
                    }
                }
                break;
            case MOTOR_STATE_ERROR:
                xSemaphoreGive(handle->lock);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
        }

        float error = (float)(target_pos - current_pos);
        float pid_output;
        pid_compute(handle->pid_controller, error, &pid_output);

        if (current_state == MOTOR_STATE_IDLE && fabsf(error) <= CONFIG_POSITION_DEADBAND) {
            pid_output = 0;
        }
        
        motor_set_pwm_duty(pid_output);
        xSemaphoreGive(handle->lock);
        
        vTaskDelay(pdMS_TO_TICKS(PID_CONTROL_PERIOD_MS));
    }
}

static void process_command(motor_handle_t handle, const motor_command_t *cmd) {
    switch (cmd->type) {
        case CMD_MOVE_RELATIVE: {
            if (!handle->is_calibrated) {
                ESP_LOGE(TAG, "Cannot move relative, motor not calibrated.");
                return;
            }
            if (handle->current_state == MOTOR_STATE_ERROR) {
                ESP_LOGE(TAG, "Cannot move relative, motor is in an error state.");
                return;
            }
            int32_t new_target = handle->current_position + cmd->position;
            motor_command_t set_target_cmd = {
                .type = CMD_SET_TARGET,
                .position = new_target,
                .max_speed_pps = cmd->max_speed_pps
            };
            process_command(handle, &set_target_cmd);
            break;
        }
        case CMD_STOP:
            // This command now handles stopping a regular move AND finishing a calibration move.
            if (handle->current_state == MOTOR_STATE_CALIBRATING) {
                if (handle->current_direction == MOTOR_DIRECTION_UP) {
                    // Logic to set the zero point
                    ESP_LOGI(TAG, "CMD: Setting zero point from calibration stop.");
                    int32_t offset = handle->current_position;
                    handle->current_position = 0;
                    if (handle->fully_closed_position > 0) {
                        handle->fully_closed_position -= offset;
                    }
                    handle->is_calibrated = true;
                    ESP_LOGI(TAG, "New zero point set. Position offset by %" PRId32, -offset);
                    
                    nvs_handle_t nvs_handle;
                    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
                        nvs_set_i32(nvs_handle, NVS_POSITION_KEY, handle->current_position);
                        nvs_set_i32(nvs_handle, NVS_FULLY_CLOSED_KEY, handle->fully_closed_position);
                        nvs_commit(nvs_handle);
                        nvs_close(nvs_handle);
                    }
                } else if (handle->current_direction == MOTOR_DIRECTION_DOWN) {
                    // Logic to set the fully closed point
                    ESP_LOGI(TAG, "CMD: Setting fully closed point from calibration stop.");
                    handle->fully_closed_position = handle->current_position;
                    ESP_LOGI(TAG, "New fully closed position set to %" PRId32, handle->fully_closed_position);
                    nvs_handle_t nvs_handle_closed;
                    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle_closed) == ESP_OK) {
                        nvs_set_i32(nvs_handle_closed, NVS_FULLY_CLOSED_KEY, handle->fully_closed_position);
                        nvs_commit(nvs_handle_closed);
                        nvs_close(nvs_handle_closed);
                    }
                }
            }
            
            // For all stop scenarios, set target to current position and go to IDLE.
            handle->target_position = handle->current_position;
            pid_reset_ctrl_block(handle->pid_controller);
            
            motor_set_state(handle, MOTOR_STATE_IDLE);
            ESP_LOGI(TAG, "CMD: Motor stopped at position %" PRId32, handle->current_position);
            break;
        case CMD_SET_TARGET: {
            if (!handle->is_calibrated) {
                ESP_LOGE(TAG, "Cannot set target, motor not calibrated.");
                return;
            }
            if (handle->current_state == MOTOR_STATE_ERROR) {
                ESP_LOGE(TAG, "Cannot set target, motor is in an error state.");
                return;
            }
            int32_t new_target = cmd->position;
            if (handle->fully_closed_position > 0) {
                new_target = fmaxf(0, fminf(new_target, handle->fully_closed_position));
            }
            float speed_to_use = (cmd->max_speed_pps > 0) ? cmd->max_speed_pps : handle->default_speed_pps;
            motor_initiate_movement(handle, new_target, speed_to_use, MOTOR_STATE_MOVING);
            break;
        }
        case CMD_START_CALIBRATION_UP:
            if (handle->current_state == MOTOR_STATE_ERROR) return;
            motor_initiate_movement(handle, -2000000000, cmd->max_speed_pps, MOTOR_STATE_CALIBRATING);
            break;
        case CMD_START_CALIBRATION_DOWN:
            if (handle->current_state == MOTOR_STATE_ERROR) return;
            motor_initiate_movement(handle, 2000000000, cmd->max_speed_pps, MOTOR_STATE_CALIBRATING);
            break;
    }
}

static esp_err_t motor_set_pwm_duty(float duty_percent) {
    duty_percent = fmaxf(PID_MIN_OUTPUT, fminf(PID_MAX_OUTPUT, duty_percent));
    uint32_t duty_val = (uint32_t)(fabsf(duty_percent) / 100.0f * (1 << LEDC_DUTY_RES));

    bool move_down = (duty_percent > 0);

#if CONFIG_MOTOR_REVERSE_DIRECTION
    move_down = !move_down;
#endif

    if (move_down) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty_val);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    } else { // Move up or stop
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

#if CONFIG_MOTOR_REVERSE_DIRECTION
    transition = -transition;
#endif
    
    if (transition == 1) {
        handle->current_position++;
    } else if (transition == -1) {
        handle->current_position--;
    }
    
    handle->last_hall_state = current_state;
}

static void motor_set_state(motor_handle_t handle, motor_state_t new_state) {
    // Determine direction based on the new state
    motor_direction_t new_direction = MOTOR_DIRECTION_STOP;
    if (new_state == MOTOR_STATE_MOVING || new_state == MOTOR_STATE_CALIBRATING) {
        // Determine direction by comparing target to current position
        if (handle->target_position > handle->current_position) {
            new_direction = MOTOR_DIRECTION_DOWN;
        } else if (handle->target_position < handle->current_position) {
            new_direction = MOTOR_DIRECTION_UP;
        }
    }

    // For IDLE or ERROR state, the direction is implicitly STOP.
    if (handle->current_direction != new_direction) {
        handle->current_direction = new_direction;
        esp_event_post(MOTOR_EVENTS, MOTOR_EVENT_DIRECTION_CHANGED, &new_direction, sizeof(new_direction), pdMS_TO_TICKS(10));
    }

    if (handle->current_state != new_state) {
        handle->current_state = new_state;
        esp_event_post(MOTOR_EVENTS, MOTOR_EVENT_STATE_CHANGED, &new_state, sizeof(new_state), pdMS_TO_TICKS(10));
    }
}

int32_t motor_get_fully_closed_position(motor_handle_t handle) {
    if (!handle) return 0;
    int32_t pos;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    pos = handle->fully_closed_position;
    xSemaphoreGive(handle->lock);
    return pos;
}

// --- New Public API Functions ---

esp_err_t motor_set_default_speed(motor_handle_t handle, float speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (speed_pps <= 0) return ESP_ERR_INVALID_ARG;

    esp_err_t err = ESP_FAIL;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->default_speed_pps = speed_pps;
    ESP_LOGI(TAG, "Set default speed to %.2f pps", speed_pps);

    nvs_handle_t nvs_handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS for writing speed");
    } else {
        char speed_str[16];
        snprintf(speed_str, sizeof(speed_str), "%.2f", speed_pps);
        err = nvs_set_str(nvs_handle, NVS_SPEED_KEY, speed_str);
        if (err == ESP_OK) {
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    xSemaphoreGive(handle->lock);
    return err;
}

float motor_get_default_speed(motor_handle_t handle) {
    if (!handle) return 0.0f;
    float speed;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    speed = handle->default_speed_pps;
    xSemaphoreGive(handle->lock);
    return speed;
}

esp_err_t motor_go_to_percentage(motor_handle_t handle, uint8_t percentage, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (percentage > 100) percentage = 100;

    int32_t target_pos = 0;
    bool is_calibrated = false;
    int32_t fully_closed_pos = 0;

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    is_calibrated = handle->is_calibrated;
    fully_closed_pos = handle->fully_closed_position;
    xSemaphoreGive(handle->lock);

    if (!is_calibrated || fully_closed_pos <= 0) {
        ESP_LOGE(TAG, "Cannot go to percentage. Motor not calibrated or fully closed position not set.");
        return ESP_ERR_INVALID_STATE;
    }

    target_pos = (int32_t)(((float)percentage / 100.0f) * fully_closed_pos);
    ESP_LOGI(TAG, "Moving to %d%%, which is position %" PRId32, percentage, target_pos);

    return motor_set_target(handle, target_pos, max_speed_pps);
}

esp_err_t motor_move_relative(motor_handle_t handle, int32_t delta, float max_speed_pps) {
    if (!handle) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Queueing relative move by %" PRId32, delta);

    motor_command_t cmd = {
        .type = CMD_MOVE_RELATIVE,
        .position = delta, // Use position field to store the delta
        .max_speed_pps = max_speed_pps
    };

    if (xQueueSend(handle->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send relative_move command to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}
