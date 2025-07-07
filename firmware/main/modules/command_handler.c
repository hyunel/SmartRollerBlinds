#include "command_handler.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CMD_HANDLER";

// --- Static Helper Functions for Individual Commands ---

static void handle_help_cmd(char *response_buffer, size_t buffer_size) {
    snprintf(response_buffer, buffer_size,
             "Available commands:\n"
             "  help - Show this message\n"
             "  status - Get current motor status\n"
             "  cal_up - Start moving up to calibrate zero point\n"
             "  cal_down - Start moving down to calibrate closed point\n"
             "  stop - Stop any motor movement\n"
             "  goto <percentage> [speed] - Move motor to a percentage (0-100) with optional speed\n"
             "  move <delta> [speed] - Move motor by a relative amount\n"
             "  speed <pps> - Set the default motor speed\n"
             "  set_pid <kp> <ki> <kd> - Set PID gains");
}

static void handle_status_cmd(motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    int32_t pos, target_pos;
    motor_get_position(motor_handle, &pos, &target_pos);
    motor_state_t state = motor_get_state(motor_handle);
    bool calibrated = motor_is_calibrated(motor_handle);
    float speed = motor_get_default_speed(motor_handle);
    int32_t fully_closed_pos = motor_get_fully_closed_position(motor_handle);
    
    const char* state_str = "Unknown";
    switch(state) {
        case MOTOR_STATE_IDLE: state_str = "Idle"; break;
        case MOTOR_STATE_MOVING: state_str = "Moving"; break;
        case MOTOR_STATE_CALIBRATING: state_str = "Calibrating"; break;
        case MOTOR_STATE_ERROR: state_str = "Error"; break;
    }
    snprintf(response_buffer, buffer_size, "Pos: %" PRIi32 " -> %" PRIi32 " (Max: %" PRIi32 ") | State: %s | Calibrated: %s | Speed: %.2f pps", pos, target_pos, fully_closed_pos, state_str, calibrated ? "Y" : "N", speed);
}

static void handle_cal_up_cmd(motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    motor_start_calibration(motor_handle, true, CONFIG_DEFAULT_MOVE_SPEED);
    snprintf(response_buffer, buffer_size, "Starting upward calibration.");
}

static void handle_cal_down_cmd(motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    motor_start_calibration(motor_handle, false, CONFIG_DEFAULT_MOVE_SPEED);
    snprintf(response_buffer, buffer_size, "Starting downward calibration.");
}

static void handle_stop_cmd(motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    motor_state_t current_state = motor_get_state(motor_handle);
    if (current_state == MOTOR_STATE_IDLE) {
        snprintf(response_buffer, buffer_size, "Motor is already idle.");
    } else {
        motor_stop(motor_handle);
        snprintf(response_buffer, buffer_size, "Stop command sent.");
    }
}

static void handle_goto_cmd(char *saveptr, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    char *pos_str = strtok_r(NULL, " \n\r", &saveptr);
    if (pos_str) {
        int percentage = atoi(pos_str);
        float speed = 0; // Default to 0, which means use default speed

        // Check for optional speed argument
        char *speed_str = strtok_r(NULL, " \n\r", &saveptr);
        if (speed_str) {
            speed = atof(speed_str);
        }

        motor_go_to_percentage(motor_handle, percentage, speed);
        if (speed > 0) {
            snprintf(response_buffer, buffer_size, "Move command sent to %d%% with speed %.2f.", percentage, speed);
        } else {
            snprintf(response_buffer, buffer_size, "Goto command sent to %d%% with default speed.", percentage);
        }
    } else {
        snprintf(response_buffer, buffer_size, "Error: 'goto' command requires a percentage argument.");
    }
}

static void handle_move_cmd(char *saveptr, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    char *delta_str = strtok_r(NULL, " \n\r", &saveptr);
    if (delta_str) {
        int32_t delta = atoi(delta_str);
        float speed = 0; // Default to 0, which means use default speed

        // Check for optional speed argument
        char *speed_str = strtok_r(NULL, " \n\r", &saveptr);
        if (speed_str) {
            speed = atof(speed_str);
        }

        motor_move_relative(motor_handle, delta, speed);
        if (speed > 0) {
            snprintf(response_buffer, buffer_size, "Relative move command sent for %" PRId32 " steps with speed %.2f.", delta, speed);
        } else {
            snprintf(response_buffer, buffer_size, "Move command sent for %" PRId32 " steps with default speed.", delta);
        }
    } else {
        snprintf(response_buffer, buffer_size, "Error: 'move' command requires a delta argument.");
    }
}

static void handle_set_pid_cmd(char *saveptr, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    char *kp_str = strtok_r(NULL, " \n\r", &saveptr);
    char *ki_str = strtok_r(NULL, " \n\r", &saveptr);
    char *kd_str = strtok_r(NULL, " \n\r", &saveptr);
    if (kp_str && ki_str && kd_str) {
        float kp = atof(kp_str);
        float ki = atof(ki_str);
        float kd = atof(kd_str);
        motor_update_pid_params(motor_handle, kp, ki, kd);
        snprintf(response_buffer, buffer_size, "PID params updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
    } else {
        snprintf(response_buffer, buffer_size, "Error: 'set_pid' requires kp, ki, and kd arguments.");
    }
}

static void handle_speed_cmd(char *saveptr, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    char *speed_str = strtok_r(NULL, " \n\r", &saveptr);
    if (speed_str) {
        float speed = atof(speed_str);
        if (speed > 0) {
            motor_set_default_speed(motor_handle, speed);
            snprintf(response_buffer, buffer_size, "Default speed set to %.2f pps.", speed);
        } else {
            snprintf(response_buffer, buffer_size, "Error: Speed must be a positive number.");
        }
    } else {
        snprintf(response_buffer, buffer_size, "Error: 'speed' command requires a pps argument.");
    }
}


// --- Public API Function ---

void cmd_handle_command_string(char *command_string, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size) {
    char *cmd;
    char *saveptr;

    if (buffer_size > 0) {
        response_buffer[0] = '\0';
    }
    if (command_string == NULL) {
        snprintf(response_buffer, buffer_size, "Error: Empty command.");
        return;
    }

    ESP_LOGI(TAG, "Processing command: %s", command_string);
    cmd = strtok_r(command_string, " \n\r", &saveptr);

    if (cmd == NULL) {
        snprintf(response_buffer, buffer_size, "Error: Empty command.");
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        handle_help_cmd(response_buffer, buffer_size);
    } else if (!motor_handle) {
        // All other commands require a motor handle.
        ESP_LOGE(TAG, "Motor not initialized!");
        snprintf(response_buffer, buffer_size, "Error: Motor not initialized.");
    } else if (strcmp(cmd, "status") == 0) {
        handle_status_cmd(motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "cal_up") == 0) {
        handle_cal_up_cmd(motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "cal_down") == 0) {
        handle_cal_down_cmd(motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "stop") == 0) {
        handle_stop_cmd(motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "goto") == 0) {
        handle_goto_cmd(saveptr, motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "move") == 0) {
        handle_move_cmd(saveptr, motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "speed") == 0) {
        handle_speed_cmd(saveptr, motor_handle, response_buffer, buffer_size);
    } else if (strcmp(cmd, "set_pid") == 0) {
        handle_set_pid_cmd(saveptr, motor_handle, response_buffer, buffer_size);
    } else {
        snprintf(response_buffer, buffer_size, "Error: Unknown command '%s'", cmd);
    }
}
