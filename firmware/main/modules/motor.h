#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "esp_err.h"
#include "esp_event.h"
#include <stdbool.h>
#include <stdint.h>

ESP_EVENT_DECLARE_BASE(MOTOR_EVENTS);

/**
 * @brief Opaque handle to a motor instance.
 */
typedef struct motor_control_t* motor_handle_t;

/**
 * @brief Defines the operational state of the motor.
 */
typedef enum {
    MOTOR_STATE_IDLE,               /*!< Motor is idle and holding position */
    MOTOR_STATE_MOVING,             /*!< Motor is actively moving to a target position */
    MOTOR_STATE_CALIBRATING,        /*!< Motor is moving indefinitely for calibration */
    MOTOR_STATE_ERROR,              /*!< Motor has encountered an error (e.g., PID failed to reach target) */
} motor_state_t;

/**
 * @brief Defines the physical direction of motor movement.
 */
typedef enum {
    MOTOR_DIRECTION_STOP,           /*!< Motor is stopped */
    MOTOR_DIRECTION_UP,             /*!< Motor is moving up (opening) */
    MOTOR_DIRECTION_DOWN,           /*!< Motor is moving down (closing) */
} motor_direction_t;

/**
 * @brief Motor event IDs
 */
typedef enum {
    MOTOR_EVENT_STATE_CHANGED,      /*!< Motor state has changed. Event data is a pointer to motor_state_t */
    MOTOR_EVENT_DIRECTION_CHANGED,  /*!< Motor direction has changed. Event data is a pointer to motor_direction_t */
} motor_event_t;


/**
 * @brief Creates and initializes a new motor control instance.
 *
 * Sets up hardware, software components (PID, NVS, command queue), and starts the control task.
 * It attempts to load the last known position from NVS.
 *
 * @param motor_handle Pointer to a motor_handle_t to store the handle of the new instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_init(motor_handle_t *motor_handle);

/**
 * @brief Sends a command to initiate the calibration sequence.
 *
 * This is a non-blocking function. The motor will move a large, fixed distance
 * to ensure it reaches a physical limit, at which point the gear will slip.
 * This final position is then defined as the new zero point.
 *
 * @param handle The handle to the motor instance.
 * @param max_speed_pps The maximum speed for the calibration movement in pulses per second.
 * @return ESP_OK if the command was successfully sent to the queue.
 */
esp_err_t motor_start_calibration(motor_handle_t handle, bool move_up, float max_speed_pps);

/**
 * @brief Stops an active calibration movement and sets the final position.
 *
 * This function should be called after `motor_start_calibration`. It stops the motor
 * and defines either the zero point or the fully closed point based on the `is_setting_zero` flag.
 *
 * @param handle The handle to the motor instance.
 * @param is_setting_zero If true, the current position is set as the new zero, and all other
 *                        positions are adjusted accordingly. If false, the current position is
 *                        set as the new 'fully_closed_position'.
 * @return ESP_OK on success, or ESP_ERR_INVALID_STATE if not in a calibration state.
 */
esp_err_t motor_stop(motor_handle_t handle);

/**
 * @brief Sends a command to set the motor's target position and speed.
 *
 * This is a non-blocking function that sends a command to the motor task's queue.
 *
 * @param handle The handle to the motor instance.
 * @param position The target absolute position in encoder pulses.
 * @param max_speed_pps The maximum speed for the movement in pulses per second. If 0, the default speed is used.
 * @return ESP_OK if the command was successfully sent to the queue.
 */
esp_err_t motor_set_target(motor_handle_t handle, int32_t position, float max_speed_pps);

/**
 * @brief Sends a command to move the motor by a relative amount.
 *
 * @param handle The handle to the motor instance.
 * @param delta The relative amount to move. Negative for up, positive for down.
 * @param max_speed_pps The maximum speed for the movement. If 0, the default speed is used.
 * @return ESP_OK if the command was successfully sent to the queue.
 */
esp_err_t motor_move_relative(motor_handle_t handle, int32_t delta, float max_speed_pps);

/**
 * @brief Gets the current and target positions of the motor.
 *
 * @param handle The handle to the motor instance.
 * @param[out] current_pos Pointer to store the current position.
 * @param[out] target_pos Pointer to store the target position.
 */
void motor_get_position(motor_handle_t handle, int32_t *current_pos, int32_t *target_pos);

/**
 * @brief Gets the raw encoder value for the fully closed position.
 *
 * @param handle The handle to the motor instance.
 * @return The raw encoder value of the fully closed position. Returns 0 if not calibrated.
 */
int32_t motor_get_fully_closed_position(motor_handle_t handle);

/**
 * @brief Get the current state of the motor.
 *
 * @param handle Handle to the motor control instance.
 * @return The current motor_state_t.
 */
motor_state_t motor_get_state(motor_handle_t handle);

/**
 * @brief Get the current physical direction of the motor.
 *
 * @param handle Handle to the motor control instance.
 * @return The current motor_direction_t.
 */
motor_direction_t motor_get_direction(motor_handle_t handle);

/**
 * @brief Set the default speed for the motor.
 *
 * This speed will be used for movements where no specific speed is provided.
 * The value is persisted in NVS.
 *
 * @param handle Handle to the motor control instance.
 * @param speed_pps The default speed in pulses per second.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_set_default_speed(motor_handle_t handle, float speed_pps);

/**
 * @brief Get the currently configured default speed for the motor.
 *
 * @param handle Handle to the motor control instance.
 * @return The default speed in pulses per second.
 */
float motor_get_default_speed(motor_handle_t handle);

/**
 * @brief Checks if the motor has been calibrated.
 *
 * @param handle The handle to the motor instance.
 * @return true if calibrated, false otherwise.
 */
bool motor_is_calibrated(motor_handle_t handle);

/**
 * @brief Updates the PID controller gains.
 *
 * @param handle The handle to the motor instance.
 * @param kp The new proportional gain.
 * @param ki The new integral gain.
 * @param kd The new derivative gain.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_update_pid_params(motor_handle_t handle, float kp, float ki, float kd);

/**
 * @brief Sends a command to move the motor to a specific percentage of its total travel.
 *
 * 0% is fully open (position 0), and 100% is fully closed.
 * Requires `motor_set_fully_closed_position` to have been called previously to define the travel range.
 *
 * @param handle The handle to the motor instance.
 * @param percentage The target percentage (0-100).
 * @param max_speed_pps The maximum speed for the movement. If 0, the default speed is used.
 * @return ESP_OK if the command was successfully sent.
 */
esp_err_t motor_go_to_percentage(motor_handle_t handle, uint8_t percentage, float max_speed_pps);


#endif // _MOTOR_H_
