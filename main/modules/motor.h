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
    MOTOR_STATE_IDLE,         /*!< Motor is idle and holding position */
    MOTOR_STATE_MOVING,       /*!< Motor is actively moving to a target position */
    MOTOR_STATE_CALIBRATING,  /*!< Motor is performing the initial calibration sequence */
    MOTOR_STATE_HOMING,       /*!< Motor is moving to the zero position to re-synchronize */
    MOTOR_STATE_ERROR,        /*!< Motor has encountered an error (e.g., PID failed to reach target) */
} motor_state_t;

/**
 * @brief Motor event IDs
 */
typedef enum {
    MOTOR_EVENT_STATE_CHANGED, /*!< Motor state has changed. Event data is a pointer to motor_state_t */
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
esp_err_t motor_calibrate(motor_handle_t handle, float max_speed_pps);

/**
 * @brief Sends a command to move the motor to the zero position to re-synchronize.
 *
 * This is a non-blocking function. The motor moves to what it believes is position 0,
 * plus a small overrun, to ensure it hits the physical limit. The position is then
 * reset to 0 to correct any accumulated error.
 *
 * @param handle The handle to the motor instance.
 * @param max_speed_pps The maximum speed for the homing movement in pulses per second.
 * @return ESP_OK if the command was successfully sent to the queue.
 */
esp_err_t motor_go_to_zero(motor_handle_t handle, float max_speed_pps);

/**
 * @brief Sends a command to set the motor's target position and speed.
 *
 * This is a non-blocking function that sends a command to the motor task's queue.
 *
 * @param handle The handle to the motor instance.
 * @param position The target absolute position in encoder pulses.
 * @param max_speed_pps The maximum speed for the movement in pulses per second.
 * @return ESP_OK if the command was successfully sent to the queue.
 */
esp_err_t motor_set_target(motor_handle_t handle, int32_t position, float max_speed_pps);

/**
 * @brief Gets the current absolute position of the motor.
 *
 * @param handle The handle to the motor instance.
 * @return The current position in encoder pulses.
 */
int32_t motor_get_position(motor_handle_t handle);

/**
 * @brief Gets the current target position of the motor.
 *
 * @param handle The handle to the motor instance.
 * @return The target position in encoder pulses.
 */
int32_t motor_get_target_position(motor_handle_t handle);

/**
 * @brief Gets the current operational state of the motor.
 *
 * @param handle The handle to the motor instance.
 * @return The current motor_state_t enum value.
 */
motor_state_t motor_get_state(motor_handle_t handle);

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
 * @brief Sets the motor's current position as the fully closed position.
 *
 * This defines the maximum travel distance for the blinds and is required for
 * percentage-based control. The value is saved to NVS.
 *
 * @param handle The handle to the motor instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_set_fully_closed_position(motor_handle_t handle);

/**
 * @brief Sends a command to move the motor to a specific percentage of its total travel.
 *
 * 0% is fully open (position 0), and 100% is fully closed.
 * Requires `motor_set_fully_closed_position` to have been called previously to define the travel range.
 *
 * @param handle The handle to the motor instance.
 * @param percentage The target percentage (0-100).
 * @param max_speed_pps The maximum speed for the movement.
 * @return ESP_OK if the command was successfully sent.
 */
esp_err_t motor_go_to_percentage(motor_handle_t handle, uint8_t percentage, float max_speed_pps);


#endif // _MOTOR_H_
