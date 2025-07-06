#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "modules/motor.h"

/**
 * @brief Processes a command string from any source (UART, MQTT, etc.).
 *
 * This function parses the command and its arguments, executes the corresponding
 * motor action, and writes a response string into the provided buffer.
 *
 * @param command_string The raw command string (e.g., "move 50", "calibrate").
 * @param motor_handle Handle to the motor instance.
 * @param response_buffer A buffer to write the response string into.
 * @param buffer_size The size of the response buffer.
 */
void cmd_handle_command_string(char *command_string, motor_handle_t motor_handle, char *response_buffer, size_t buffer_size);

#endif // COMMAND_HANDLER_H
