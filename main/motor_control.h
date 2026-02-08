/**
 * @file motor_control.h
 * @brief Motor control interface for Voice-Controlled RC System
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_err.h"
#include "types.h"


/**
 * @brief Initialize motor control GPIO and state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_init(void);

/**
 * @brief Move forward
 * @note Only executes if motors are allowed by fail-safe
 */
void motor_forward(void);

/**
 * @brief Move backward
 * @note Only executes if motors are allowed by fail-safe
 */
void motor_backward(void);

/**
 * @brief Turn left (differential drive)
 * @note Only executes if motors are allowed by fail-safe
 */
void motor_turn_left(void);

/**
 * @brief Turn right (differential drive)
 * @note Only executes if motors are allowed by fail-safe
 */
void motor_turn_right(void);

/**
 * @brief Stop all motors gracefully
 */
void motor_stop_all(void);

/**
 * @brief Emergency stop - immediate halt, bypasses all checks
 * @note Can be called from any context (interrupt-safe)
 */
void motor_emergency_stop(void);

/**
 * @brief Get current motor state
 * @return Current motor state structure
 */
motor_state_t motor_get_state(void);

/**
 * @brief Check if motors are currently active
 * @return true if any motor is running
 */
bool motor_is_active(void);

/**
 * @brief Verify motor physical state matches commanded state
 * @return true if state matches, false if mismatch detected
 */
bool motor_verify_state(void);

#endif // MOTOR_CONTROL_H
