/**
 * @file failsafe.h
 * @brief Fail-safe mechanism interface for Voice-Controlled RC System
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include "esp_err.h"
#include "types.h"


/**
 * @brief Initialize fail-safe subsystem
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t failsafe_init(void);

/**
 * @brief Check if motors are currently allowed
 * @return true if motors can be enabled, false otherwise
 */
bool failsafe_motors_allowed(void);

/**
 * @brief Enable motor permission
 * @note Should only be called by state machine when entering motion state
 */
void failsafe_enable_motors(void);

/**
 * @brief Disable motor permission
 * @note Safe to call from any context
 */
void failsafe_disable_motors(void);

/**
 * @brief Update last valid command timestamp
 * @note Call this when a valid voice command is received
 */
void failsafe_update_command_time(void);

/**
 * @brief Check for command timeout
 * @return true if timeout has occurred
 */
bool failsafe_check_timeout(void);

/**
 * @brief Run periodic fail-safe checks
 * @note This function should be called by the motor safety task
 * @return true if system is healthy, false if emergency stop triggered
 */
bool failsafe_periodic_check(void);

/**
 * @brief Set system health status
 * @param healthy true if system is healthy
 */
void failsafe_set_health(bool healthy);

/**
 * @brief Get system health status
 * @return true if system is healthy
 */
bool failsafe_is_healthy(void);

/**
 * @brief Get fail-safe state (for debugging)
 * @return Copy of current fail-safe state
 */
failsafe_state_t failsafe_get_state(void);

/**
 * @brief Reset fail-safe state to initial values
 */
void failsafe_reset(void);

#endif // FAILSAFE_H
