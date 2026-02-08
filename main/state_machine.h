/**
 * @file state_machine.h
 * @brief State machine interface for Voice-Controlled RC System
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "esp_err.h"
#include "types.h"


/**
 * @brief Initialize state machine
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t state_machine_init(void);

/**
 * @brief Process an event and update state
 * @param event Pointer to event to process
 * @return New state after processing
 */
rc_state_t state_machine_process_event(const rc_event_msg_t *event);

/**
 * @brief Get current state
 * @return Current state
 */
rc_state_t state_machine_get_state(void);

/**
 * @brief Force transition to a specific state
 * @param state Target state
 * @note Only use for error recovery
 */
void state_machine_force_state(rc_state_t state);

/**
 * @brief State machine task entry point
 * @param param Task parameter (unused)
 */
void state_machine_task(void *param);

#endif // STATE_MACHINE_H
