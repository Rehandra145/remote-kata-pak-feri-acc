/**
 * @file mode_manager.h
 * @brief System mode management for input isolation
 *
 * Provides robust isolation between Voice and Remote modes
 */

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "esp_err.h"
#include "types.h"

/**
 * @brief Initialize mode manager
 * Starts in MODE_SELECTING state
 * @return ESP_OK on success
 */
esp_err_t mode_manager_init(void);

/**
 * @brief Get current system mode
 * @return Current mode
 */
system_mode_t mode_manager_get_mode(void);

/**
 * @brief Set system mode
 * @param mode New mode to set
 */
void mode_manager_set_mode(system_mode_t mode);

/**
 * @brief Check if voice input should be processed
 * Only returns true when in SYS_MODE_VOICE
 * @return true if voice commands should be processed
 */
bool is_voice_input_allowed(void);

/**
 * @brief Check if joystick input should be processed
 * Only returns true when in SYS_MODE_REMOTE
 * @return true if joystick input should be processed
 */
bool is_joystick_input_allowed(void);

/**
 * @brief Handle button events for mode selection
 * @param event Button event
 * @return true if mode was changed
 */
bool mode_manager_handle_button(rc_event_type_t event);

/**
 * @brief Get currently selected menu item (0=Voice, 1=Remote)
 * @return Selected index
 */
uint8_t mode_manager_get_selection(void);

#endif // MODE_MANAGER_H
