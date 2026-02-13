/**
 * @file mode_manager.h
 * @brief System mode management for input isolation
 *
 * Provides robust isolation between Voice and Remote modes
 * Includes settings menu for voice command duration
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
 * @return true if voice commands should be processed
 */
bool is_voice_input_allowed(void);

/**
 * @brief Check if joystick input should be processed
 * @return true if joystick input should be processed
 */
bool is_joystick_input_allowed(void);

/**
 * @brief Handle button events for mode selection and settings
 * @param event Button event
 * @return true if mode was changed (Voice/Remote selected)
 */
bool mode_manager_handle_button(rc_event_type_t event);

/**
 * @brief Get currently selected menu item
 * @return Selected index (0=Voice, 1=Remote, 2=Settings)
 */
uint8_t mode_manager_get_selection(void);

/**
 * @brief Get voice command duration setting
 * @return Duration in milliseconds
 */
uint16_t mode_manager_get_voice_duration(void);

/**
 * @brief Return to mode selection menu from any mode
 * Called on double-press OK
 */
void mode_manager_return_to_menu(void);

#endif // MODE_MANAGER_H
