/**
 * @file joystick_input.h
 * @brief Analog joystick input handling
 */

#ifndef JOYSTICK_INPUT_H
#define JOYSTICK_INPUT_H

#include "esp_err.h"
#include "types.h"

/**
 * @brief Joystick direction
 */
typedef enum {
  JOY_DIR_CENTER,
  JOY_DIR_FORWARD,
  JOY_DIR_BACKWARD,
  JOY_DIR_LEFT,
  JOY_DIR_RIGHT
} joystick_dir_t;

/**
 * @brief Initialize joystick ADC and GPIO
 * @return ESP_OK on success
 */
esp_err_t joystick_init(void);

/**
 * @brief Read current joystick direction
 * @return Current direction
 */
joystick_dir_t joystick_get_direction(void);

/**
 * @brief Check if joystick button is pressed
 * @return true if pressed
 */
bool joystick_button_pressed(void);

/**
 * @brief Joystick polling task - sends events to queue when in Remote mode
 * @param param Task parameter (unused)
 */
void joystick_task(void *param);

#endif // JOYSTICK_INPUT_H
