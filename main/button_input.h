/**
 * @file button_input.h
 * @brief Push button input handling with debounce
 */

#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include "esp_err.h"
#include "types.h"

/**
 * @brief Initialize button GPIO pins
 * @return ESP_OK on success
 */
esp_err_t button_init(void);

/**
 * @brief Check if a button is currently pressed
 * @param button 0=UP, 1=DOWN, 2=OK
 * @return true if pressed
 */
bool button_is_pressed(uint8_t button);

/**
 * @brief Button polling task - sends events to queue
 * @param param Task parameter (unused)
 */
void button_task(void *param);

#endif // BUTTON_INPUT_H
