/**
 * @file oled_display.h
 * @brief OLED Display interface for Voice-Controlled System
 *
 * Uses SSD1306 0.96" I2C OLED display
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "esp_err.h"
#include "types.h"

/**
 * @brief Initialize OLED display
 * @return ESP_OK on success
 */
esp_err_t oled_init(void);

/**
 * @brief Clear the display
 */
void oled_clear(void);

/**
 * @brief Display system status
 * @param state Current system state
 */
void oled_show_state(rc_state_t state);

/**
 * @brief Display detected voice command
 * @param command Command string to display
 * @param confidence Recognition confidence (0.0-1.0)
 */
void oled_show_command(const char *command, float confidence);

/**
 * @brief Display message on OLED
 * @param line1 First line text
 * @param line2 Second line text (can be NULL)
 * @param line3 Third line text (can be NULL)
 */
void oled_show_message(const char *line1, const char *line2, const char *line3);

/**
 * @brief Display task - handles OLED updates
 * @param param Task parameter (unused)
 */
void oled_display_task(void *param);

#endif // OLED_DISPLAY_H
