/**
 * @file lcd_display.h
 * @brief LCD 16x2 I2C Display interface
 *
 * Uses HD44780 compatible LCD with PCF8574 I2C expander
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "esp_err.h"
#include "types.h"

/**
 * @brief Initialize LCD display
 * Auto-detects I2C address (0x27 or 0x3F)
 * @return ESP_OK on success
 */
esp_err_t lcd_init(void);

/**
 * @brief Clear the LCD display
 */
void lcd_clear(void);

/**
 * @brief Set cursor position
 * @param row Row (0 or 1)
 * @param col Column (0-15)
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Print string at current cursor position
 * @param str String to print
 */
void lcd_print(const char *str);

/**
 * @brief Print formatted string
 * @param fmt Format string
 * @param ... Arguments
 */
void lcd_printf(const char *fmt, ...);

/**
 * @brief Control backlight
 * @param on true=on, false=off
 */
void lcd_backlight(bool on);

/**
 * @brief Show message on two lines (centered)
 * @param line1 First line text
 * @param line2 Second line text
 */
void lcd_show_message(const char *line1, const char *line2);

/**
 * @brief Show mode selection menu
 * @param selected_mode Currently selected mode (0=Voice, 1=Remote)
 */
void lcd_show_mode_menu(uint8_t selected_mode);

/**
 * @brief Show current state
 * @param state System state
 */
void lcd_show_state(rc_state_t state);

/**
 * @brief Show voice command with confidence
 * @param command Command name
 * @param confidence Recognition confidence (0.0-1.0)
 */
void lcd_show_command(const char *command, float confidence);

/**
 * @brief Show joystick direction
 * @param direction Direction string
 */
void lcd_show_joystick(const char *direction);

/**
 * @brief LCD display task for FreeRTOS
 * @param param Task parameter (unused)
 */
void lcd_display_task(void *param);

#endif // LCD_DISPLAY_H
