/**
 * @file mode_manager.c
 * @brief System mode management for input isolation
 */

#include "mode_manager.h"
#include "esp_log.h"
#include "lcd_display.h"

static const char *TAG = "MODE_MGR";

// Current mode
static system_mode_t s_current_mode = SYS_MODE_SELECTING;

// Menu selection (0=Voice, 1=Remote)
static uint8_t s_menu_selection = 0;

esp_err_t mode_manager_init(void) {
  ESP_LOGI(TAG, "Initializing mode manager...");

  s_current_mode = SYS_MODE_SELECTING;
  s_menu_selection = 0;

  // Show mode selection menu on LCD
  lcd_show_mode_menu(s_menu_selection);

  ESP_LOGI(TAG, "Mode manager initialized - showing mode selection");
  return ESP_OK;
}

system_mode_t mode_manager_get_mode(void) { return s_current_mode; }

void mode_manager_set_mode(system_mode_t mode) {
  if (mode != s_current_mode) {
    ESP_LOGI(TAG, "Mode changed: %d -> %d", s_current_mode, mode);
    s_current_mode = mode;

    // Update display based on mode
    switch (mode) {
    case SYS_MODE_SELECTING:
      lcd_show_mode_menu(s_menu_selection);
      break;
    case SYS_MODE_VOICE:
      lcd_show_message("MODE: VOICE", "SAY: HI ESP");
      break;
    case SYS_MODE_REMOTE:
      lcd_show_message("MODE: REMOTE", "USE JOYSTICK");
      break;
    }
  }
}

bool is_voice_input_allowed(void) { return s_current_mode == SYS_MODE_VOICE; }

bool is_joystick_input_allowed(void) {
  return s_current_mode == SYS_MODE_REMOTE;
}

bool mode_manager_handle_button(rc_event_type_t event) {
  // Only handle buttons in mode selection screen
  if (s_current_mode != SYS_MODE_SELECTING) {
    return false;
  }

  switch (event) {
  case EVT_BTN_UP:
    if (s_menu_selection > 0) {
      s_menu_selection--;
      lcd_show_mode_menu(s_menu_selection);
      ESP_LOGI(TAG, "Menu selection: %d", s_menu_selection);
    }
    return false;

  case EVT_BTN_DOWN:
    if (s_menu_selection < 1) {
      s_menu_selection++;
      lcd_show_mode_menu(s_menu_selection);
      ESP_LOGI(TAG, "Menu selection: %d", s_menu_selection);
    }
    return false;

  case EVT_BTN_OK:
    // Confirm selection
    if (s_menu_selection == 0) {
      mode_manager_set_mode(SYS_MODE_VOICE);
      ESP_LOGI(TAG, "Voice mode selected");
    } else {
      mode_manager_set_mode(SYS_MODE_REMOTE);
      ESP_LOGI(TAG, "Remote mode selected");
    }
    return true; // Mode changed

  default:
    return false;
  }
}

uint8_t mode_manager_get_selection(void) { return s_menu_selection; }
