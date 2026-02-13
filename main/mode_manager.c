/**
 * @file mode_manager.c
 * @brief System mode management with settings menu
 *
 * Main Menu: Voice / Remote / Settings
 * Settings: Voice command duration (500ms - 5000ms)
 * Double-press OK: return to menu from any mode
 */

#include "mode_manager.h"
#include "esp_log.h"
#include "lcd_display.h"

static const char *TAG = "MODE_MGR";

// Current mode
static system_mode_t s_current_mode = SYS_MODE_SELECTING;

// Menu selection (0=Voice, 1=Remote, 2=Settings)
#define MENU_ITEM_COUNT 3
static uint8_t s_menu_selection = 0;

// Settings state
static bool s_in_settings = false;

// Voice command duration (500ms - 5000ms, step 500ms)
#define DURATION_MIN_MS 500
#define DURATION_MAX_MS 5000
#define DURATION_STEP_MS 500
#define DURATION_DEFAULT_MS 2000
static uint16_t s_voice_duration_ms = DURATION_DEFAULT_MS;

esp_err_t mode_manager_init(void) {
  ESP_LOGI(TAG, "Initializing mode manager...");

  s_current_mode = SYS_MODE_SELECTING;
  s_menu_selection = 0;
  s_in_settings = false;

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
      s_in_settings = false;
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

uint16_t mode_manager_get_voice_duration(void) { return s_voice_duration_ms; }

void mode_manager_return_to_menu(void) {
  ESP_LOGI(TAG, "Returning to menu (double-press OK)");
  s_in_settings = false;
  s_menu_selection = 0;
  s_current_mode = SYS_MODE_SELECTING;
  lcd_show_mode_menu(s_menu_selection);
}

/**
 * @brief Handle button events in settings sub-menu
 */
static bool handle_settings_button(rc_event_type_t event) {
  switch (event) {
  case EVT_BTN_UP:
    // Increase duration
    if (s_voice_duration_ms < DURATION_MAX_MS) {
      s_voice_duration_ms += DURATION_STEP_MS;
      ESP_LOGI(TAG, "Duration: %d ms", s_voice_duration_ms);
      lcd_show_settings(s_voice_duration_ms);
    }
    return false;

  case EVT_BTN_DOWN:
    // Decrease duration
    if (s_voice_duration_ms > DURATION_MIN_MS) {
      s_voice_duration_ms -= DURATION_STEP_MS;
      ESP_LOGI(TAG, "Duration: %d ms", s_voice_duration_ms);
      lcd_show_settings(s_voice_duration_ms);
    }
    return false;

  case EVT_BTN_OK:
    // Confirm and go back to main menu
    ESP_LOGI(TAG, "Duration saved: %d ms", s_voice_duration_ms);
    s_in_settings = false;
    lcd_show_message("SAVED!", "DURATION SET");
    // Small delay then return to menu
    lcd_show_mode_menu(s_menu_selection);
    return false;

  default:
    return false;
  }
}

bool mode_manager_handle_button(rc_event_type_t event) {
  // Double-press OK: return to menu from ANY mode
  if (event == EVT_BTN_OK_DOUBLE) {
    mode_manager_return_to_menu();
    return true; // Signal mode changed
  }

  // Only handle other buttons in mode selection/settings screen
  if (s_current_mode != SYS_MODE_SELECTING) {
    return false;
  }

  // If in settings sub-menu, delegate to settings handler
  if (s_in_settings) {
    return handle_settings_button(event);
  }

  // Main menu handling
  switch (event) {
  case EVT_BTN_UP:
    if (s_menu_selection > 0) {
      s_menu_selection--;
      lcd_show_mode_menu(s_menu_selection);
      ESP_LOGI(TAG, "Menu selection: %d", s_menu_selection);
    }
    return false;

  case EVT_BTN_DOWN:
    if (s_menu_selection < MENU_ITEM_COUNT - 1) {
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
      return true; // Mode changed
    } else if (s_menu_selection == 1) {
      mode_manager_set_mode(SYS_MODE_REMOTE);
      ESP_LOGI(TAG, "Remote mode selected");
      return true; // Mode changed
    } else if (s_menu_selection == 2) {
      // Enter settings sub-menu
      s_in_settings = true;
      lcd_show_settings(s_voice_duration_ms);
      ESP_LOGI(TAG, "Entered settings");
      return false; // Mode NOT changed, still selecting
    }
    return false;

  default:
    return false;
  }
}

uint8_t mode_manager_get_selection(void) { return s_menu_selection; }
