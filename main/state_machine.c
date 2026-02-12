/**
 * @file state_machine.c
 * @brief State machine for Voice-Controlled System with LCD Display
 *
 * Supports two modes: Voice and Remote (Joystick)
 * Button events handled for mode selection
 * Voice commands and joystick events handled based on current mode
 */

#include "state_machine.h"
#include "config.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "espnow_comm.h"
#include "event_queue.h"
#include "failsafe.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"
#include "mode_manager.h"

static const char *TAG = "STATE_MACHINE";

// Current state
static rc_state_t s_current_state = STATE_MODE_SELECT;

// Forward declarations
static rc_state_t process_event_internal(rc_state_t current,
                                         const rc_event_msg_t *event);
static void state_entry_action(rc_state_t state, const rc_event_msg_t *event);
static void state_exit_action(rc_state_t state);

// Get command name from event type
static const char *get_command_name(rc_event_type_t type) {
  switch (type) {
  case EVT_VOICE_FORWARD:
  case EVT_JOY_FORWARD:
    return "FORWARD";
  case EVT_VOICE_BACKWARD:
  case EVT_JOY_BACKWARD:
    return "BACKWARD";
  case EVT_VOICE_LEFT:
  case EVT_JOY_LEFT:
    return "LEFT";
  case EVT_VOICE_RIGHT:
  case EVT_JOY_RIGHT:
    return "RIGHT";
  case EVT_VOICE_STOP:
  case EVT_JOY_CENTER:
    return "STOP";
  default:
    return "UNKNOWN";
  }
}

esp_err_t state_machine_init(void) {
  ESP_LOGI(TAG, "Initializing state machine...");

  // Initialize ESP-NOW
  if (espnow_comm_init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
  }

  s_current_state = STATE_MODE_SELECT;
  failsafe_disable_motors();
  ESP_LOGI(TAG, "State machine initialized - starting in MODE_SELECT state");
  return ESP_OK;
}

rc_state_t state_machine_get_state(void) { return s_current_state; }

void state_machine_force_state(rc_state_t state) {
  ESP_LOGW(TAG, "Forcing state transition: %s -> %s",
           state_name(s_current_state), state_name(state));
  state_exit_action(s_current_state);
  s_current_state = state;
  state_entry_action(state, NULL);
}

rc_state_t state_machine_process_event(const rc_event_msg_t *event) {
  if (event == NULL) {
    return s_current_state;
  }

  ESP_LOGI(TAG, "Event: %s | State: %s | Mode: %d", event_name(event->type),
           state_name(s_current_state), mode_manager_get_mode());

  // Handle button events for mode selection first
  if (event->type == EVT_BTN_UP || event->type == EVT_BTN_DOWN ||
      event->type == EVT_BTN_OK) {
    if (mode_manager_handle_button(event->type)) {
      // Mode changed - transition to appropriate state
      system_mode_t mode = mode_manager_get_mode();
      if (mode == SYS_MODE_VOICE) {
        state_machine_force_state(STATE_IDLE);
      } else if (mode == SYS_MODE_REMOTE) {
        state_machine_force_state(STATE_STOPPED);
      }
    }
    return s_current_state;
  }

  rc_state_t next_state = process_event_internal(s_current_state, event);

  if (next_state != s_current_state) {
    ESP_LOGI(TAG, "Transition: %s -> %s", state_name(s_current_state),
             state_name(next_state));
    state_exit_action(s_current_state);
    s_current_state = next_state;
    state_entry_action(next_state, event);
  }

  return s_current_state;
}

/**
 * @brief Core state transition logic
 */
static rc_state_t process_event_internal(rc_state_t current,
                                         const rc_event_msg_t *event) {
  system_mode_t mode = mode_manager_get_mode();

  // Global error handling
  if (event->type == EVT_SYSTEM_ERROR || event->type == EVT_WATCHDOG_RESET) {
    return STATE_ERROR;
  }

  // In mode selection, don't process other events
  if (mode == SYS_MODE_SELECTING) {
    return current;
  }

  // Voice mode handling
  if (mode == SYS_MODE_VOICE) {
    switch (current) {
    case STATE_IDLE:
      if (event->type == EVT_WAKEWORD_DETECTED) {
        return STATE_LISTENING;
      }
      break;

    case STATE_LISTENING:
      switch (event->type) {
      case EVT_VOICE_FORWARD:
        return STATE_MOVING_FWD;
      case EVT_VOICE_BACKWARD:
        return STATE_MOVING_BWD;
      case EVT_VOICE_LEFT:
        return STATE_TURNING_LEFT;
      case EVT_VOICE_RIGHT:
        return STATE_TURNING_RIGHT;
      case EVT_VOICE_STOP:
        return STATE_STOPPED;
      case EVT_TIMEOUT:
        return STATE_STOPPED;
      case EVT_WAKEWORD_TIMEOUT:
        return STATE_IDLE;
      case EVT_LOW_CONFIDENCE:
        return STATE_LISTENING;
      default:
        break;
      }
      break;

    case STATE_MOVING_FWD:
    case STATE_MOVING_BWD:
    case STATE_TURNING_LEFT:
    case STATE_TURNING_RIGHT:
      switch (event->type) {
      case EVT_VOICE_STOP:
        return STATE_STOPPED;
      case EVT_TIMEOUT:
        return STATE_STOPPED;
      case EVT_VOICE_FORWARD:
        return STATE_MOVING_FWD;
      case EVT_VOICE_BACKWARD:
        return STATE_MOVING_BWD;
      case EVT_VOICE_LEFT:
        return STATE_TURNING_LEFT;
      case EVT_VOICE_RIGHT:
        return STATE_TURNING_RIGHT;
      case EVT_WAKEWORD_DETECTED:
        return current;
      case EVT_LOW_CONFIDENCE:
        return current;
      default:
        break;
      }
      break;

    case STATE_STOPPED:
      switch (event->type) {
      case EVT_WAKEWORD_DETECTED:
        return STATE_LISTENING;
      case EVT_TIMEOUT:
        return STATE_IDLE;
      default:
        break;
      }
      break;

    case STATE_ERROR:
      return STATE_STOPPED;

    default:
      break;
    }
  }

  // Remote (Joystick) mode handling
  if (mode == SYS_MODE_REMOTE) {
    switch (event->type) {
    case EVT_JOY_FORWARD:
      return STATE_MOVING_FWD;
    case EVT_JOY_BACKWARD:
      return STATE_MOVING_BWD;
    case EVT_JOY_LEFT:
      return STATE_TURNING_LEFT;
    case EVT_JOY_RIGHT:
      return STATE_TURNING_RIGHT;
    case EVT_JOY_CENTER:
      return STATE_STOPPED;
    default:
      break;
    }
  }

  return current;
}

/**
 * @brief Actions to execute when entering a state
 */
static void state_entry_action(rc_state_t state, const rc_event_msg_t *event) {
  system_mode_t mode = mode_manager_get_mode();
  const char *mode_prefix = (mode == SYS_MODE_VOICE) ? "[VOICE]" : "[REMOTE]";

  switch (state) {
  case STATE_MODE_SELECT:
    failsafe_disable_motors();
    // Mode manager already shows the menu
    ESP_LOGI(TAG, "Entry MODE_SELECT");
    break;

  case STATE_IDLE:
    failsafe_disable_motors();
    lcd_show_message("VOICE MODE", "SAY: HI ESP");
    ESP_LOGI(TAG, "Entry IDLE: Waiting for wake word");
    break;

  case STATE_LISTENING:
    failsafe_update_command_time();
    lcd_show_message("LISTENING...", "SAY COMMAND");
    ESP_LOGI(TAG, "Entry LISTENING: Waiting for voice command");
    break;

  case STATE_MOVING_FWD:
    failsafe_enable_motors();
    failsafe_update_command_time();
    // Voice mode: kirim via state machine (diskrit)
    // Remote mode: joystick_task kirim langsung (continuous), skip disini
    if (mode == SYS_MODE_VOICE) {
      espnow_send_event(EVT_VOICE_FORWARD);
    }
    if (mode == SYS_MODE_VOICE && event) {
      lcd_show_command("FORWARD", event->confidence);
    } else {
      lcd_show_message(mode_prefix, "FORWARD");
    }
    break;

  case STATE_MOVING_BWD:
    failsafe_enable_motors();
    failsafe_update_command_time();
    if (mode == SYS_MODE_VOICE) {
      espnow_send_event(EVT_VOICE_BACKWARD);
    }
    if (mode == SYS_MODE_VOICE && event) {
      lcd_show_command("BACKWARD", event->confidence);
    } else {
      lcd_show_message(mode_prefix, "BACKWARD");
    }
    break;

  case STATE_TURNING_LEFT:
    failsafe_enable_motors();
    failsafe_update_command_time();
    if (mode == SYS_MODE_VOICE) {
      espnow_send_event(EVT_VOICE_LEFT);
    }
    if (mode == SYS_MODE_VOICE && event) {
      lcd_show_command("LEFT", event->confidence);
    } else {
      lcd_show_message(mode_prefix, "LEFT");
    }
    break;

  case STATE_TURNING_RIGHT:
    failsafe_enable_motors();
    failsafe_update_command_time();
    if (mode == SYS_MODE_VOICE) {
      espnow_send_event(EVT_VOICE_RIGHT);
    }
    if (mode == SYS_MODE_VOICE && event) {
      lcd_show_command("RIGHT", event->confidence);
    } else {
      lcd_show_message(mode_prefix, "RIGHT");
    }
    break;

  case STATE_STOPPED:
    failsafe_disable_motors();
    // Voice mode: kirim STOP via state machine
    // Remote mode: joystick_task sudah kirim STOP continuous
    if (mode == SYS_MODE_VOICE) {
      espnow_send_event(EVT_VOICE_STOP);
    }
    if (mode == SYS_MODE_VOICE) {
      lcd_show_message("STOPPED", "SAY: HI ESP");
    } else {
      lcd_show_message("REMOTE MODE", "USE JOYSTICK");
    }
    ESP_LOGI(TAG, "Entry STOPPED");
    break;

  case STATE_ERROR:
    failsafe_set_health(false);
    espnow_send_emergency_stop();
    lcd_show_message("ERROR!", "RECOVERING...");
    ESP_LOGE(TAG, "Entry ERROR");
    vTaskDelay(pdMS_TO_TICKS(100));
    failsafe_set_health(true);
    break;

  default:
    break;
  }

  // Update LCD state display
  lcd_show_state(state);
}

/**
 * @brief Actions to execute when exiting a state
 */
static void state_exit_action(rc_state_t state) {
  ESP_LOGD(TAG, "Exit state: %s", state_name(state));
}

/**
 * @brief State machine FreeRTOS task
 */
void state_machine_task(void *param) {
  ESP_LOGI(TAG, "State machine task started on core %d", xPortGetCoreID());

  rc_event_msg_t event;

  while (1) {
    if (event_receive(&event, EVENT_QUEUE_WAIT_MS)) {
      state_machine_process_event(&event);
    }
  }
}
