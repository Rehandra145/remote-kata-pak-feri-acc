/**
 * @file types.h
 * @brief Type definitions for Voice-Controlled RC System
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdbool.h>
#include <stdint.h>

/*===========================================================================
 * SYSTEM OPERATION MODE
 *===========================================================================*/

/**
 * @brief System operation mode for input isolation
 */
typedef enum {
  SYS_MODE_SELECTING, // Mode selection screen active
  SYS_MODE_VOICE,     // Voice mode (voice active, joystick ignored)
  SYS_MODE_REMOTE     // Remote mode (joystick active, voice disabled)
} system_mode_t;

/*===========================================================================
 * EVENT TYPES
 *===========================================================================*/

/**
 * @brief Event types that drive the state machine
 */
typedef enum {
  EVT_NONE = 0,

  // Voice command events
  EVT_VOICE_FORWARD,  // "forward" command recognized
  EVT_VOICE_BACKWARD, // "backward" command recognized
  EVT_VOICE_LEFT,     // "left" command recognized
  EVT_VOICE_RIGHT,    // "right" command recognized
  EVT_VOICE_STOP,     // "stop" command recognized

  // Wake word events
  EVT_WAKEWORD_DETECTED, // Wake word detected, listening active
  EVT_WAKEWORD_TIMEOUT,  // Wake word listening timed out

  // Button events
  EVT_BTN_UP,   // UP button pressed
  EVT_BTN_DOWN, // DOWN button pressed
  EVT_BTN_OK,   // OK button pressed

  // Joystick events (for Remote mode)
  EVT_JOY_FORWARD,  // Joystick pushed forward
  EVT_JOY_BACKWARD, // Joystick pushed backward
  EVT_JOY_LEFT,     // Joystick pushed left
  EVT_JOY_RIGHT,    // Joystick pushed right
  EVT_JOY_CENTER,   // Joystick centered/released

  // Mode events
  EVT_MODE_VOICE_SELECTED,  // Voice mode selected
  EVT_MODE_REMOTE_SELECTED, // Remote mode selected

  // System events
  EVT_TIMEOUT,        // No valid command within timeout window
  EVT_LOW_CONFIDENCE, // Recognition below threshold
  EVT_SYSTEM_ERROR,   // Any system error
  EVT_WATCHDOG_RESET, // Watchdog triggered

  EVT_MAX
} rc_event_type_t;

/**
 * @brief Event message structure for queue
 */
typedef struct {
  rc_event_type_t type;  // Event type
  uint32_t timestamp_ms; // Event timestamp
  float confidence;      // Recognition confidence (0.0-1.0)
  int raw_phrase_id;     // MultiNet phrase ID
  uint8_t priority;      // Event priority (lower = higher)
} rc_event_msg_t;

/*===========================================================================
 * STATE MACHINE STATES
 *===========================================================================*/

/**
 * @brief RC system states
 */
typedef enum {
  STATE_MODE_SELECT,   // Mode selection screen
  STATE_IDLE,          // Waiting for wake word (Voice mode)
  STATE_LISTENING,     // Wake word detected, listening for command
  STATE_MOVING_FWD,    // Moving forward
  STATE_MOVING_BWD,    // Moving backward
  STATE_TURNING_LEFT,  // Turning left
  STATE_TURNING_RIGHT, // Turning right
  STATE_STOPPED,       // Explicitly stopped (motors off)
  STATE_ERROR,         // Error state (motors off, recovery pending)
  STATE_MAX
} rc_state_t;

/*===========================================================================
 * FAIL-SAFE STRUCTURES
 *===========================================================================*/

/**
 * @brief Fail-safe state tracking
 */
typedef struct {
  bool motors_allowed;            // Global motor enable flag
  uint32_t last_valid_command_ms; // Last valid command timestamp
  uint32_t last_watchdog_feed_ms; // Last watchdog feed timestamp
  bool system_healthy;            // Overall system health flag
  rc_state_t last_known_state;    // Last known good state
} failsafe_state_t;

/*===========================================================================
 * RECOGNITION RESULT
 *===========================================================================*/

/**
 * @brief Validated recognition result
 */
typedef struct {
  bool is_valid;         // Whether recognition is valid
  rc_event_type_t event; // Mapped event type
  float confidence;      // Recognition confidence
  int phrase_id;         // Raw phrase ID
} recognition_result_t;

/*===========================================================================
 * MOTOR STATE
 *===========================================================================*/

/**
 * @brief Motor drive direction
 */
typedef enum {
  MOTOR_DIR_STOP = 0,
  MOTOR_DIR_FORWARD,
  MOTOR_DIR_BACKWARD,
  MOTOR_DIR_LEFT,
  MOTOR_DIR_RIGHT
} motor_direction_t;

/**
 * @brief Motor state information
 */
typedef struct {
  motor_direction_t direction;
  uint32_t start_time_ms;
  bool is_active;
} motor_state_t;

/*===========================================================================
 * UTILITY MACROS
 *===========================================================================*/

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// Time helpers
#define MS_TO_TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define TICKS_TO_MS(ticks) ((ticks) * portTICK_PERIOD_MS)

// State name lookup (for debugging)
static inline const char *state_name(rc_state_t state) {
  static const char *names[] = {"IDLE",       "LISTENING",    "MOVING_FWD",
                                "MOVING_BWD", "TURNING_LEFT", "TURNING_RIGHT",
                                "STOPPED",    "ERROR"};
  return (state < STATE_MAX) ? names[state] : "UNKNOWN";
}

// Event name lookup (for debugging)
static inline const char *event_name(rc_event_type_t evt) {
  static const char *names[] = {
      "NONE",        "VOICE_FORWARD",  "VOICE_BACKWARD",    "VOICE_LEFT",
      "VOICE_RIGHT", "VOICE_STOP",     "WAKEWORD_DETECTED", "WAKEWORD_TIMEOUT",
      "TIMEOUT",     "LOW_CONFIDENCE", "SYSTEM_ERROR",      "WATCHDOG_RESET"};
  return (evt < EVT_MAX) ? names[evt] : "UNKNOWN";
}

#endif // TYPES_H
