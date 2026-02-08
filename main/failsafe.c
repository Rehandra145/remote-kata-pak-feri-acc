/**
 * @file failsafe.c
 * @brief Simplified fail-safe for Voice Display System (no motors)
 */

#include "failsafe.h"
#include "config.h"
#include "esp_log.h"
#include "event_queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FAILSAFE";

// Global fail-safe state
static failsafe_state_t s_failsafe = {.motors_allowed = false,
                                      .last_valid_command_ms = 0,
                                      .last_watchdog_feed_ms = 0,
                                      .system_healthy = true,
                                      .last_known_state = STATE_IDLE};

esp_err_t failsafe_init(void) {
  ESP_LOGI(TAG, "Initializing fail-safe subsystem...");
  failsafe_reset();
  ESP_LOGI(TAG, "Fail-safe initialized");
  return ESP_OK;
}

bool failsafe_motors_allowed(void) {
  return s_failsafe.motors_allowed && s_failsafe.system_healthy;
}

void failsafe_enable_motors(void) {
  if (!s_failsafe.system_healthy) {
    ESP_LOGW(TAG, "Cannot enable: system unhealthy");
    return;
  }
  s_failsafe.motors_allowed = true;
  s_failsafe.last_valid_command_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  ESP_LOGI(TAG, "Output ENABLED");
}

void failsafe_disable_motors(void) {
  s_failsafe.motors_allowed = false;
  ESP_LOGI(TAG, "Output DISABLED");
}

void failsafe_update_command_time(void) {
  s_failsafe.last_valid_command_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

bool failsafe_check_timeout(void) {
  if (!s_failsafe.motors_allowed) {
    return false;
  }

  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  uint32_t elapsed = now - s_failsafe.last_valid_command_ms;

  return (elapsed > TIMEOUT_COMMAND_ACTIVITY_MS);
}

bool failsafe_periodic_check(void) {
  bool healthy = true;

  // Check command timeout
  if (failsafe_check_timeout()) {
    ESP_LOGW(TAG, "Command timeout detected");
    event_post(EVT_TIMEOUT);
    healthy = false;
  }

  // Check system health flag
  if (!s_failsafe.system_healthy) {
    ESP_LOGE(TAG, "System flagged unhealthy");
    event_post(EVT_SYSTEM_ERROR);
    healthy = false;
  }

  return healthy;
}

void failsafe_set_health(bool healthy) {
  if (!healthy && s_failsafe.system_healthy) {
    ESP_LOGE(TAG, "System health set to UNHEALTHY");
  } else if (healthy && !s_failsafe.system_healthy) {
    ESP_LOGI(TAG, "System health restored to HEALTHY");
  }
  s_failsafe.system_healthy = healthy;
}

bool failsafe_is_healthy(void) { return s_failsafe.system_healthy; }

failsafe_state_t failsafe_get_state(void) { return s_failsafe; }

void failsafe_reset(void) {
  s_failsafe.motors_allowed = false;
  s_failsafe.last_valid_command_ms = 0;
  s_failsafe.last_watchdog_feed_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_failsafe.system_healthy = true;
  s_failsafe.last_known_state = STATE_IDLE;
  ESP_LOGI(TAG, "Fail-safe state reset");
}
