/**
 * @file motor_control.c
 * @brief Motor control implementation for Voice-Controlled RC System
 */

#include "motor_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "failsafe.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "MOTOR";

// Current motor state
static motor_state_t s_motor_state = {
    .direction = MOTOR_DIR_STOP, .start_time_ms = 0, .is_active = false};

// GPIO pin list for bulk configuration
static const gpio_num_t s_motor_pins[] = {GPIO_MOTOR_EN_A, GPIO_MOTOR_EN_B,
                                          GPIO_MOTOR_IN1,  GPIO_MOTOR_IN2,
                                          GPIO_MOTOR_IN3,  GPIO_MOTOR_IN4};

/**
 * @brief Set motor GPIO pins directly
 */
static void motor_set_pins(int in1, int in2, int in3, int in4, int en) {
  gpio_set_level(GPIO_MOTOR_IN1, in1);
  gpio_set_level(GPIO_MOTOR_IN2, in2);
  gpio_set_level(GPIO_MOTOR_IN3, in3);
  gpio_set_level(GPIO_MOTOR_IN4, in4);
  gpio_set_level(GPIO_MOTOR_EN_A, en);
  gpio_set_level(GPIO_MOTOR_EN_B, en);
}

esp_err_t motor_init(void) {
  ESP_LOGI(TAG, "Initializing motor control...");

  // Configure all motor pins as outputs
  gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                           .mode = GPIO_MODE_OUTPUT,
                           .pin_bit_mask = 0,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .pull_up_en = GPIO_PULLUP_DISABLE};

  // Build pin mask
  for (int i = 0; i < sizeof(s_motor_pins) / sizeof(s_motor_pins[0]); i++) {
    io_conf.pin_bit_mask |= (1ULL << s_motor_pins[i]);
  }

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure motor GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Ensure motors are off at startup
  motor_emergency_stop();

  ESP_LOGI(TAG, "Motor control initialized - all motors OFF");
  return ESP_OK;
}

void motor_forward(void) {
  if (!failsafe_motors_allowed()) {
    ESP_LOGW(TAG, "Forward blocked: motors not allowed");
    return;
  }

  // Motor A: Forward, Motor B: Forward
  motor_set_pins(1, 0, 1, 0, 1);

  s_motor_state.direction = MOTOR_DIR_FORWARD;
  s_motor_state.start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_motor_state.is_active = true;

  failsafe_update_command_time();
  ESP_LOGI(TAG, "Motor: FORWARD");
}

void motor_backward(void) {
  if (!failsafe_motors_allowed()) {
    ESP_LOGW(TAG, "Backward blocked: motors not allowed");
    return;
  }

  // Motor A: Backward, Motor B: Backward
  motor_set_pins(0, 1, 0, 1, 1);

  s_motor_state.direction = MOTOR_DIR_BACKWARD;
  s_motor_state.start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_motor_state.is_active = true;

  failsafe_update_command_time();
  ESP_LOGI(TAG, "Motor: BACKWARD");
}

void motor_turn_left(void) {
  if (!failsafe_motors_allowed()) {
    ESP_LOGW(TAG, "Left blocked: motors not allowed");
    return;
  }

  // Motor A: Backward, Motor B: Forward (differential turn)
  motor_set_pins(0, 1, 1, 0, 1);

  s_motor_state.direction = MOTOR_DIR_LEFT;
  s_motor_state.start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_motor_state.is_active = true;

  failsafe_update_command_time();
  ESP_LOGI(TAG, "Motor: TURN LEFT");
}

void motor_turn_right(void) {
  if (!failsafe_motors_allowed()) {
    ESP_LOGW(TAG, "Right blocked: motors not allowed");
    return;
  }

  // Motor A: Forward, Motor B: Backward (differential turn)
  motor_set_pins(1, 0, 0, 1, 1);

  s_motor_state.direction = MOTOR_DIR_RIGHT;
  s_motor_state.start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_motor_state.is_active = true;

  failsafe_update_command_time();
  ESP_LOGI(TAG, "Motor: TURN RIGHT");
}

void motor_stop_all(void) {
  motor_set_pins(0, 0, 0, 0, 0);

  s_motor_state.direction = MOTOR_DIR_STOP;
  s_motor_state.is_active = false;

  ESP_LOGI(TAG, "Motor: STOP");
}

void motor_emergency_stop(void) {
  // Direct GPIO manipulation - no checks, immediate effect
  // This function must be safe to call from any context

  gpio_set_level(GPIO_MOTOR_EN_A, 0);
  gpio_set_level(GPIO_MOTOR_EN_B, 0);
  gpio_set_level(GPIO_MOTOR_IN1, 0);
  gpio_set_level(GPIO_MOTOR_IN2, 0);
  gpio_set_level(GPIO_MOTOR_IN3, 0);
  gpio_set_level(GPIO_MOTOR_IN4, 0);

  s_motor_state.direction = MOTOR_DIR_STOP;
  s_motor_state.is_active = false;

  // Update fail-safe state
  failsafe_disable_motors();

  ESP_LOGE(TAG, ">>> EMERGENCY STOP ACTIVATED <<<");
}

motor_state_t motor_get_state(void) { return s_motor_state; }

bool motor_is_active(void) { return s_motor_state.is_active; }

bool motor_verify_state(void) {
  // If motors should be off but enable pins are high, we have a mismatch
  if (!failsafe_motors_allowed()) {
    int en_a = gpio_get_level(GPIO_MOTOR_EN_A);
    int en_b = gpio_get_level(GPIO_MOTOR_EN_B);

    if (en_a || en_b) {
      ESP_LOGE(TAG, "State mismatch! Motors should be off but EN_A=%d, EN_B=%d",
               en_a, en_b);
      return false;
    }
  }

  return true;
}
