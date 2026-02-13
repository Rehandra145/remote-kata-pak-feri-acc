/**
 * @file button_input.c
 * @brief Push button input handling with debounce + double-press detection
 *
 * Detects single press and double-press on OK button.
 * Double-press OK (within 500ms) sends EVT_BTN_OK_DOUBLE to return to menu.
 */

#include "button_input.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTON";

// Button definitions
#define BTN_COUNT 3
#define BTN_OK_INDEX 2 // OK button is index 2

static const gpio_num_t s_buttons[BTN_COUNT] = {GPIO_BTN_UP, GPIO_BTN_DOWN,
                                                GPIO_BTN_OK};

// Debounce state
static uint32_t s_last_press_time[BTN_COUNT] = {0};
static bool s_last_state[BTN_COUNT] = {false};

// Double-press detection for OK button
#define DOUBLE_PRESS_INTERVAL_MS 300 // Snappier OK response
static uint32_t s_ok_first_press_time = 0;
static bool s_ok_waiting_second = false;

esp_err_t button_init(void) {
  ESP_LOGI(TAG, "Initializing buttons...");

  gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                           .mode = GPIO_MODE_INPUT,
                           .pin_bit_mask = 0,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .pull_up_en = GPIO_PULLUP_ENABLE};

  // Configure all button pins
  for (int i = 0; i < BTN_COUNT; i++) {
    io_conf.pin_bit_mask = (1ULL << s_buttons[i]);
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure GPIO %d", s_buttons[i]);
      return ret;
    }
    ESP_LOGI(TAG, "Button %d configured on GPIO %d", i, s_buttons[i]);
  }

  ESP_LOGI(TAG, "Buttons initialized successfully");
  return ESP_OK;
}

bool button_is_pressed(uint8_t button) {
  if (button >= BTN_COUNT)
    return false;
  // Active LOW (pressed = 0)
  return gpio_get_level(s_buttons[button]) == 0;
}

void button_task(void *param) {
  ESP_LOGI(TAG, "Button task started on core %d", xPortGetCoreID());

  const rc_event_type_t btn_events[BTN_COUNT] = {EVT_BTN_UP, EVT_BTN_DOWN,
                                                 EVT_BTN_OK};

  while (1) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < BTN_COUNT; i++) {
      bool pressed = button_is_pressed(i);

      // Check for rising edge (press) with debounce
      if (pressed && !s_last_state[i]) {
        if ((now - s_last_press_time[i]) >= BTN_DEBOUNCE_MS) {

          if (i == BTN_OK_INDEX) {
            // === OK BUTTON: Double-press detection ===
            if (s_ok_waiting_second &&
                (now - s_ok_first_press_time) < DOUBLE_PRESS_INTERVAL_MS) {
              // Double-press detected!
              ESP_LOGI(TAG, "OK DOUBLE-PRESS detected!");
              event_post_voice(EVT_BTN_OK_DOUBLE, 1.0f, 0);
              s_ok_waiting_second = false;
            } else {
              // First press — start waiting for second
              s_ok_first_press_time = now;
              s_ok_waiting_second = true;
            }
          } else {
            // UP/DOWN buttons: immediate single press
            ESP_LOGI(TAG, "Button %d pressed", i);
            event_post_voice(btn_events[i], 1.0f, 0);
          }

          s_last_press_time[i] = now;
        }
      }

      s_last_state[i] = pressed;
    }

    // Check if OK single-press timer expired (no second press came)
    if (s_ok_waiting_second &&
        (now - s_ok_first_press_time) >= DOUBLE_PRESS_INTERVAL_MS) {
      // Single press confirmed — send normal OK event
      ESP_LOGI(TAG, "OK single press confirmed");
      event_post_voice(EVT_BTN_OK, 1.0f, 0);
      s_ok_waiting_second = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Poll at 50Hz
  }
}
