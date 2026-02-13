/**
 * @file main.c
 * @brief Main entry point for Voice-Controlled System with LCD Display
 *
 * ESP32-S3 + ESP-SR + I2S Microphone + LCD 16x2 I2C
 * Supports two modes: Voice Control and Remote (Joystick) Control
 *
 * Architecture:
 * - Power On → Mode Selection (LCD + Buttons)
 * - Voice Mode: Voice input → Events → State Machine → LCD Display
 * - Remote Mode: Joystick → Events → State Machine → LCD Display
 *
 * Commands: "forward", "backward", "left", "right", "stop"
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>

#include "audio_task.h"
#include "button_input.h"
#include "config.h"
#include "event_queue.h"
#include "failsafe.h"
#include "joystick_input.h"
#include "lcd_display.h"
#include "mode_manager.h"
#include "state_machine.h"
#include "types.h"

static const char *TAG = "MAIN";

// Task handles for monitoring
static TaskHandle_t s_audio_task_handle = NULL;
static TaskHandle_t s_state_machine_handle = NULL;
static TaskHandle_t s_lcd_task_handle = NULL;
static TaskHandle_t s_watchdog_handle = NULL;
static TaskHandle_t s_status_led_handle = NULL;
static TaskHandle_t s_button_task_handle = NULL;
static TaskHandle_t s_joystick_task_handle = NULL;

/**
 * @brief Initialize status LED GPIO
 */
static esp_err_t init_status_led(void) {
  gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                           .mode = GPIO_MODE_OUTPUT,
                           .pin_bit_mask = (1ULL << GPIO_LED_STATUS),
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .pull_up_en = GPIO_PULLUP_DISABLE};

  esp_err_t ret = gpio_config(&io_conf);
  if (ret == ESP_OK) {
    gpio_set_level(GPIO_LED_STATUS, 0);
  }
  return ret;
}

/**
 * @brief Watchdog feeder task
 */
static void watchdog_feeder_task(void *param) {
  ESP_LOGI(TAG, "Watchdog feeder task started on core %d", xPortGetCoreID());

  while (1) {
    if (failsafe_is_healthy()) {
      // System healthy
    } else {
      ESP_LOGW(TAG, "System unhealthy - watchdog not fed");
    }

    vTaskDelay(pdMS_TO_TICKS(WATCHDOG_FEED_INTERVAL_MS));
  }
}

/**
 * @brief Status LED task - visual feedback based on current mode and state
 */
static void status_led_task(void *param) {
  ESP_LOGI(TAG, "Status LED task started on core %d", xPortGetCoreID());

  uint32_t blink_interval_ms = 1000;

  while (1) {
    system_mode_t mode = mode_manager_get_mode();
    rc_state_t state = state_machine_get_state();

    // Different patterns for different modes
    if (mode == SYS_MODE_SELECTING) {
      blink_interval_ms = 300; // Fast blink during mode selection
    } else if (mode == SYS_MODE_VOICE) {
      switch (state) {
      case STATE_IDLE:
        blink_interval_ms = 2000;
        break;
      case STATE_LISTENING:
        blink_interval_ms = 200;
        break;
      case STATE_MOVING_FWD:
      case STATE_MOVING_BWD:
      case STATE_TURNING_LEFT:
      case STATE_TURNING_RIGHT:
        blink_interval_ms = 100;
        break;
      case STATE_STOPPED:
        blink_interval_ms = 500;
        break;
      case STATE_ERROR:
        blink_interval_ms = 50;
        break;
      default:
        blink_interval_ms = 1000;
        break;
      }
    } else if (mode == SYS_MODE_REMOTE) {
      // Solid slow blink for remote mode
      blink_interval_ms = 1500;
    }

    gpio_set_level(GPIO_LED_STATUS, 1);
    vTaskDelay(pdMS_TO_TICKS(blink_interval_ms / 2));
    gpio_set_level(GPIO_LED_STATUS, 0);
    vTaskDelay(pdMS_TO_TICKS(blink_interval_ms / 2));
  }
}

/**
 * @brief Initialize hardware watchdog timer
 */
static esp_err_t init_watchdog(void) {
  ESP_LOGI(TAG, "Initializing hardware watchdog (%d ms timeout)...",
           WATCHDOG_TIMEOUT_MS);

  esp_task_wdt_config_t wdt_config = {.timeout_ms = WATCHDOG_TIMEOUT_MS,
                                      .trigger_panic = true,
                                      .idle_core_mask = 0};

  esp_err_t ret = esp_task_wdt_reconfigure(&wdt_config);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG,
             "Watchdog reconfigure returned: %s (may already be configured)",
             esp_err_to_name(ret));
  }

  return ESP_OK;
}

/**
 * @brief Print system information
 */
static void print_system_info(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, " %s v%s", PROJECT_NAME, PROJECT_VERSION);
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Platform: ESP32-S3");
  ESP_LOGI(TAG, "Framework: ESP-IDF + ESP-SR");
  ESP_LOGI(TAG, "Display: LCD 16x2 I2C");
  ESP_LOGI(TAG, "Input: Voice + Joystick + Buttons");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Modes:");
  ESP_LOGI(TAG, "  [VOICE]  - Voice commands (wake word + command)");
  ESP_LOGI(TAG, "  [REMOTE] - Joystick control");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "LCD Pins: SDA=GPIO%d, SCL=GPIO%d", GPIO_LCD_SDA, GPIO_LCD_SCL);
  ESP_LOGI(TAG, "Buttons: UP=GPIO%d, DOWN=GPIO%d, OK=GPIO%d", GPIO_BTN_UP,
           GPIO_BTN_DOWN, GPIO_BTN_OK);
  ESP_LOGI(TAG, "Joystick: X=GPIO%d, Y=GPIO%d, SW=GPIO%d", GPIO_JOY_X,
           GPIO_JOY_Y, GPIO_JOY_SW);
  ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Application entry point
 */
void app_main(void) {
  esp_err_t ret;

  print_system_info();

  // 1. Initialize NVS
  ESP_LOGI(TAG, "Initializing NVS...");
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 2. Initialize hardware
  ESP_LOGI(TAG, "Initializing hardware...");

  ret = init_status_led();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Status LED init failed (non-critical)");
  }

  // Initialize LCD 16x2 I2C display
  ret = lcd_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(ret));
    // Continue anyway - system can work without display
  }

  // Initialize buttons
  ret = button_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Button init failed: %s", esp_err_to_name(ret));
  }

  // Initialize joystick
  ret = joystick_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Joystick init failed: %s", esp_err_to_name(ret));
  }

  // 3. Initialize software subsystems
  ESP_LOGI(TAG, "Initializing software...");

  ret = event_queue_init();
  ESP_ERROR_CHECK(ret);

  ret = failsafe_init();
  ESP_ERROR_CHECK(ret);

  ret = state_machine_init();
  ESP_ERROR_CHECK(ret);

  // === FAST BOOT: Create LCD task FIRST so it can render during init ===
  ESP_LOGI(TAG, "Creating tasks...");

  // LCD Display Task - CREATED FIRST for visual feedback during boot
  xTaskCreatePinnedToCore(lcd_display_task, "lcd_display", STACK_SIZE_OLED,
                          NULL, PRIORITY_STATUS_LED + 1, &s_lcd_task_handle,
                          CORE_SAFETY_CRITICAL);

  // Small delay to let LCD task start and render startup message
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialize mode manager (shows mode selection menu IMMEDIATELY)
  ret = mode_manager_init();
  ESP_ERROR_CHECK(ret);

  // === HEAVY INIT: Audio moved to its own task (non-blocking) ===
  // audio_init() is now called inside audio_processing_task
  // This saves ~3-4 seconds of blocking boot time

  // 5. Initialize watchdog
  ret = init_watchdog();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Watchdog init warning (continuing)");
  }

  // 6. Create remaining tasks
  // Button Task
  xTaskCreatePinnedToCore(button_task, "button_input", 2048, NULL,
                          PRIORITY_STATUS_LED + 1, &s_button_task_handle,
                          CORE_SAFETY_CRITICAL);

  // Joystick Task (calibration happens inside the task now)
  xTaskCreatePinnedToCore(joystick_task, "joystick_input", 4096, NULL,
                          PRIORITY_STATUS_LED + 1, &s_joystick_task_handle,
                          CORE_SAFETY_CRITICAL);

  // State Machine Task
  xTaskCreatePinnedToCore(
      state_machine_task, "state_machine", STACK_SIZE_STATE_MACHINE, NULL,
      PRIORITY_STATE_MACHINE, &s_state_machine_handle, CORE_SAFETY_CRITICAL);

  // Audio Processing Task (will call audio_init internally)
  xTaskCreatePinnedToCore(
      audio_processing_task, "audio_proc", STACK_SIZE_AUDIO_TASK, NULL,
      PRIORITY_AUDIO_PROCESSING, &s_audio_task_handle, CORE_AUDIO_PROCESSING);

  // Watchdog Feeder Task
  xTaskCreatePinnedToCore(watchdog_feeder_task, "wdt_feed", STACK_SIZE_WATCHDOG,
                          NULL, PRIORITY_WATCHDOG_FEEDER, &s_watchdog_handle,
                          CORE_SAFETY_CRITICAL);

  // Status LED Task
  xTaskCreatePinnedToCore(status_led_task, "status_led", STACK_SIZE_STATUS_LED,
                          NULL, PRIORITY_STATUS_LED, &s_status_led_handle,
                          CORE_SAFETY_CRITICAL);

  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, " All tasks started. System ready.");
  ESP_LOGI(TAG, " Select mode using UP/DOWN/OK buttons.");
  ESP_LOGI(TAG, "========================================");

  // Main task idle loop
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Periodic health report
    ESP_LOGI(TAG, "Health: Mode=%d, State=%s, Queue=%lu",
             mode_manager_get_mode(), state_name(state_machine_get_state()),
             event_queue_pending());
  }
}
