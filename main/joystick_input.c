/**
 * @file joystick_input.c
 * @brief Analog joystick input handling with AUTO-CALIBRATION
 *
 * MODE CONTINUOUS: Joystick mengirim sinyal ESP-NOW TERUS-MENERUS
 * setiap 50ms (20Hz), bukan hanya saat berubah arah.
 * Ini memastikan receiver berhenti ketika transmitter mati.
 *
 * MODE VOICE tetap diskrit (event-based) via state_machine.
 */

#include "joystick_input.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "espnow_comm.h"
#include "event_queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "lcd_display.h"
#include "mode_manager.h"

static const char *TAG = "JOYSTICK";

// ADC handle
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

// ADC channels (ESP32-S3 GPIO to ADC channel mapping)
// GPIO10 = ADC1_CH9, GPIO3 = ADC1_CH2
#define ADC_CHANNEL_X ADC_CHANNEL_9 // GPIO10 - Steering (kiri/kanan)
#define ADC_CHANNEL_Y ADC_CHANNEL_2 // GPIO3  - Throttle (maju/mundur)

// Threshold untuk deteksi arah (dari mapped value -255 to 255)
#define JOY_DIRECTION_THRESHOLD 50

// Calibrated center values (diisi saat kalibrasi)
static int s_center_x = 2048;
static int s_center_y = 2048;

// Last direction for edge detection (LCD update only)
static joystick_dir_t s_last_dir = JOY_DIR_CENTER;

/**
 * @brief Map value dengan center point yang dikalibrasi
 * @param value Nilai ADC raw (sudah di-invert jika perlu)
 * @param center Nilai center hasil kalibrasi
 * @return Nilai -255 sampai +255
 */
static int16_t map_with_center(int value, int center) {
  int result;

  if (value < center) {
    // Di bawah center -> nilai negatif
    if (center == 0)
      return 0;
    result = (value - center) * 255 / center;
  } else {
    // Di atas center -> nilai positif
    if (center >= 4095)
      return 0;
    result = (value - center) * 255 / (4095 - center);
  }

  // Aplikasikan deadzone
  if (abs(result) < JOY_DEADZONE) {
    return 0;
  }

  // Constrain ke range valid
  if (result > 255)
    result = 255;
  if (result < -255)
    result = -255;

  return (int16_t)result;
}

/**
 * @brief Convert joystick direction to ESP-NOW command
 */
static espnow_cmd_t dir_to_espnow_cmd(joystick_dir_t dir) {
  switch (dir) {
  case JOY_DIR_FORWARD:
    return ESPNOW_CMD_FORWARD;
  case JOY_DIR_BACKWARD:
    return ESPNOW_CMD_BACKWARD;
  case JOY_DIR_LEFT:
    return ESPNOW_CMD_LEFT;
  case JOY_DIR_RIGHT:
    return ESPNOW_CMD_RIGHT;
  case JOY_DIR_CENTER:
  default:
    return ESPNOW_CMD_STOP;
  }
}

/**
 * @brief Perform joystick auto-calibration
 * Reads multiple samples at rest position and averages them
 */
static void calibrate_joystick(void) {
  ESP_LOGI(TAG, "=====================================");
  ESP_LOGI(TAG, ">>> KALIBRASI: Jangan sentuh joystick...");
  ESP_LOGI(TAG, "=====================================");

  // Show calibration message on LCD
  lcd_show_message("KALIBRASI", "JANGAN SENTUH!");

  // Let joystick settle
  vTaskDelay(pdMS_TO_TICKS(500));

  // Take multiple samples
  long sum_x = 0, sum_y = 0;
  int raw_x, raw_y;

  for (int i = 0; i < JOY_CALIBRATION_SAMPLES; i++) {
    adc_oneshot_read(s_adc_handle, ADC_CHANNEL_X, &raw_x);
    adc_oneshot_read(s_adc_handle, ADC_CHANNEL_Y, &raw_y);

    // Apply invert BEFORE summing
    sum_x += INVERT_JOY_X ? (4095 - raw_x) : raw_x;
    sum_y += INVERT_JOY_Y ? (4095 - raw_y) : raw_y;

    vTaskDelay(pdMS_TO_TICKS(JOY_CALIBRATION_DELAY_MS));
  }

  // Calculate average
  s_center_x = sum_x / JOY_CALIBRATION_SAMPLES;
  s_center_y = sum_y / JOY_CALIBRATION_SAMPLES;

  ESP_LOGI(TAG, ">>> KALIBRASI SELESAI!");
  ESP_LOGI(TAG, "  Center: X=%d, Y=%d", s_center_x, s_center_y);
  ESP_LOGI(TAG, "  Invert: X=%s, Y=%s", INVERT_JOY_X ? "true" : "false",
           INVERT_JOY_Y ? "true" : "false");
  ESP_LOGI(TAG, "=====================================");

  // Show calibration complete
  lcd_show_message("KALIBRASI OK!", "CENTER SAVED");
  vTaskDelay(pdMS_TO_TICKS(500));
}

esp_err_t joystick_init(void) {
  ESP_LOGI(TAG, "Initializing joystick...");

  // Configure ADC
  adc_oneshot_unit_init_cfg_t adc_cfg = {
      .unit_id = ADC_UNIT_1,
  };

  esp_err_t ret = adc_oneshot_new_unit(&adc_cfg, &s_adc_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Configure channels with 12-bit resolution and full range attenuation
  adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12,
                                     .bitwidth = ADC_BITWIDTH_12};

  ret = adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_X, &chan_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel X config failed");
    return ret;
  }

  ret = adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_Y, &chan_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel Y config failed");
    return ret;
  }

  // Configure joystick button
  gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                           .mode = GPIO_MODE_INPUT,
                           .pin_bit_mask = (1ULL << GPIO_JOY_SW),
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .pull_up_en = GPIO_PULLUP_ENABLE};

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Joystick button GPIO config failed");
    return ret;
  }

  // Perform auto-calibration
  calibrate_joystick();

  ESP_LOGI(TAG, "Joystick initialized successfully");
  return ESP_OK;
}

joystick_dir_t joystick_get_direction(void) {
  if (s_adc_handle == NULL)
    return JOY_DIR_CENTER;

  int raw_x = 0, raw_y = 0;

  adc_oneshot_read(s_adc_handle, ADC_CHANNEL_X, &raw_x);
  adc_oneshot_read(s_adc_handle, ADC_CHANNEL_Y, &raw_y);

  // Apply invert (sama seperti saat kalibrasi)
  int joy_x = INVERT_JOY_X ? (4095 - raw_x) : raw_x;
  int joy_y = INVERT_JOY_Y ? (4095 - raw_y) : raw_y;

  // Map dengan kalibrasi: Y = throttle, X = steering
  int16_t throttle = map_with_center(joy_y, s_center_y); // maju/mundur
  int16_t steering = map_with_center(joy_x, s_center_x); // kiri/kanan

  // DEBUG LOG setiap 1 detik
  static int debug_counter = 0;
  if (++debug_counter % 20 == 0) {
    ESP_LOGI(TAG,
             "RAW: X=%d Y=%d | INV: X=%d Y=%d | MAP: T=%d S=%d | CEN: X=%d "
             "Y=%d",
             raw_x, raw_y, joy_x, joy_y, throttle, steering, s_center_x,
             s_center_y);
  }

  // Prioritaskan axis yang lebih dominan
  if (abs(throttle) > abs(steering)) {
    // Y axis dominant (maju/mundur)
    if (throttle > JOY_DIRECTION_THRESHOLD)
      return JOY_DIR_FORWARD;
    if (throttle < -JOY_DIRECTION_THRESHOLD)
      return JOY_DIR_BACKWARD;
  } else {
    // X axis dominant (kiri/kanan)
    if (steering > JOY_DIRECTION_THRESHOLD)
      return JOY_DIR_RIGHT;
    if (steering < -JOY_DIRECTION_THRESHOLD)
      return JOY_DIR_LEFT;
  }

  return JOY_DIR_CENTER;
}

bool joystick_button_pressed(void) {
  // Active LOW
  return gpio_get_level(GPIO_JOY_SW) == 0;
}

/**
 * @brief Joystick task - MODE CONTINUOUS
 *
 * Perbedaan dengan mode diskrit:
 * - SELALU kirim ESP-NOW command setiap 50ms (20Hz), bukan hanya saat berubah
 * - Ketika joystick di center → kirim STOP terus-menerus
 * - Ketika transmitter mati → receiver tidak dapat sinyal → otomatis berhenti
 * - LCD hanya update saat direction berubah (hemat I2C bandwidth)
 */
void joystick_task(void *param) {
  ESP_LOGI(TAG, "Joystick task started (CONTINUOUS MODE) on core %d",
           xPortGetCoreID());

  const char *dir_names[] = {"CENTER", "FORWARD", "BACKWARD", "LEFT", "RIGHT"};

  while (1) {
    // Only process joystick in Remote mode
    if (is_joystick_input_allowed()) {
      joystick_dir_t dir = joystick_get_direction();

      // === CONTINUOUS: Kirim ESP-NOW ASYNC (non-blocking!) ===
      espnow_cmd_t cmd = dir_to_espnow_cmd(dir);
      uint8_t speed = (dir == JOY_DIR_CENTER) ? 0 : 200;
      espnow_send_command_async(cmd, speed);

      // === LCD update hanya saat direction berubah (hemat) ===
      if (dir != s_last_dir) {
        ESP_LOGI(TAG, "Joystick: %s -> %s", dir_names[s_last_dir],
                 dir_names[dir]);

        if (dir != JOY_DIR_CENTER) {
          lcd_show_joystick(dir_names[dir]);
        } else {
          lcd_show_message("REMOTE MODE", "USE JOYSTICK");
        }
        s_last_dir = dir;
      }

    } else {
      // Reset to center when not in Remote mode
      if (s_last_dir != JOY_DIR_CENTER) {
        // Kirim STOP terakhir saat keluar Remote mode
        espnow_send_command_async(ESPNOW_CMD_STOP, 0);
        s_last_dir = JOY_DIR_CENTER;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Kirim setiap 50ms = 20Hz
  }
}
