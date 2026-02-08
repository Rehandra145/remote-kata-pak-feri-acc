/**
 * @file audio_task.c
 * @brief Audio processing task for Voice-Controlled RC System
 *
 * Handles I2S microphone input, ESP-SR AFE processing, WakeNet wake word
 * detection, and MultiNet command recognition.
 *
 * ARCHITECTURE: Uses TWO separate tasks as required by ESP-SR:
 * 1. Feed Task: Continuously reads audio from I2S and feeds to AFE
 * 2. Detect Task: Fetches processed audio and runs WakeNet/MultiNet
 *
 * IMPORTANT: This module converts voice recognition results into EVENTS.
 * It NEVER directly controls motors.
 */

#include "audio_task.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "event_queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"
#include "mode_manager.h"

// ESP-SR includes
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"

static const char *TAG = "AUDIO";

// Audio ready flag
static bool s_audio_ready = false;

// I2S Port
#define I2S_PORT I2S_NUM_0

// INMP441 requires 32-bit I2S frame (24-bit data)
#undef AUDIO_BITS_PER_SAMPLE
#define AUDIO_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT

// ESP-SR handles (shared between tasks)
static esp_afe_sr_iface_t *s_afe_handle = NULL;
static esp_afe_sr_data_t *s_afe_data = NULL;
static esp_mn_iface_t *s_multinet = NULL;
static model_iface_data_t *s_mn_model_data = NULL;
static srmodel_list_t *s_models = NULL;

// Task handles
static TaskHandle_t s_feed_task_handle = NULL;
static TaskHandle_t s_detect_task_handle = NULL;

/**
 * @brief Initialize I2S for microphone input (Legacy API)
 */
static esp_err_t init_i2s(void) {
  ESP_LOGI(TAG, "Initializing I2S microphone (Legacy API) 32-bit...");

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = AUDIO_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8, // Increased for better buffering
      .dma_buf_len = 256, // Smaller chunks for faster response
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  i2s_pin_config_t pin_config = {.bck_io_num = GPIO_I2S_BCLK,
                                 .ws_io_num = GPIO_I2S_WS,
                                 .data_out_num = -1,
                                 .data_in_num = GPIO_I2S_DIN};

  esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2S driver");
    return ret;
  }

  ret = i2s_set_pin(I2S_PORT, &pin_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set I2S pins");
    return ret;
  }

  ret = i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT,
                    I2S_CHANNEL_MONO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set I2S clock");
    return ret;
  }

  ret = i2s_start(I2S_PORT);
  if (ret != ESP_OK)
    return ret;

  ESP_LOGI(TAG, "I2S initialized (Legacy): 16kHz, 32-bit");
  return ESP_OK;
}

/**
 * @brief Initialize ESP-SR models and AFE
 */
static esp_err_t init_esp_sr(void) {
  ESP_LOGI(TAG, "Initializing ESP-SR...");

  // Load models from partition
  s_models = esp_srmodel_init("model");
  if (s_models == NULL) {
    ESP_LOGE(TAG, "Failed to load SR models");
    return ESP_FAIL;
  }

  // Get model names
  char *wn_name = esp_srmodel_filter(s_models, ESP_WN_PREFIX, NULL);
  char *mn_name = esp_srmodel_filter(s_models, ESP_MN_PREFIX, NULL);

  if (wn_name == NULL) {
    ESP_LOGE(TAG, "No WakeNet model found");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "WakeNet model: %s", wn_name);

  if (mn_name == NULL) {
    ESP_LOGE(TAG, "No MultiNet model found");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "MultiNet model: %s", mn_name);

  // Configure AFE - CORRECT SETTINGS FOR WAKE WORD DETECTION
  afe_config_t afe_config = AFE_CONFIG_DEFAULT();
  afe_config.wakenet_model_name = wn_name;
  afe_config.aec_init = false; // No echo cancellation (single mic, no speaker)
  afe_config.se_init = true;   // Speech enhancement ON
  afe_config.vad_init = true;  // Voice activity detection ON
  afe_config.wakenet_init = true;
  afe_config.voice_communication_init = false;
  afe_config.pcm_config.total_ch_num = 1;
  afe_config.pcm_config.mic_num = 1;
  afe_config.pcm_config.ref_num = 0;
  afe_config.pcm_config.sample_rate = AUDIO_SAMPLE_RATE;

  // IMPORTANT: Set memory allocation type for proper performance
  afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
  afe_config.afe_ringbuf_size = 50; // Increase ring buffer for better buffering

  ESP_LOGI(TAG, "AFE Config: SE=On, VAD=On, WakeNet=On. Model: %s", wn_name);

  // Initialize AFE
  s_afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
  s_afe_data = s_afe_handle->create_from_config(&afe_config);
  if (s_afe_data == NULL) {
    ESP_LOGE(TAG, "Failed to create AFE");
    return ESP_FAIL;
  }

  // Initialize MultiNet
  s_multinet = esp_mn_handle_from_name(mn_name);
  if (s_multinet == NULL) {
    ESP_LOGE(TAG, "Failed to get MultiNet handle");
    return ESP_FAIL;
  }

  s_mn_model_data = s_multinet->create(mn_name, TIMEOUT_WAKEWORD_LISTEN_MS);
  if (s_mn_model_data == NULL) {
    ESP_LOGE(TAG, "Failed to create MultiNet model data");
    return ESP_FAIL;
  }

  // Configure custom commands
  ESP_LOGI(TAG, "Configuring voice commands...");
  esp_mn_commands_alloc(s_multinet, s_mn_model_data);

  // Simple direction words for MultiNet7 English
  esp_mn_commands_add(CMD_ID_FORWARD, "forward");
  esp_mn_commands_add(CMD_ID_BACKWARD, "backward");
  esp_mn_commands_add(CMD_ID_LEFT, "left");
  esp_mn_commands_add(CMD_ID_RIGHT, "right");
  esp_mn_commands_add(CMD_ID_STOP, "stop");

  esp_mn_error_t *err = esp_mn_commands_update();
  if (err != NULL) {
    ESP_LOGW(TAG,
             "Some commands may not be recognized - check command phrases");
  }

  // Print registered commands
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "=== REGISTERED COMMANDS ===");
  ESP_LOGI(TAG, "  \"FORWARD\"");
  ESP_LOGI(TAG, "  \"BACKWARD\"");
  ESP_LOGI(TAG, "  \"LEFT\"");
  ESP_LOGI(TAG, "  \"RIGHT\"");
  ESP_LOGI(TAG, "  \"STOP\"");
  ESP_LOGI(TAG, "===========================");
  ESP_LOGI(TAG, "");

  ESP_LOGI(TAG, "ESP-SR initialized with 5 commands");
  return ESP_OK;
}

esp_err_t audio_init(void) {
  esp_err_t ret;

  ret = init_i2s();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = init_esp_sr();
  if (ret != ESP_OK) {
    return ret;
  }

  s_audio_ready = true;
  ESP_LOGI(TAG, "Audio subsystem initialized successfully");
  return ESP_OK;
}

bool audio_is_ready(void) { return s_audio_ready; }

rc_event_type_t audio_phrase_id_to_event(int phrase_id) {
  switch (phrase_id) {
  case CMD_ID_FORWARD:
    return EVT_VOICE_FORWARD;
  case CMD_ID_BACKWARD:
    return EVT_VOICE_BACKWARD;
  case CMD_ID_LEFT:
    return EVT_VOICE_LEFT;
  case CMD_ID_RIGHT:
    return EVT_VOICE_RIGHT;
  case CMD_ID_STOP:
    return EVT_VOICE_STOP;
  default:
    ESP_LOGW(TAG, "Unknown phrase ID: %d", phrase_id);
    return EVT_NONE;
  }
}

recognition_result_t audio_validate_recognition(void *result_ptr) {
  recognition_result_t output = {.is_valid = false,
                                 .event = EVT_NONE,
                                 .confidence = 0.0f,
                                 .phrase_id = -1};

  esp_mn_results_t *result = (esp_mn_results_t *)result_ptr;

  if (result->num == 0) {
    ESP_LOGD(TAG, "No recognition result");
    return output;
  }

  if (result->prob[0] < CONFIDENCE_THRESHOLD_MIN) {
    ESP_LOGW(TAG, "Rejected: conf=%.2f < threshold=%.2f", result->prob[0],
             CONFIDENCE_THRESHOLD_MIN);
    return output;
  }

  if (result->num > 1) {
    float gap = result->prob[0] - result->prob[1];
    if (gap < CONFIDENCE_GAP_MIN) {
      ESP_LOGW(TAG, "Rejected: ambiguous (top=%.2f, second=%.2f, gap=%.2f)",
               result->prob[0], result->prob[1], gap);
      return output;
    }
  }

  rc_event_type_t evt = audio_phrase_id_to_event(result->phrase_id[0]);
  if (evt == EVT_NONE) {
    ESP_LOGW(TAG, "Rejected: unknown phrase ID %d", result->phrase_id[0]);
    return output;
  }

  output.is_valid = true;
  output.event = evt;
  output.confidence = result->prob[0];
  output.phrase_id = result->phrase_id[0];

  ESP_LOGI(TAG, "Valid recognition: %s (conf=%.2f)", event_name(evt),
           output.confidence);
  return output;
}

/**
 * @brief FEED TASK: Continuously reads audio from I2S and feeds to AFE
 * This task runs on Core 1 and must never block for long periods
 */
static void audio_feed_task(void *param) {
  ESP_LOGI(TAG, "Feed task started on core %d", xPortGetCoreID());

  // Wait for initialization
  while (!s_audio_ready) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  int afe_chunksize = s_afe_handle->get_feed_chunksize(s_afe_data);
  ESP_LOGI(TAG, "Feed task: chunk size = %d samples (%.1fms)", afe_chunksize,
           afe_chunksize / 16.0f);

  // Allocate audio buffer
  int16_t *audio_buffer = heap_caps_malloc(
      afe_chunksize * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (audio_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate audio buffer for feed task");
    vTaskDelete(NULL);
    return;
  }

  // Temporary buffer for 32-bit I2S samples
  int32_t *i2s_read_buff = heap_caps_malloc(
      afe_chunksize * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (i2s_read_buff == NULL) {
    ESP_LOGE(TAG, "Failed to allocate I2S buffer");
    free(audio_buffer);
    vTaskDelete(NULL);
    return;
  }

  int feed_count = 0;

  while (1) {
    size_t bytes_read = 0;

    // Read 32-bit samples from I2S
    esp_err_t ret =
        i2s_read(I2S_PORT, i2s_read_buff, afe_chunksize * sizeof(int32_t),
                 &bytes_read, pdMS_TO_TICKS(100));

    if (ret != ESP_OK || bytes_read == 0) {
      ESP_LOGW(TAG, "I2S read failed or empty");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    int samples_read = bytes_read / sizeof(int32_t);

    // Convert 32-bit to 16-bit (INMP441: 24-bit data in 32-bit frame)
    for (int i = 0; i < samples_read; i++) {
      int32_t val32 = i2s_read_buff[i];
      int32_t val16 = val32 >> 16;

      if (val16 > 32767)
        val16 = 32767;
      if (val16 < -32768)
        val16 = -32768;

      audio_buffer[i] = (int16_t)val16;
    }

    // Feed to AFE (non-blocking)
    s_afe_handle->feed(s_afe_data, audio_buffer);

    feed_count++;

    // Debug logging (every 100 feeds, approximately every 3.2 seconds)
    if (feed_count % 100 == 0) {
      // Calculate max amplitude
      int16_t max_val = 0;
      for (int i = 0; i < samples_read; i++) {
        int16_t abs_val =
            audio_buffer[i] > 0 ? audio_buffer[i] : -audio_buffer[i];
        if (abs_val > max_val)
          max_val = abs_val;
      }
      ESP_LOGI(TAG, "Feed[%d]: samples=%d, max_amp=%d", feed_count,
               samples_read, max_val);
    }

    // Small yield to prevent starving other tasks
    taskYIELD();
  }

  // Cleanup (never reached)
  free(audio_buffer);
  free(i2s_read_buff);
  vTaskDelete(NULL);
}

/**
 * @brief DETECT TASK: Fetches processed audio from AFE and runs recognition
 * This task runs on Core 1 alongside feed task
 */
static void audio_detect_task(void *param) {
  ESP_LOGI(TAG, "Detect task started on core %d", xPortGetCoreID());

  // Wait for initialization
  while (!s_audio_ready) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Give feed task time to start
  vTaskDelay(pdMS_TO_TICKS(500));

  int afe_fetch_chunksize = s_afe_handle->get_fetch_chunksize(s_afe_data);
  ESP_LOGI(TAG, "Detect task: fetch chunk size = %d samples",
           afe_fetch_chunksize);

  bool listening_for_command = false;
  int detect_count = 0;

  while (1) {
    // Fetch processed audio from AFE (this will block until data is ready)
    afe_fetch_result_t *fetch_result = s_afe_handle->fetch(s_afe_data);

    detect_count++;

    if (fetch_result == NULL) {
      ESP_LOGW(TAG, "Fetch returned NULL");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Debug logging for first 30 iterations
    if (detect_count <= 30 || detect_count % 200 == 0) {
      int16_t max_out = 0;
      if (fetch_result->data != NULL) {
        for (int i = 0; i < afe_fetch_chunksize; i++) {
          int16_t abs_val = fetch_result->data[i] > 0 ? fetch_result->data[i]
                                                      : -fetch_result->data[i];
          if (abs_val > max_out)
            max_out = abs_val;
        }
      }
      ESP_LOGI(TAG, "Detect[%d]: Out=%d, Wake=%d, VAD=%d, ret=%d", detect_count,
               max_out, fetch_result->wakeup_state, fetch_result->vad_state,
               fetch_result->ret_value);
    }

    // Check for wake word detection (only in Voice mode)
    if (fetch_result->wakeup_state == WAKENET_DETECTED &&
        is_voice_input_allowed()) {
      ESP_LOGI(TAG, "");
      ESP_LOGI(TAG, "========================================");
      ESP_LOGI(TAG, ">>> WAKE WORD DETECTED! <<<");
      ESP_LOGI(TAG, "========================================");
      ESP_LOGI(TAG, "Say one of these commands:");
      ESP_LOGI(TAG, "  - \"FORWARD\"");
      ESP_LOGI(TAG, "  - \"BACKWARD\"");
      ESP_LOGI(TAG, "  - \"LEFT\"");
      ESP_LOGI(TAG, "  - \"RIGHT\"");
      ESP_LOGI(TAG, "  - \"STOP\"");
      ESP_LOGI(TAG, "Listening for 10 seconds...");
      ESP_LOGI(TAG, "========================================");
      ESP_LOGI(TAG, "");

      // *** IMMEDIATE LCD FEEDBACK ***
      // Show listening state DIRECTLY on LCD before event processing
      lcd_show_message("LISTENING...", "SAY COMMAND!");

      // Disable WakeNet to focus on command detection
      s_afe_handle->disable_wakenet(s_afe_data);
      ESP_LOGI(TAG, "WakeNet disabled for command listening");

      listening_for_command = true;
      event_post(EVT_WAKEWORD_DETECTED);

      // Reset Multinet state
      s_multinet->clean(s_mn_model_data);
    }

    // If listening mode, run MultiNet
    if (listening_for_command && fetch_result->data != NULL) {
      esp_mn_state_t mn_state =
          s_multinet->detect(s_mn_model_data, fetch_result->data);

      switch (mn_state) {
      case ESP_MN_STATE_DETECTING:
        // Log every 50 iterations while detecting
        if (detect_count % 50 == 0) {
          ESP_LOGI(TAG, "[LISTENING] MultiNet processing... (%d)",
                   detect_count);
        }
        break;

      case ESP_MN_STATE_DETECTED: {
        esp_mn_results_t *mn_result = s_multinet->get_results(s_mn_model_data);

        if (mn_result->num > 0) {
          int phrase_id = mn_result->phrase_id[0];
          float prob = mn_result->prob[0];

          // Get command name
          const char *cmd_name = "UNKNOWN";
          switch (phrase_id) {
          case CMD_ID_FORWARD:
            cmd_name = "FORWARD";
            break;
          case CMD_ID_BACKWARD:
            cmd_name = "BACKWARD";
            break;
          case CMD_ID_LEFT:
            cmd_name = "LEFT";
            break;
          case CMD_ID_RIGHT:
            cmd_name = "RIGHT";
            break;
          case CMD_ID_STOP:
            cmd_name = "STOP";
            break;
          }

          ESP_LOGI(TAG, "");
          ESP_LOGI(TAG, "****************************************");
          ESP_LOGI(TAG, "*** COMMAND DETECTED: %s ***", cmd_name);
          ESP_LOGI(TAG, "*** ID=%d, Confidence=%.0f%% ***", phrase_id,
                   prob * 100);
          ESP_LOGI(TAG, "****************************************");
          ESP_LOGI(TAG, "");

          rc_event_type_t evt_type = audio_phrase_id_to_event(phrase_id);
          event_post(evt_type);

          // Re-enable WakeNet after command was processed
          listening_for_command = false;
          s_afe_handle->enable_wakenet(s_afe_data);
          ESP_LOGI(TAG, "WakeNet re-enabled after command");
        }
        break;
      }

      case ESP_MN_STATE_TIMEOUT:
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG,
                 "--- Command timeout - listening for wake word again ---");
        ESP_LOGI(TAG, "");
        listening_for_command = false;
        // Re-enable WakeNet for wake word detection
        s_afe_handle->enable_wakenet(s_afe_data);
        ESP_LOGI(TAG, "WakeNet re-enabled");
        break;

      default:
        break;
      }
    }
  }

  vTaskDelete(NULL);
}

/**
 * @brief Main audio processing task entry point - spawns feed and detect tasks
 */
void audio_processing_task(void *param) {
  ESP_LOGI(TAG,
           "Audio processing task started - spawning feed and detect tasks");

  // Wait for initialization
  while (!s_audio_ready) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Log chunk sizes for debugging
  int afe_feed_size = s_afe_handle->get_feed_chunksize(s_afe_data);
  int afe_fetch_size = s_afe_handle->get_fetch_chunksize(s_afe_data);
  int mn_size = s_multinet->get_samp_chunksize(s_mn_model_data);

  ESP_LOGW(TAG, "=== CHUNK SIZES ===");
  ESP_LOGW(TAG, "AFE feed: %d samples (%.1fms)", afe_feed_size,
           afe_feed_size / 16.0f);
  ESP_LOGW(TAG, "AFE fetch: %d samples (%.1fms)", afe_fetch_size,
           afe_fetch_size / 16.0f);
  ESP_LOGW(TAG, "MultiNet: %d samples", mn_size);

  // Create feed task on Core 1 (needs more stack for I2S buffers)
  xTaskCreatePinnedToCore(audio_feed_task, "audio_feed", 8192, NULL,
                          PRIORITY_AUDIO_PROCESSING, &s_feed_task_handle,
                          CORE_AUDIO_PROCESSING);

  // Create detect task on Core 1
  xTaskCreatePinnedToCore(audio_detect_task, "audio_detect", 8192, NULL,
                          PRIORITY_AUDIO_PROCESSING -
                              1, // Slightly lower priority than feed
                          &s_detect_task_handle, CORE_AUDIO_PROCESSING);

  ESP_LOGI(TAG, "Audio tasks created - feed and detect running");

  // This task can now exit or monitor
  vTaskDelete(NULL);
}
