/**
 * @file event_queue.c
 * @brief Event queue implementation for Voice-Controlled RC System
 */

#include "event_queue.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "EVT_QUEUE";

// Static queue handle
static QueueHandle_t s_event_queue = NULL;

esp_err_t event_queue_init(void) {
  if (s_event_queue != NULL) {
    ESP_LOGW(TAG, "Event queue already initialized");
    return ESP_OK;
  }

  s_event_queue = xQueueCreate(EVT_QUEUE_LENGTH, sizeof(rc_event_msg_t));

  if (s_event_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create event queue");
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "Event queue initialized (length=%d)", EVT_QUEUE_LENGTH);
  return ESP_OK;
}

bool event_post(rc_event_type_t type) {
  return event_post_voice(type, 0.0f, -1);
}

bool event_post_voice(rc_event_type_t type, float confidence, int phrase_id) {
  if (s_event_queue == NULL) {
    ESP_LOGE(TAG, "Event queue not initialized");
    return false;
  }

  rc_event_msg_t event = {
      .type = type,
      .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
      .confidence = confidence,
      .raw_phrase_id = phrase_id,
      .priority = 0 // Default priority
  };

  // Non-blocking send
  BaseType_t result =
      xQueueSend(s_event_queue, &event, pdMS_TO_TICKS(EVT_QUEUE_TIMEOUT_MS));

  if (result != pdTRUE) {
    ESP_LOGW(TAG, "Event queue full, dropping event: %s", event_name(type));
    return false;
  }

#if ENABLE_DEBUG_LOGGING
  if (type >= EVT_VOICE_FORWARD && type <= EVT_VOICE_STOP) {
    ESP_LOGI(TAG, "Posted voice event: %s (conf=%.2f, id=%d)", event_name(type),
             confidence, phrase_id);
  } else {
    ESP_LOGD(TAG, "Posted event: %s", event_name(type));
  }
#endif

  return true;
}

bool event_receive(rc_event_msg_t *event, uint32_t timeout_ms) {
  if (s_event_queue == NULL || event == NULL) {
    return false;
  }

  BaseType_t result =
      xQueueReceive(s_event_queue, event, pdMS_TO_TICKS(timeout_ms));

  return (result == pdTRUE);
}

uint32_t event_queue_pending(void) {
  if (s_event_queue == NULL) {
    return 0;
  }
  return uxQueueMessagesWaiting(s_event_queue);
}

void event_queue_clear(void) {
  if (s_event_queue != NULL) {
    xQueueReset(s_event_queue);
    ESP_LOGW(TAG, "Event queue cleared");
  }
}
