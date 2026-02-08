/**
 * @file event_queue.h
 * @brief Event queue management for Voice-Controlled RC System
 */

#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "types.h"


/**
 * @brief Initialize the event queue
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t event_queue_init(void);

/**
 * @brief Post an event to the queue (non-blocking)
 * @param type Event type
 * @return true if event was posted, false if queue full
 */
bool event_post(rc_event_type_t type);

/**
 * @brief Post a voice event with confidence info
 * @param type Event type
 * @param confidence Recognition confidence (0.0-1.0)
 * @param phrase_id Raw MultiNet phrase ID
 * @return true if event was posted, false if queue full
 */
bool event_post_voice(rc_event_type_t type, float confidence, int phrase_id);

/**
 * @brief Receive an event from the queue (blocking with timeout)
 * @param event Output event structure
 * @param timeout_ms Maximum wait time in milliseconds
 * @return true if event received, false on timeout
 */
bool event_receive(rc_event_msg_t *event, uint32_t timeout_ms);

/**
 * @brief Get number of events waiting in queue
 * @return Number of pending events
 */
uint32_t event_queue_pending(void);

/**
 * @brief Clear all pending events
 */
void event_queue_clear(void);

#endif // EVENT_QUEUE_H
