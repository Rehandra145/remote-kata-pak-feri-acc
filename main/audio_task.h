/**
 * @file audio_task.h
 * @brief Audio processing task interface for Voice-Controlled RC System
 */

#ifndef AUDIO_TASK_H
#define AUDIO_TASK_H

#include "esp_err.h"
#include "types.h"


/**
 * @brief Initialize audio subsystem (I2S, AFE, models)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_init(void);

/**
 * @brief Audio processing FreeRTOS task entry point
 * @param param Task parameter (unused)
 */
void audio_processing_task(void *param);

/**
 * @brief Check if audio system is ready
 * @return true if initialization complete and running
 */
bool audio_is_ready(void);

/**
 * @brief Map MultiNet phrase ID to event type
 * @param phrase_id Raw phrase ID from MultiNet
 * @return Corresponding event type
 */
rc_event_type_t audio_phrase_id_to_event(int phrase_id);

/**
 * @brief Validate recognition result against thresholds
 * @param result MultiNet result structure
 * @return Validated recognition result
 */
recognition_result_t audio_validate_recognition(void *result);

#endif // AUDIO_TASK_H
