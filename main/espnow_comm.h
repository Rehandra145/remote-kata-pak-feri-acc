/**
 * @file espnow_comm.h
 * @brief ESP-NOW Communication for Voice-Controlled RC System
 *
 * Handles wireless communication between Transmitter (voice node)
 * and Receiver (actuator node) using ESP-NOW protocol.
 */

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include "esp_err.h"
#include "types.h"
#include <stdbool.h>
#include <stdint.h>

/*===========================================================================
 * PACKET DEFINITIONS
 *===========================================================================*/

// Message types
typedef enum {
  MSG_TYPE_COMMAND = 0x01,   // Motor command
  MSG_TYPE_HEARTBEAT = 0x02, // Keep-alive
  MSG_TYPE_ACK = 0x03,       // Acknowledgment
  MSG_TYPE_EMERGENCY = 0xFF  // Emergency stop
} espnow_msg_type_t;

// Command types (matches rc_event_type_t)
typedef enum {
  ESPNOW_CMD_STOP = 0,
  ESPNOW_CMD_FORWARD = 1,
  ESPNOW_CMD_BACKWARD = 2,
  ESPNOW_CMD_LEFT = 3,
  ESPNOW_CMD_RIGHT = 4,
  ESPNOW_CMD_STRAFE_LEFT = 5,  // Mecanum strafe
  ESPNOW_CMD_STRAFE_RIGHT = 6, // Mecanum strafe
  ESPNOW_CMD_EMERGENCY = 0xFF
} espnow_cmd_t;

// Command packet structure (7 bytes)
typedef struct __attribute__((packed)) {
  uint8_t header;       // 0xAA - start marker
  uint8_t msg_type;     // Message type
  uint8_t command;      // Command ID
  uint8_t speed;        // Speed 0-255
  uint16_t duration_ms; // Duration (0 = continuous)
  uint8_t checksum;     // XOR checksum
} espnow_packet_t;

/*===========================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

/**
 * @brief Initialize ESP-NOW communication
 * @return ESP_OK on success
 */
esp_err_t espnow_comm_init(void);

/**
 * @brief Send command to receiver (blocking, waits for callback)
 * Cocok untuk Voice mode (diskrit, jarang kirim)
 * @param cmd Command to send
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t espnow_send_command(espnow_cmd_t cmd, uint8_t speed);

/**
 * @brief Send command to receiver (non-blocking, fire-and-forget)
 * Cocok untuk Joystick mode (continuous, 20Hz)
 * @param cmd Command to send
 * @param speed Motor speed (0-255)
 * @return ESP_OK if queued successfully
 */
esp_err_t espnow_send_command_async(espnow_cmd_t cmd, uint8_t speed);

/**
 * @brief Send command from event type
 * @param event_type RC event type
 * @return ESP_OK on success
 */
esp_err_t espnow_send_event(rc_event_type_t event_type);

/**
 * @brief Send emergency stop
 * @return ESP_OK on success
 */
esp_err_t espnow_send_emergency_stop(void);

/**
 * @brief Check if ESP-NOW is initialized and ready
 * @return true if ready
 */
bool espnow_is_ready(void);

/**
 * @brief Get last transmission status
 * @return true if last TX was successful
 */
bool espnow_last_tx_success(void);

#endif // ESPNOW_COMM_H
