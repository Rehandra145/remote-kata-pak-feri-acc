/**
 * @file espnow_comm.c
 * @brief ESP-NOW Communication Implementation
 *
 * Transmitter side: Sends voice commands to receiver via ESP-NOW
 */

#include "espnow_comm.h"
#include "config.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "ESPNOW";

/*===========================================================================
 * CONFIGURATION
 *===========================================================================*/

// Receiver MAC address
static uint8_t s_receiver_mac[ESP_NOW_ETH_ALEN] = {0xFC, 0x01, 0x2C, 0xD1,
                                                   0x76, 0x54}; // MAC address

// Packet header marker
#define PACKET_HEADER 0xAA

// Default motor speed (0-255)
#define DEFAULT_SPEED 200

/*===========================================================================
 * STATE VARIABLES
 *===========================================================================*/

static bool s_espnow_initialized = false;
static bool s_last_tx_success = false;
static SemaphoreHandle_t s_tx_semaphore = NULL;

/*===========================================================================
 * HELPER FUNCTIONS
 *===========================================================================*/

/**
 * @brief Calculate XOR checksum of packet
 */
static uint8_t calculate_checksum(const espnow_packet_t *packet) {
  const uint8_t *data = (const uint8_t *)packet;
  uint8_t checksum = 0;
  // XOR all bytes except the checksum field itself
  for (int i = 0; i < sizeof(espnow_packet_t) - 1; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

/**
 * @brief Convert event type to ESP-NOW command
 */
static espnow_cmd_t event_to_command(rc_event_type_t event) {
  switch (event) {
  case EVT_VOICE_FORWARD:
    return ESPNOW_CMD_FORWARD;
  case EVT_VOICE_BACKWARD:
    return ESPNOW_CMD_BACKWARD;
  case EVT_VOICE_LEFT:
    return ESPNOW_CMD_LEFT;
  case EVT_VOICE_RIGHT:
    return ESPNOW_CMD_RIGHT;
  case EVT_VOICE_STOP:
    return ESPNOW_CMD_STOP;
  default:
    return ESPNOW_CMD_STOP;
  }
}

/*===========================================================================
 * CALLBACKS
 *===========================================================================*/

/**
 * @brief ESP-NOW send callback
 */
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    s_last_tx_success = true;
    ESP_LOGD(TAG, "TX Success");
  } else {
    s_last_tx_success = false;
    ESP_LOGW(TAG, "TX Failed");
  }

  // Release semaphore to unblock sender
  if (s_tx_semaphore != NULL) {
    xSemaphoreGive(s_tx_semaphore);
  }
}

/*===========================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

esp_err_t espnow_comm_init(void) {
  ESP_LOGI(TAG, "Initializing ESP-NOW communication...");

  // Create TX semaphore
  s_tx_semaphore = xSemaphoreCreateBinary();
  if (s_tx_semaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create TX semaphore");
    return ESP_ERR_NO_MEM;
  }

  // NVS already initialized by main.c - skip here

  // Initialize WiFi in station mode (required for ESP-NOW)
  esp_err_t ret;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set WiFi channel (must match receiver)
  ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

  // Initialize ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());

  // Register send callback
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

  // Add receiver as peer
  esp_now_peer_info_t peer_info = {0};
  memcpy(peer_info.peer_addr, s_receiver_mac, ESP_NOW_ETH_ALEN);
  peer_info.channel = 1;
  peer_info.encrypt = false;

  ret = esp_now_add_peer(&peer_info);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
    return ret;
  }

  s_espnow_initialized = true;

  ESP_LOGI(TAG, "ESP-NOW initialized");
  ESP_LOGI(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           s_receiver_mac[0], s_receiver_mac[1], s_receiver_mac[2],
           s_receiver_mac[3], s_receiver_mac[4], s_receiver_mac[5]);

  return ESP_OK;
}

esp_err_t espnow_send_command(espnow_cmd_t cmd, uint8_t speed) {
  if (!s_espnow_initialized) {
    ESP_LOGE(TAG, "ESP-NOW not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  // Build packet
  espnow_packet_t packet = {.header = PACKET_HEADER,
                            .msg_type = MSG_TYPE_COMMAND,
                            .command = cmd,
                            .speed = speed,
                            .duration_ms = 0, // Continuous
                            .checksum = 0};
  packet.checksum = calculate_checksum(&packet);

  // Log command
  const char *cmd_names[] = {"STOP",  "FORWARD",  "BACKWARD", "LEFT",
                             "RIGHT", "STRAFE_L", "STRAFE_R"};
  const char *cmd_name = (cmd <= 6) ? cmd_names[cmd] : "UNKNOWN";
  ESP_LOGI(TAG, "Sending: %s (speed=%d)", cmd_name, speed);

  // Send packet
  esp_err_t ret =
      esp_now_send(s_receiver_mac, (uint8_t *)&packet, sizeof(packet));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Wait for send callback (with timeout)
  if (xSemaphoreTake(s_tx_semaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "TX callback timeout");
    return ESP_ERR_TIMEOUT;
  }

  return s_last_tx_success ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Send command TANPA blocking (fire-and-forget)
 * Khusus untuk joystick continuous mode agar tidak trigger watchdog reset.
 * Tidak menunggu callback, langsung return setelah esp_now_send.
 */
esp_err_t espnow_send_command_async(espnow_cmd_t cmd, uint8_t speed) {
  if (!s_espnow_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Build packet
  espnow_packet_t packet = {.header = PACKET_HEADER,
                            .msg_type = MSG_TYPE_COMMAND,
                            .command = cmd,
                            .speed = speed,
                            .duration_ms = 0,
                            .checksum = 0};
  packet.checksum = calculate_checksum(&packet);

  // Fire-and-forget: TIDAK nunggu semaphore
  return esp_now_send(s_receiver_mac, (uint8_t *)&packet, sizeof(packet));
}

esp_err_t espnow_send_event(rc_event_type_t event_type) {
  espnow_cmd_t cmd = event_to_command(event_type);
  return espnow_send_command(cmd, DEFAULT_SPEED);
}

esp_err_t espnow_send_emergency_stop(void) {
  if (!s_espnow_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Build emergency packet
  espnow_packet_t packet = {.header = PACKET_HEADER,
                            .msg_type = MSG_TYPE_EMERGENCY,
                            .command = ESPNOW_CMD_EMERGENCY,
                            .speed = 0,
                            .duration_ms = 0,
                            .checksum = 0};
  packet.checksum = calculate_checksum(&packet);

  ESP_LOGE(TAG, ">>> SENDING EMERGENCY STOP <<<");

  // Send multiple times for reliability
  esp_err_t ret = ESP_OK;
  for (int i = 0; i < 3; i++) {
    ret = esp_now_send(s_receiver_mac, (uint8_t *)&packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  return ret;
}

bool espnow_is_ready(void) { return s_espnow_initialized; }

bool espnow_last_tx_success(void) { return s_last_tx_success; }
