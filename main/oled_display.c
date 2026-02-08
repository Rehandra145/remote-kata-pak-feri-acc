/**
 * @file oled_display.c
 * @brief OLED Display implementation using SSD1306 0.96" I2C
 */

#include "oled_display.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "OLED";

// SSD1306 I2C address
#define SSD1306_ADDR 0x3C

// SSD1306 commands
#define SSD1306_CMD_DISPLAY_OFF 0xAE
#define SSD1306_CMD_DISPLAY_ON 0xAF
#define SSD1306_CMD_SET_MEM_ADDR_MODE 0x20
#define SSD1306_CMD_SET_COL_ADDR 0x21
#define SSD1306_CMD_SET_PAGE_ADDR 0x22
#define SSD1306_CMD_SET_DISP_START_LINE 0x40
#define SSD1306_CMD_SET_CONTRAST 0x81
#define SSD1306_CMD_SET_SEGMENT_REMAP 0xA1
#define SSD1306_CMD_SET_NORMAL_DISP 0xA6
#define SSD1306_CMD_SET_MUX_RATIO 0xA8
#define SSD1306_CMD_SET_COM_OUT_DIR 0xC8
#define SSD1306_CMD_SET_DISP_OFFSET 0xD3
#define SSD1306_CMD_SET_DISP_CLK_DIV 0xD5
#define SSD1306_CMD_SET_PRECHARGE 0xD9
#define SSD1306_CMD_SET_COM_PIN_CFG 0xDA
#define SSD1306_CMD_SET_VCOMH_DESEL 0xDB
#define SSD1306_CMD_CHARGE_PUMP 0x8D

// Display dimensions
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)

// Frame buffer
static uint8_t s_frame_buffer[OLED_WIDTH * OLED_PAGES];

// I2C port
#define I2C_MASTER_NUM I2C_NUM_0

// Simple 5x7 font (ASCII 32-90, uppercase + numbers + symbols)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z (90)
};

// Display message queue
typedef struct {
  char line1[22];
  char line2[22];
  char line3[22];
  rc_state_t state;
  bool update_state;
} oled_msg_t;

static QueueHandle_t s_oled_queue = NULL;
static bool s_oled_initialized = false;

/**
 * @brief Send command to SSD1306
 */
static esp_err_t ssd1306_send_cmd(uint8_t cmd) {
  uint8_t data[2] = {0x00, cmd}; // Co = 0, D/C# = 0 (command)
  return i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, data, 2,
                                    pdMS_TO_TICKS(100));
}

/**
 * @brief Send data to SSD1306
 */
static esp_err_t ssd1306_send_data(uint8_t *data, size_t len) {
  uint8_t *buf = malloc(len + 1);
  if (buf == NULL)
    return ESP_ERR_NO_MEM;

  buf[0] = 0x40; // Co = 0, D/C# = 1 (data)
  memcpy(buf + 1, data, len);

  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, buf,
                                             len + 1, pdMS_TO_TICKS(100));
  free(buf);
  return ret;
}

/**
 * @brief Initialize I2C for OLED
 */
static esp_err_t init_i2c_oled(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = GPIO_OLED_SDA,
      .scl_io_num = GPIO_OLED_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 400000, // 400kHz
  };

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (ret != ESP_OK)
    return ret;

  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief Initialize SSD1306 display
 */
static esp_err_t ssd1306_init(void) {
  // Initialization sequence for SSD1306 128x64
  uint8_t init_cmds[] = {
      SSD1306_CMD_DISPLAY_OFF,
      SSD1306_CMD_SET_MEM_ADDR_MODE,
      0x00, // Horizontal addressing
      SSD1306_CMD_SET_DISP_START_LINE | 0x00,
      SSD1306_CMD_SET_SEGMENT_REMAP,
      SSD1306_CMD_SET_MUX_RATIO,
      0x3F, // 64 rows
      SSD1306_CMD_SET_COM_OUT_DIR,
      SSD1306_CMD_SET_DISP_OFFSET,
      0x00,
      SSD1306_CMD_SET_COM_PIN_CFG,
      0x12,
      SSD1306_CMD_SET_DISP_CLK_DIV,
      0x80,
      SSD1306_CMD_SET_PRECHARGE,
      0xF1,
      SSD1306_CMD_SET_VCOMH_DESEL,
      0x30,
      SSD1306_CMD_SET_CONTRAST,
      0xFF,
      SSD1306_CMD_CHARGE_PUMP,
      0x14, // Enable charge pump
      SSD1306_CMD_SET_NORMAL_DISP,
      SSD1306_CMD_DISPLAY_ON,
  };

  for (size_t i = 0; i < sizeof(init_cmds); i++) {
    esp_err_t ret = ssd1306_send_cmd(init_cmds[i]);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to send init cmd %d: %s", (int)i,
               esp_err_to_name(ret));
      return ret;
    }
  }

  return ESP_OK;
}

/**
 * @brief Update display with frame buffer
 */
static void ssd1306_update(void) {
  // Set column address 0-127
  ssd1306_send_cmd(SSD1306_CMD_SET_COL_ADDR);
  ssd1306_send_cmd(0);
  ssd1306_send_cmd(OLED_WIDTH - 1);

  // Set page address 0-7
  ssd1306_send_cmd(SSD1306_CMD_SET_PAGE_ADDR);
  ssd1306_send_cmd(0);
  ssd1306_send_cmd(OLED_PAGES - 1);

  // Send frame buffer
  ssd1306_send_data(s_frame_buffer, sizeof(s_frame_buffer));
}

/**
 * @brief Draw a character at position
 */
static void draw_char(int x, int y, char c, int scale) {
  // Convert to uppercase if lowercase
  if (c >= 'a' && c <= 'z')
    c = c - 'a' + 'A';

  if (c < 32 || c > 90)
    c = '?';
  int idx = c - 32;

  for (int col = 0; col < 5; col++) {
    uint8_t line = font5x7[idx][col];
    for (int row = 0; row < 7; row++) {
      if (line & (1 << row)) {
        for (int sy = 0; sy < scale; sy++) {
          for (int sx = 0; sx < scale; sx++) {
            int px = x + col * scale + sx;
            int py = y + row * scale + sy;
            if (px >= 0 && px < OLED_WIDTH && py >= 0 && py < OLED_HEIGHT) {
              int page = py / 8;
              int bit = py % 8;
              s_frame_buffer[page * OLED_WIDTH + px] |= (1 << bit);
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Draw string at position
 */
static void draw_string(int x, int y, const char *str, int scale) {
  int char_width = 6 * scale; // 5 pixels + 1 space
  while (*str) {
    draw_char(x, y, *str, scale);
    x += char_width;
    str++;
  }
}

esp_err_t oled_init(void) {
  ESP_LOGI(TAG, "Initializing OLED display...");

  // Initialize I2C
  esp_err_t ret = init_i2c_oled();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(ret));
    return ret;
  }

  // Initialize SSD1306
  ret = ssd1306_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init SSD1306: %s", esp_err_to_name(ret));
    return ret;
  }

  // Create message queue
  s_oled_queue = xQueueCreate(5, sizeof(oled_msg_t));
  if (s_oled_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create OLED queue");
    return ESP_ERR_NO_MEM;
  }

  // Clear display
  oled_clear();

  // Show welcome message
  oled_show_message("VOICE RC", "READY", "SAY HI ESP");

  s_oled_initialized = true;
  ESP_LOGI(TAG, "OLED initialized successfully");
  return ESP_OK;
}

void oled_clear(void) {
  memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
  ssd1306_update();
}

void oled_show_state(rc_state_t state) {
  if (s_oled_queue == NULL)
    return;

  oled_msg_t msg = {0};
  msg.state = state;
  msg.update_state = true;
  xQueueSend(s_oled_queue, &msg, 0);
}

void oled_show_command(const char *command, float confidence) {
  if (s_oled_queue == NULL)
    return;

  oled_msg_t msg = {0};
  snprintf(msg.line1, sizeof(msg.line1), "DETECTED:");
  snprintf(msg.line2, sizeof(msg.line2), "%s", command);
  snprintf(msg.line3, sizeof(msg.line3), "CONF: %.0f%%", confidence * 100);
  msg.update_state = false;
  xQueueSend(s_oled_queue, &msg, 0);
}

void oled_show_message(const char *line1, const char *line2,
                       const char *line3) {
  memset(s_frame_buffer, 0, sizeof(s_frame_buffer));

  if (line1)
    draw_string(0, 0, line1, 2); // Large font at top
  if (line2)
    draw_string(0, 24, line2, 2); // Large font in middle
  if (line3)
    draw_string(0, 50, line3, 1); // Small font at bottom

  ssd1306_update();
}

void oled_display_task(void *param) {
  ESP_LOGI(TAG, "OLED display task started");

  // Wait for OLED init
  while (!s_oled_initialized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  oled_msg_t msg;

  while (1) {
    if (xQueueReceive(s_oled_queue, &msg, pdMS_TO_TICKS(100))) {
      if (msg.update_state) {
        // Update state display
        const char *state_str = state_name(msg.state);
        memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
        draw_string(0, 0, "STATE:", 1);
        draw_string(0, 16, state_str, 2);
        ssd1306_update();
      } else {
        // Show command
        memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
        draw_string(0, 0, msg.line1, 1);
        draw_string(0, 20, msg.line2, 2);
        draw_string(0, 50, msg.line3, 1);
        ssd1306_update();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
