/**
 * @file lcd_display.c
 * @brief LCD 16x2 I2C Display implementation
 *
 * Uses HD44780 compatible LCD with PCF8574 I2C expander
 */

#include "lcd_display.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "LCD";

// I2C configuration
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000 // PCF8574 hanya support 100kHz
#define I2C_TIMEOUT_MS 100

// LCD commands
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x06
#define LCD_CMD_DISPLAY_ON 0x0C
#define LCD_CMD_DISPLAY_OFF 0x08
#define LCD_CMD_FUNCTION_SET 0x28 // 4-bit, 2 lines, 5x8 font
#define LCD_CMD_SET_DDRAM 0x80

// PCF8574 pin mapping (directly matching common LCD I2C backpack)
#define LCD_RS (1 << 0)
#define LCD_RW (1 << 1)
#define LCD_EN (1 << 2)
#define LCD_BL (1 << 3)
#define LCD_D4 (1 << 4)
#define LCD_D5 (1 << 5)
#define LCD_D6 (1 << 6)
#define LCD_D7 (1 << 7)

// State
static uint8_t s_lcd_addr = 0;
static uint8_t s_backlight = LCD_BL;
static bool s_initialized = false;
static QueueHandle_t s_lcd_queue = NULL;

// Message types
typedef struct {
  char line1[17];
  char line2[17];
  uint8_t msg_type;     // 0=message, 1=menu, 2=command, 3=settings
  uint8_t param;        // For menu: selected index
  float confidence;     // For command
  uint16_t duration_ms; // For settings
} lcd_msg_t;

/*===========================================================================
 * I2C Communication
 *===========================================================================*/

static esp_err_t lcd_write_byte(uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (s_lcd_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, data | s_backlight, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void lcd_pulse_enable(uint8_t data) {
  lcd_write_byte(data | LCD_EN);
  vTaskDelay(pdMS_TO_TICKS(1));
  lcd_write_byte(data & ~LCD_EN);
  vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_send_nibble(uint8_t nibble, bool is_data) {
  uint8_t data = (nibble << 4) & 0xF0;
  if (is_data)
    data |= LCD_RS;
  lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, bool is_data) {
  lcd_send_nibble(byte >> 4, is_data);
  lcd_send_nibble(byte & 0x0F, is_data);
}

static void lcd_send_cmd(uint8_t cmd) { lcd_send_byte(cmd, false); }

static void lcd_send_data(uint8_t data) { lcd_send_byte(data, true); }

/*===========================================================================
 * I2C Initialization
 *===========================================================================*/

static esp_err_t init_i2c(void) {
  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = GPIO_LCD_SDA,
                       .scl_io_num = GPIO_LCD_SCL,
                       .sda_pullup_en = GPIO_PULLUP_ENABLE,
                       .scl_pullup_en = GPIO_PULLUP_ENABLE,
                       .master.clk_speed = I2C_MASTER_FREQ_HZ};

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (ret != ESP_OK)
    return ret;

  ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
  if (ret == ESP_ERR_INVALID_STATE) {
    // Already installed
    ret = ESP_OK;
  }
  return ret;
}

static bool detect_lcd_address(void) {
  // Try primary address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LCD_I2C_ADDR_PRIMARY << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    s_lcd_addr = LCD_I2C_ADDR_PRIMARY;
    ESP_LOGI(TAG, "LCD found at address 0x%02X", s_lcd_addr);
    return true;
  }

  // Try secondary address
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LCD_I2C_ADDR_SECONDARY << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_stop(cmd);
  ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    s_lcd_addr = LCD_I2C_ADDR_SECONDARY;
    ESP_LOGI(TAG, "LCD found at address 0x%02X", s_lcd_addr);
    return true;
  }

  ESP_LOGE(TAG, "LCD not found at 0x%02X or 0x%02X", LCD_I2C_ADDR_PRIMARY,
           LCD_I2C_ADDR_SECONDARY);
  return false;
}

/*===========================================================================
 * Public Functions
 *===========================================================================*/

esp_err_t lcd_init(void) {
  ESP_LOGI(TAG, "Initializing LCD 16x2 I2C...");

  esp_err_t ret = init_i2c();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(50)); // Wait for LCD power up

  if (!detect_lcd_address()) {
    return ESP_ERR_NOT_FOUND;
  }

  // Initialize in 4-bit mode
  lcd_send_nibble(0x03, false);
  vTaskDelay(pdMS_TO_TICKS(5));
  lcd_send_nibble(0x03, false);
  vTaskDelay(pdMS_TO_TICKS(5));
  lcd_send_nibble(0x03, false);
  vTaskDelay(pdMS_TO_TICKS(1));
  lcd_send_nibble(0x02, false); // Switch to 4-bit mode
  vTaskDelay(pdMS_TO_TICKS(1));

  // Configure LCD
  lcd_send_cmd(LCD_CMD_FUNCTION_SET);
  lcd_send_cmd(LCD_CMD_DISPLAY_OFF);
  lcd_send_cmd(LCD_CMD_CLEAR);
  vTaskDelay(pdMS_TO_TICKS(2));
  lcd_send_cmd(LCD_CMD_ENTRY_MODE);
  lcd_send_cmd(LCD_CMD_DISPLAY_ON);

  // Create message queue
  s_lcd_queue = xQueueCreate(5, sizeof(lcd_msg_t));
  if (s_lcd_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create LCD queue");
    return ESP_ERR_NO_MEM;
  }

  s_initialized = true;
  ESP_LOGI(TAG, "LCD initialized successfully");

  // Show startup message
  lcd_show_message("VOICE RC v1.0", "STARTING...");

  return ESP_OK;
}

void lcd_clear(void) {
  if (!s_initialized)
    return;
  lcd_send_cmd(LCD_CMD_CLEAR);
  vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
  if (!s_initialized)
    return;
  uint8_t addr = (row == 0) ? col : (0x40 + col);
  lcd_send_cmd(LCD_CMD_SET_DDRAM | addr);
}

void lcd_print(const char *str) {
  if (!s_initialized || str == NULL)
    return;
  while (*str) {
    lcd_send_data((uint8_t)*str++);
  }
}

void lcd_printf(const char *fmt, ...) {
  if (!s_initialized)
    return;
  char buf[17];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  lcd_print(buf);
}

void lcd_backlight(bool on) {
  s_backlight = on ? LCD_BL : 0;
  lcd_write_byte(s_backlight);
}

void lcd_show_message(const char *line1, const char *line2) {
  if (!s_initialized)
    return;

  lcd_msg_t msg;
  msg.msg_type = 0;
  strncpy(msg.line1, line1 ? line1 : "", 16);
  msg.line1[16] = '\0';
  strncpy(msg.line2, line2 ? line2 : "", 16);
  msg.line2[16] = '\0';

  xQueueSend(s_lcd_queue, &msg, pdMS_TO_TICKS(10));
}

void lcd_show_mode_menu(uint8_t selected_mode) {
  if (!s_initialized)
    return;

  lcd_msg_t msg;
  msg.msg_type = 1;
  msg.param = selected_mode;

  xQueueSend(s_lcd_queue, &msg, pdMS_TO_TICKS(10));
}

void lcd_show_state(rc_state_t state) {
  const char *state_str;
  switch (state) {
  case STATE_MODE_SELECT:
    state_str = "MODE SELECT";
    break;
  case STATE_IDLE:
    state_str = "IDLE";
    break;
  case STATE_LISTENING:
    state_str = "LISTENING...";
    break;
  case STATE_MOVING_FWD:
    state_str = "FORWARD";
    break;
  case STATE_MOVING_BWD:
    state_str = "BACKWARD";
    break;
  case STATE_TURNING_LEFT:
    state_str = "LEFT";
    break;
  case STATE_TURNING_RIGHT:
    state_str = "RIGHT";
    break;
  case STATE_STOPPED:
    state_str = "STOPPED";
    break;
  case STATE_ERROR:
    state_str = "ERROR!";
    break;
  default:
    state_str = "UNKNOWN";
    break;
  }

  lcd_show_message("STATE:", state_str);
}

void lcd_show_command(const char *command, float confidence) {
  if (!s_initialized)
    return;

  lcd_msg_t msg;
  msg.msg_type = 2;
  strncpy(msg.line1, command ? command : "", 16);
  msg.line1[16] = '\0';
  msg.confidence = confidence;

  xQueueSend(s_lcd_queue, &msg, pdMS_TO_TICKS(10));
}

void lcd_show_joystick(const char *direction) {
  lcd_show_message("JOYSTICK:", direction);
}

void lcd_show_settings(uint16_t duration_ms) {
  if (!s_initialized)
    return;

  lcd_msg_t msg;
  msg.msg_type = 3;
  msg.duration_ms = duration_ms;

  xQueueSend(s_lcd_queue, &msg, pdMS_TO_TICKS(10));
}

/*===========================================================================
 * LCD Display Task
 *===========================================================================*/

static void render_message(const lcd_msg_t *msg) {
  lcd_clear();

  // Calculate padding for centering
  int len1 = strlen(msg->line1);
  int pad1 = (16 - len1) / 2;
  if (pad1 < 0)
    pad1 = 0;

  int len2 = strlen(msg->line2);
  int pad2 = (16 - len2) / 2;
  if (pad2 < 0)
    pad2 = 0;

  lcd_set_cursor(0, pad1);
  lcd_print(msg->line1);

  lcd_set_cursor(1, pad2);
  lcd_print(msg->line2);
}

static void render_mode_menu(uint8_t selected) {
  lcd_clear();

  // 3 menu items, LCD has 2 rows -> scrolling window
  // Items: Voice(0), Remote(1), Settings(2)
  const char *items[] = {"VOICE", "REMOTE", "SETTINGS"};
  const int item_count = 3;

  // Calculate which 2 items to show
  int top_item = 0;
  if (selected >= 1 && selected < item_count - 1) {
    top_item = selected - 1; // Keep selected in view
  } else if (selected >= item_count - 1) {
    top_item = item_count - 2; // Show last 2
  }

  // Draw 2 visible rows
  for (int row = 0; row < 2; row++) {
    int idx = top_item + row;
    if (idx >= item_count)
      break;

    lcd_set_cursor(row, 0);
    char buf[17];
    snprintf(buf, sizeof(buf), "%c %s", (idx == selected) ? '>' : ' ',
             items[idx]);
    lcd_print(buf);
  }
}

static void render_command(const char *cmd, float confidence) {
  lcd_clear();

  // Center command on line 1
  int len = strlen(cmd);
  int pad = (16 - len) / 2;
  if (pad < 0)
    pad = 0;

  lcd_set_cursor(0, pad);
  lcd_print(cmd);

  // Show confidence on line 2
  char buf[17];
  int pct = (int)(confidence * 100);
  snprintf(buf, sizeof(buf), "CONF: %d%%", pct);

  len = strlen(buf);
  pad = (16 - len) / 2;
  if (pad < 0)
    pad = 0;

  lcd_set_cursor(1, pad);
  lcd_print(buf);
}

static void render_settings(uint16_t duration_ms) {
  lcd_clear();

  lcd_set_cursor(0, 0);
  lcd_print("VOICE DURATION:");

  char buf[17];
  snprintf(buf, sizeof(buf), "  %d.%ds  [OK]", duration_ms / 1000,
           (duration_ms % 1000) / 100);

  lcd_set_cursor(1, 0);
  lcd_print(buf);
}

void lcd_display_task(void *param) {
  ESP_LOGI(TAG, "LCD display task started on core %d", xPortGetCoreID());

  lcd_msg_t msg;

  while (1) {
    if (xQueueReceive(s_lcd_queue, &msg, pdMS_TO_TICKS(100))) {
      switch (msg.msg_type) {
      case 0: // Message
        render_message(&msg);
        break;
      case 1: // Mode menu
        render_mode_menu(msg.param);
        break;
      case 2: // Command with confidence
        render_command(msg.line1, msg.confidence);
        break;
      case 3: // Settings
        render_settings(msg.duration_ms);
        break;
      }
    }
  }
}
