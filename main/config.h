/**
 * @file config.h
 * @brief Configuration constants for Voice-Controlled RC System
 *
 * ESP32-S3 + ESP-SR (MultiNet) + I2S Microphone
 * Real-time, offline, safety-first design
 */

#ifndef CONFIG_H
#define CONFIG_H

/*===========================================================================
 * SYSTEM CONFIGURATION
 *===========================================================================*/

// Project identification
#define PROJECT_NAME "kata_pak_feri_acc"
#define PROJECT_VERSION "1.0.0"

/*===========================================================================
 * GPIO PIN CONFIGURATION
 *===========================================================================*/

// I2S Microphone Pins (adjust to your hardware)
#define GPIO_I2S_BCLK GPIO_NUM_41
#define GPIO_I2S_WS GPIO_NUM_42
#define GPIO_I2S_DIN GPIO_NUM_2

// LCD 16x2 I2C Display Pins
#define GPIO_LCD_SDA GPIO_NUM_8
#define GPIO_LCD_SCL GPIO_NUM_9
#define LCD_I2C_ADDR_PRIMARY 0x27   // Primary I2C address
#define LCD_I2C_ADDR_SECONDARY 0x3F // Fallback I2C address

// Navigation Buttons (Active LOW with Internal Pull-Up)
#define GPIO_BTN_UP GPIO_NUM_4
#define GPIO_BTN_DOWN GPIO_NUM_5
#define GPIO_BTN_OK GPIO_NUM_6
#define BTN_DEBOUNCE_MS 50

// Joystick (Analog X/Y + Digital SW)
#define GPIO_JOY_X GPIO_NUM_10 // ADC1 Channel for X axis
#define GPIO_JOY_Y GPIO_NUM_3  // ADC1 Channel for Y axis
#define GPIO_JOY_SW GPIO_NUM_7 // Joystick button (Active LOW)

// Joystick calibration settings
#define JOY_DEADZONE 30 // Deadzone untuk menghindari drift (diperbesar)
#define JOY_CALIBRATION_SAMPLES 20  // Jumlah sample saat kalibrasi
#define JOY_CALIBRATION_DELAY_MS 50 // Delay antar sample (total 1 detik)

// Invert axes (sesuaikan dengan joystick kamu)
// Sudah dicoba dan dikoreksi berdasarkan testing
#define INVERT_JOY_X false // Flip dari true
#define INVERT_JOY_Y true  // Flip dari false

// Status LED
#define GPIO_LED_STATUS GPIO_NUM_48

/*===========================================================================
 * TIMING CONFIGURATION (milliseconds)
 *===========================================================================*/

// Wake word and command listening
#define TIMEOUT_WAKEWORD_LISTEN_MS                                             \
  10000 // Max time listening after wake word (10 sec)
#define TIMEOUT_COMMAND_ACTIVITY_MS 5000 // Auto-stop if no new command (5 sec)

// Motor control
#define MOTOR_COMMAND_DEBOUNCE_MS                                              \
  200 // Minimum time between motor state changes

// Recognition
#define VOICE_RECOGNITION_LATENCY_MS 500 // Expected MultiNet processing time

// Watchdog
#define WATCHDOG_TIMEOUT_MS                                                    \
  5000 // Hardware watchdog timeout (5s, relaxed for init)
#define WATCHDOG_FEED_INTERVAL_MS 1000 // Feed interval

// Event processing
#define EVENT_QUEUE_WAIT_MS 50 // State machine poll interval

/*===========================================================================
 * CONFIDENCE THRESHOLDS
 *===========================================================================*/

// Recognition confidence thresholds (0.0 - 1.0)
#define CONFIDENCE_THRESHOLD_HIGH 0.85f // High confidence - immediate action
#define CONFIDENCE_THRESHOLD_MEDIUM                                            \
  0.70f                                // Medium confidence - action with delay
#define CONFIDENCE_THRESHOLD_LOW 0.50f // Below this = reject

// Active threshold for RC safety (use HIGH)
#define CONFIDENCE_THRESHOLD_MIN CONFIDENCE_THRESHOLD_HIGH

// Ambiguity detection
#define CONFIDENCE_GAP_MIN 0.15f // Min gap between top two results

/*===========================================================================
 * QUEUE CONFIGURATION
 *===========================================================================*/

#define EVT_QUEUE_LENGTH 16
#define EVT_QUEUE_TIMEOUT_MS 10 // Queue send timeout

/*===========================================================================
 * TASK CONFIGURATION
 *===========================================================================*/

// Stack sizes (words, not bytes)
#define STACK_SIZE_AUDIO_TASK (8 * 1024)
#define STACK_SIZE_STATE_MACHINE (4 * 1024)
#define STACK_SIZE_MOTOR_SAFETY (2 * 1024)
#define STACK_SIZE_WATCHDOG (2 * 1024)
#define STACK_SIZE_STATUS_LED (2 * 1024)
#define STACK_SIZE_OLED (4 * 1024) // OLED needs more stack for frame buffer

// Task priorities (higher number = higher priority)
#define PRIORITY_MOTOR_SAFETY 7 // Highest - safety critical
#define PRIORITY_STATE_MACHINE 6
#define PRIORITY_AUDIO_PROCESSING 5
#define PRIORITY_EVENT_PRODUCER 4
#define PRIORITY_WATCHDOG_FEEDER 3
#define PRIORITY_STATUS_LED 2

// Core affinity
#define CORE_SAFETY_CRITICAL 0  // Motors, state machine, watchdog
#define CORE_AUDIO_PROCESSING 1 // Audio, recognition

/*===========================================================================
 * AUDIO CONFIGURATION
 *===========================================================================*/

#define AUDIO_SAMPLE_RATE 16000  // 16 kHz required by ESP-SR
#define AUDIO_BITS_PER_SAMPLE 16 // 16-bit samples
#define AUDIO_CHANNEL_COUNT 1    // Mono

/*===========================================================================
 * COMMAND IDS (must match commands_en.txt)
 *===========================================================================*/

#define CMD_ID_FORWARD 1
#define CMD_ID_BACKWARD 2
#define CMD_ID_LEFT 3
#define CMD_ID_RIGHT 4
#define CMD_ID_STOP 5

/*===========================================================================
 * DEBUG CONFIGURATION
 *===========================================================================*/

#define ENABLE_DEBUG_LOGGING 1
#define ENABLE_TIMING_STATS 1

#endif // CONFIG_H
