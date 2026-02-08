# Voice-Controlled RC System (Transmitter)

ESP32-S3 based voice and joystick controlled RC transmitter using ESP-NOW protocol.

## 🎯 Fitur

- **Dual Mode Control:**
  - 🎤 **Voice Mode** - Kontrol dengan perintah suara (ESP-SR + WakeNet + MultiNet)  
  - 🕹️ **Remote Mode** - Kontrol dengan analog joystick

- **Mode Selection** - Pilih mode saat startup via LCD + tombol navigasi
- **Mode Isolation** - Voice dan joystick input terisolasi untuk mencegah konflik
- **ESP-NOW Communication** - Wireless low-latency ke receiver

---

## 📦 Hardware Requirements

### Transmitter (ESP32-S3)

| Komponen | GPIO | Keterangan |
|----------|------|------------|
| **INMP441 Mic** | | |
| - GND | GND | Ground |
| - L/R | GND | Mono Left |
| - SCK | GPIO 41 | BCLK |
| - WS | GPIO 42 | Word Select |
| - VDD | 3.3V | Power (3.3V Only!) |
| - SD | GPIO 2 | Data |
| **LCD 16x2 I2C** | | |
| - SDA | GPIO 8 | I2C Data |
| - SCL | GPIO 9 | I2C Clock |
| - VCC | 5V | Power |
| **Joystick** | | |
| - VRx | GPIO 10 | X axis (ADC1_CH9) |
| - VRy | GPIO 3 | Y axis (ADC1_CH2) |
| - SW | GPIO 7 | Button |
| - VCC | 5V | Power |
| **Buttons** | | Active LOW + Internal Pull-Up |
| - UP | GPIO 4 | Menu Up |
| - DOWN | GPIO 5 | Menu Down |
| - OK | GPIO 6 | Confirm |
| **LED** | GPIO 48 | Status indicator |

---

## 📡 ESP-NOW Protocol (UNTUK RECEIVER)

### Receiver MAC Address

Edit di `main/espnow_comm.c` baris 25:
```c
static uint8_t s_receiver_mac[ESP_NOW_ETH_ALEN] = {0xFC, 0x01, 0x2C, 0xD1, 0x76, 0x54};
```

### Packet Structure (7 bytes)

```c
typedef struct __attribute__((packed)) {
  uint8_t header;       // 0xAA - start marker
  uint8_t msg_type;     // Message type (lihat di bawah)
  uint8_t command;      // Command ID (lihat di bawah)
  uint8_t speed;        // Speed 0-255 (default: 200)
  uint16_t duration_ms; // Duration (0 = continuous)
  uint8_t checksum;     // XOR checksum
} espnow_packet_t;
```

### Message Types

| Value | Name | Keterangan |
|-------|------|------------|
| `0x01` | `MSG_TYPE_COMMAND` | Motor command |
| `0x02` | `MSG_TYPE_HEARTBEAT` | Keep-alive |
| `0x03` | `MSG_TYPE_ACK` | Acknowledgment |
| `0xFF` | `MSG_TYPE_EMERGENCY` | Emergency stop |

### Command IDs

| Value | Name | Action |
|-------|------|--------|
| `0` | `ESPNOW_CMD_STOP` | Berhenti |
| `1` | `ESPNOW_CMD_FORWARD` | Maju |
| `2` | `ESPNOW_CMD_BACKWARD` | Mundur |
| `3` | `ESPNOW_CMD_LEFT` | Belok kiri |
| `4` | `ESPNOW_CMD_RIGHT` | Belok kanan |
| `5` | `ESPNOW_CMD_STRAFE_LEFT` | Strafe kiri (mecanum) |
| `6` | `ESPNOW_CMD_STRAFE_RIGHT` | Strafe kanan (mecanum) |
| `0xFF` | `ESPNOW_CMD_EMERGENCY` | Emergency stop |

### Checksum Calculation

```c
uint8_t calculate_checksum(const uint8_t *packet, size_t len) {
  uint8_t checksum = 0;
  for (int i = 0; i < len - 1; i++) {  // Exclude checksum byte itself
    checksum ^= packet[i];
  }
  return checksum;
}
```

### WiFi Channel

Transmitter menggunakan **Channel 1**. Receiver harus set channel yang sama:
```c
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
```

---

## 🔧 Receiver Implementation Guide

### Minimum Receiver Code Structure

```c
#include "esp_now.h"
#include "esp_wifi.h"

// Packet structure (sama dengan transmitter)
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t msg_type;
  uint8_t command;
  uint8_t speed;
  uint16_t duration_ms;
  uint8_t checksum;
} espnow_packet_t;

// Receive callback
void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(espnow_packet_t)) return;
  
  espnow_packet_t *pkt = (espnow_packet_t *)data;
  
  // Validate header
  if (pkt->header != 0xAA) return;
  
  // Validate checksum
  uint8_t calc_checksum = calculate_checksum(data, len);
  if (calc_checksum != pkt->checksum) return;
  
  // Handle emergency stop
  if (pkt->msg_type == 0xFF || pkt->command == 0xFF) {
    stop_all_motors();
    return;
  }
  
  // Handle normal commands
  if (pkt->msg_type == 0x01) {
    execute_command(pkt->command, pkt->speed);
  }
}

void execute_command(uint8_t cmd, uint8_t speed) {
  switch (cmd) {
    case 0: stop_all_motors(); break;
    case 1: move_forward(speed); break;
    case 2: move_backward(speed); break;
    case 3: turn_left(speed); break;
    case 4: turn_right(speed); break;
    case 5: strafe_left(speed); break;
    case 6: strafe_right(speed); break;
  }
}
```

### Receiver Initialization

```c
void init_espnow_receiver() {
  // WiFi init
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // Channel 1!
  
  // ESP-NOW init
  esp_now_init();
  esp_now_register_recv_cb(on_data_recv);
  
  // Print MAC address (untuk dimasukkan ke transmitter)
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
```

---

## 🛠️ Build & Flash

```bash
# Set ESP-IDF environment
source ~/esp/esp-idf/export.sh  # Linux/Mac
# atau
call C:\Espressif\frameworks\esp-idf\export.bat  # Windows

# Build
cd kata_pak_feri_acc
idf.py build

# Flash
idf.py -p COM[X] flash monitor
```

---

## 📁 Project Structure

```
kata_pak_feri_acc/
├── main/
│   ├── audio_task.c      # Voice recognition (ESP-SR)
│   ├── button_input.c    # Navigation buttons
│   ├── config.h          # Pin & timing configuration
│   ├── espnow_comm.c     # ESP-NOW transmitter
│   ├── joystick_input.c  # Joystick with auto-calibration
│   ├── lcd_display.c     # LCD 16x2 I2C driver
│   ├── mode_manager.c    # Voice/Remote mode isolation
│   ├── state_machine.c   # State management
│   └── main.c            # Entry point
├── wiring.md             # Detailed wiring diagram
└── README.md             # This file
```

---

## 🎤 Voice Commands

| Wake Word | Commands |
|-----------|----------|
| "Hi ESP"  | FORWARD, BACKWARD, LEFT, RIGHT, STOP |

---

## ⚠️ Troubleshooting

### Joystick Tidak Akurat
1. Cek nilai `INVERT_JOY_X` dan `INVERT_JOY_Y` di `config.h`
2. Pastikan kalibrasi dilakukan saat joystick tidak disentuh

### ESP-NOW Tidak Terkirim
1. Pastikan MAC address receiver benar di `espnow_comm.c`
2. Channel harus sama (default: 1)
3. Receiver dalam mode `WIFI_MODE_STA`

### Voice Tidak Terdeteksi
1. INMP441 harus pakai 3.3V (bukan 5V!)
2. Pastikan dalam Voice Mode (bukan Remote Mode)
