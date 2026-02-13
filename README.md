# 🤖 Kata Pak Feri ACC — Voice-Controlled RC System

**Sistem kontrol RC berbasis suara offline menggunakan ESP32-S3, ESP-SR, dan ESP-NOW.**

> Dual Mode: 🎤 **Voice Control** (perintah suara offline) & 🕹️ **Remote Control** (analog joystick)

---

## ✨ Fitur Utama

| Fitur | Deskripsi |
|-------|-----------|
| 🎤 **Voice Mode** | Kontrol dengan perintah suara offline (ESP-SR + WakeNet + MultiNet) |
| 🕹️ **Remote Mode** | Kontrol dengan analog joystick + auto-calibration |
| 📺 **LCD 16x2 I2C** | Tampilan status, mode selection, dan feedback perintah |
| 📡 **ESP-NOW** | Komunikasi wireless low-latency ke receiver |
| 🔒 **Mode Isolation** | Voice dan joystick input terisolasi — mencegah konflik |
| 🛡️ **Fail-Safe** | Watchdog, timeout otomatis, emergency stop |
| 💡 **Status LED** | Indikator visual status sistem (GPIO 48) |

---

## 📦 Hardware Requirements

### Transmitter (ESP32-S3)

| Komponen | GPIO | Keterangan |
|----------|------|------------|
| **INMP441 Microphone** | | I2S Digital MEMS |
| - SCK | GPIO 41 | BCLK (Serial Clock) |
| - WS | GPIO 42 | Word Select |
| - SD | GPIO 2 | Serial Data |
| - VDD | 3.3V | ⚠️ **3.3V Only!** |
| - L/R | GND | Mono Left |
| - GND | GND | Ground |
| **LCD 16x2 I2C** | | PCF8574 Backpack |
| - SDA | GPIO 8 | I2C Data |
| - SCL | GPIO 9 | I2C Clock |
| - VCC | 5V | Power |
| - I2C Addr | — | Auto-detect `0x27` / `0x3F` |
| **Analog Joystick** | | |
| - VRx | GPIO 10 | X axis (ADC1) |
| - VRy | GPIO 3 | Y axis (ADC1) |
| - SW | GPIO 7 | Button (Active LOW) |
| - VCC | 5V | Power |
| **Navigation Buttons** | | Active LOW + Internal Pull-Up |
| - UP | GPIO 4 | Menu Up |
| - DOWN | GPIO 5 | Menu Down |
| - OK | GPIO 6 | Confirm / Double-press = kembali ke menu |
| **Status LED** | GPIO 48 | Melalui resistor 330Ω |

> 📌 Konfigurasi pin dapat diubah di `main/config.h`

---

## 🎤 Voice Commands

| Wake Word | Perintah | Command ID |
|-----------|----------|------------|
| **"Hi ESP"** | `FORWARD` | 1 |
| | `BACKWARD` | 2 |
| | `LEFT` | 3 |
| | `RIGHT` | 4 |
| | `STOP` | 5 |

**Konfigurasi Speech Recognition:**

| Parameter | Nilai |
|-----------|-------|
| Model Wake Word | WakeNet 9 |
| Model Command | MultiNet 6 (Quantized) |
| Bahasa | English |
| Confidence Threshold | 85% (High), 70% (Medium), 50% (Low) |
| Sample Rate | 16 kHz, 16-bit, Mono |

---

## 🔄 State Machine

Sistem menggunakan finite state machine dengan **9 state**:

```
┌──────────────┐    Wake Word     ┌───────────┐   Command    ┌──────────────┐
│  MODE_SELECT │───────────────▶ │ LISTENING │────────────▶│ MOVING_FWD   │
│  (Pilih Mode)│                  │           │              │ MOVING_BWD   │
└──────────────┘                  └───────────┘              │ TURNING_LEFT │
       │                               ▲                     │ TURNING_RIGHT│
       ▼                               │                     └──────┬───────┘
┌──────────────┐                       │   Timeout / Stop           │
│     IDLE     │◀──────────────────────┘◀───────────────────────────┘
│ (Voice Mode) │
└──────────────┘                  ┌───────────┐
                                  │  STOPPED  │
                                  └───────────┘
                                  ┌───────────┐
                                  │   ERROR   │ ← Auto-recovery
                                  └───────────┘
```

---

## 📡 ESP-NOW Protocol

### Receiver MAC Address

Edit di `main/espnow_comm.c`:
```c
static uint8_t s_receiver_mac[ESP_NOW_ETH_ALEN] = {0xFC, 0x01, 0x2C, 0xD1, 0x76, 0x54};
```

### Packet Structure (7 bytes)

```c
typedef struct __attribute__((packed)) {
  uint8_t header;       // 0xAA - start marker
  uint8_t msg_type;     // Message type
  uint8_t command;      // Command ID
  uint8_t speed;        // Speed 0-255 (default: 200)
  uint16_t duration_ms; // Duration (0 = continuous)
  uint8_t checksum;     // XOR checksum
} espnow_packet_t;
```

### Message Types

| Value | Nama | Keterangan |
|-------|------|------------|
| `0x01` | `MSG_TYPE_COMMAND` | Motor command |
| `0x02` | `MSG_TYPE_HEARTBEAT` | Keep-alive |
| `0x03` | `MSG_TYPE_ACK` | Acknowledgment |
| `0xFF` | `MSG_TYPE_EMERGENCY` | Emergency stop |

### Command IDs

| Value | Nama | Aksi |
|-------|------|------|
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
  for (int i = 0; i < len - 1; i++) {
    checksum ^= packet[i];
  }
  return checksum;
}
```

### WiFi Channel

Transmitter dan Receiver harus menggunakan **Channel 1**:
```c
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
```

---

## 🔧 Receiver Implementation Guide

### Minimum Receiver Code

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
# 1. Set ESP-IDF environment
# Windows:
call C:\Espressif\frameworks\esp-idf\export.bat
# Linux/Mac:
source ~/esp/esp-idf/export.sh

# 2. Set target
idf.py set-target esp32s3

# 3. Build
cd kata_pak_feri_acc
idf.py build

# 4. Flash + Monitor
idf.py -p COM[X] flash monitor

# 5. Clean build (jika diperlukan)
idf.py fullclean
```

### Partition Table

Proyek menggunakan custom partition table (`partitions.csv`) dengan flash **16MB**:

| Name | Type | Size | Keterangan |
|------|------|------|------------|
| `nvs` | data | 24KB | Non-volatile storage |
| `phy_init` | data | 4KB | PHY calibration |
| `factory` | app | 3MB | Application binary |
| `model` | data (spiffs) | ~5MB | ESP-SR model (WakeNet + MultiNet) |

---

## 📁 Struktur Proyek

```
kata_pak_feri_acc/
├── CMakeLists.txt                  # Root build configuration
├── partitions.csv                  # Custom partition table (16MB flash)
├── sdkconfig                       # ESP-IDF configuration
│
├── main/
│   ├── CMakeLists.txt              # Component build config
│   ├── idf_component.yml           # ESP-SR dependency declaration
│   │
│   ├── main.c                      # Entry point, task creation, system init
│   ├── config.h                    # Semua konfigurasi: GPIO, timing, threshold
│   ├── types.h                     # Type definitions, enums, structs
│   │
│   ├── audio_task.c / .h           # I2S mic, ESP-SR (WakeNet + MultiNet)
│   ├── state_machine.c / .h        # Finite state machine & transisi
│   ├── event_queue.c / .h          # FreeRTOS event queue (inter-task comm)
│   │
│   ├── mode_manager.c / .h         # Voice/Remote mode selection & isolation
│   ├── joystick_input.c / .h       # Analog joystick + auto-calibration
│   ├── button_input.c / .h         # Navigation buttons (UP/DOWN/OK)
│   │
│   ├── lcd_display.c / .h          # LCD 16x2 I2C driver (PCF8574)
│   ├── oled_display.c / .h         # OLED SSD1306 0.96" driver
│   │
│   ├── espnow_comm.c / .h          # ESP-NOW transmitter protocol
│   ├── motor_control.c / .h        # Motor direction & control logic
│   └── failsafe.c / .h             # Watchdog, timeout, emergency stop
│
├── model/                           # ESP-SR speech recognition models
│   └── (WakeNet + MultiNet data)
│
├── managed_components/              # Auto-downloaded ESP-IDF components
│   ├── espressif__esp-sr/           # Speech recognition library
│   └── espressif__esp-dsp/          # Digital signal processing
│
├── wiring.md                        # Diagram wiring detail lengkap
├── PANDUAN_PENGGUNAAN.md            # Panduan penggunaan end-user
├── DOKUMENTASI_PROYEK_KP.md         # Dokumentasi teknis untuk laporan KP
└── README.md                        # File ini
```

---

## 🧵 FreeRTOS Task Architecture

| Task | Priority | Core | Stack | Fungsi |
|------|----------|------|-------|--------|
| Motor Safety | 7 | 0 | 2KB | Safety-critical motor monitoring |
| State Machine | 6 | 0 | 4KB | Event processing & state transitions |
| Audio Processing | 5 | 1 | 8KB | I2S, AFE, WakeNet, MultiNet |
| Event Producer | 4 | — | — | Button/Joystick event generation |
| Watchdog Feeder | 3 | 0 | 2KB | System health monitoring |
| Status LED | 2 | 0 | 2KB | Visual feedback LED |

---

## ⚠️ Troubleshooting

### Joystick Tidak Akurat
1. Cek `INVERT_JOY_X` dan `INVERT_JOY_Y` di `config.h`
2. Pastikan kalibrasi dilakukan saat joystick tidak disentuh
3. Sesuaikan `JOY_DEADZONE` jika ada drift

### ESP-NOW Tidak Terkirim
1. Pastikan MAC address receiver benar di `espnow_comm.c`
2. WiFi channel harus sama (default: **Channel 1**)
3. Receiver harus dalam mode `WIFI_MODE_STA`

### Voice Tidak Terdeteksi
1. INMP441 **harus pakai 3.3V** (bukan 5V!)
2. Pastikan dalam **Voice Mode** (bukan Remote Mode)
3. Ucapkan wake word "Hi ESP" dengan jelas, jarak < 50cm
4. Cek wiring I2S: BCLK (GPIO 41), WS (GPIO 42), SD (GPIO 2)

### Build Error "PSRAM not detected"
1. ESP32-S3 harus memiliki **PSRAM 8MB**
2. Aktifkan PSRAM di `idf.py menuconfig`:
   - `Component config → ESP PSRAM → Support for external SPI-connected RAM`

### LCD Tidak Menyala
1. Cek VCC terhubung ke **5V**
2. Pastikan SDA/SCL tidak tertukar
3. Alamat I2C auto-detect: `0x27` atau `0x3F`

---

## 📚 Dokumentasi Tambahan

| Dokumen | Deskripsi |
|---------|-----------|
| [wiring.md](wiring.md) | Diagram wiring detail semua komponen |
| [PANDUAN_PENGGUNAAN.md](PANDUAN_PENGGUNAAN.md) | Panduan penggunaan lengkap |
| [DOKUMENTASI_PROYEK_KP.md](DOKUMENTASI_PROYEK_KP.md) | Dokumentasi teknis untuk laporan KP |

---

## 📑 Referensi

- [ESP-SR Documentation](https://github.com/espressif/esp-sr)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/)
- [ESP32-S3 Technical Reference](https://www.espressif.com/en/products/socs/esp32-s3)
- [FreeRTOS Documentation](https://www.freertos.org/)
- [ESP-NOW Protocol Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)

---

<sub>📌 **Proyek:** Kata Pak Feri ACC — Voice Command Recognition System &nbsp;|&nbsp; **Platform:** ESP32-S3 + ESP-SR + ESP-NOW &nbsp;|&nbsp; **Version:** 1.0.0</sub>
