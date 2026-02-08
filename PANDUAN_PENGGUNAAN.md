# Panduan Penggunaan - Kata Pak Feri ACC

Sistem pengenalan suara offline menggunakan ESP32-S3 + OLED 0.96".

---

## Daftar Isi

1. [Kebutuhan Hardware](#1-kebutuhan-hardware)
2. [Wiring](#2-wiring)
3. [Konfigurasi GPIO](#3-konfigurasi-gpio)
4. [Build & Flash](#4-build--flash)
5. [Cara Penggunaan](#5-cara-penggunaan)
6. [Troubleshooting](#6-troubleshooting)

---

## 1. Kebutuhan Hardware

| Komponen | Spesifikasi |
|----------|-------------|
| ESP32-S3 DevKit | Dengan PSRAM 8MB |
| OLED Display | SSD1306 0.96" I2C |
| Mikrofon I2S | INMP441 / SPH0645 |
| Kabel Jumper | Secukupnya |

---

## 2. Wiring

### OLED SSD1306 (I2C)

| Pin OLED | Pin ESP32-S3 |
|----------|--------------|
| VCC | 3.3V |
| GND | GND |
| SDA | **GPIO 8** |
| SCL | **GPIO 9** |

### Mikrofon I2S (INMP441)

| Pin Mikrofon | Pin ESP32-S3 |
|--------------|--------------|
| VDD | 3.3V |
| GND | GND |
| SCK | GPIO 41 |
| WS | GPIO 42 |
| SD | GPIO 2 |
| L/R | GND |

---

## 3. Konfigurasi GPIO

Edit `main/config.h` jika pin berbeda:

```c
// OLED Display (I2C)
#define GPIO_OLED_SDA GPIO_NUM_8
#define GPIO_OLED_SCL GPIO_NUM_9

// I2S Microphone
#define GPIO_I2S_BCLK GPIO_NUM_41
#define GPIO_I2S_WS   GPIO_NUM_42
#define GPIO_I2S_DIN  GPIO_NUM_2
```

---

## 4. Build & Flash

### Langkah 1: Buka Terminal ESP-IDF

**Windows:** Buka "ESP-IDF 5.1 CMD" dari Start Menu

### Langkah 2: Masuk ke Folder Project

```bash
cd c:\Users\infinix\kata_pak_feri_acc
```

### Langkah 3: Set Target

```bash
idf.py set-target esp32s3
```

### Langkah 4: Konfigurasi

```bash
idf.py menuconfig
```

Aktifkan:
- `Component config → ESP PSRAM → Support for external SPI-connected RAM`
- `Component config → ESP-SR → WakeNet & MultiNet`

### Langkah 5: Build

```bash
idf.py build
```

### Langkah 6: Flash

```bash
idf.py -p COM3 flash monitor
```

> ⚠️ Ganti `COM3` dengan port Anda

**Jika gagal flash:**
1. Tekan **BOOT** + **RESET** bersamaan
2. Lepas **RESET** dulu, lalu **BOOT**
3. Jalankan flash lagi

---

## 5. Cara Penggunaan

### Alur Penggunaan

```
1. Nyalakan → OLED tampilkan "VOICE RC READY"
2. Ucapkan "Hi ESP" → OLED tampilkan "LISTENING"
3. Ucapkan perintah → OLED tampilkan perintah + confidence
4. Jika timeout → Kembali ke IDLE
```

### Perintah Suara

| Perintah | Tampilan OLED |
|----------|---------------|
| "FORWARD" | FORWARD + conf% |
| "BACKWARD" | BACKWARD + conf% |
| "LEFT" | LEFT + conf% |
| "RIGHT" | RIGHT + conf% |
| "STOP" | STOPPED |

### Tampilan OLED

| Status | Tampilan |
|--------|----------|
| Idle | "VOICE RC" / "IDLE" |
| Listening | "LISTENING" / "SAY COMMAND" |
| Command | Nama perintah + confidence |
| Error | "ERROR!" / "RECOVERING" |

---

## 6. Troubleshooting

### OLED Tidak Menyala

- Cek VCC terhubung ke 3.3V
- Cek GND terhubung
- Pastikan SDA/SCL tidak tertukar
- Alamat I2C default: 0x3C

### Wake Word Tidak Terdeteksi

- Ucapkan dengan jelas "Hi ESP"
- Jarak ke mikrofon < 50cm
- Cek wiring I2S (BCLK, WS, SD)

### Build Error "PSRAM not detected"

- ESP32-S3 Anda harus memiliki PSRAM
- Aktifkan PSRAM di menuconfig

---

## Command Reference

```bash
# Set target
idf.py set-target esp32s3

# Konfigurasi
idf.py menuconfig

# Build
idf.py build

# Flash + Monitor
idf.py -p COM3 flash monitor

# Clean build
idf.py fullclean
```

---

## Struktur File

```
kata_pak_feri_acc/
├── main/
│   ├── main.c              # Entry point
│   ├── config.h            # Konfigurasi GPIO
│   ├── audio_task.c        # Pemrosesan audio
│   ├── oled_display.c      # Driver OLED SSD1306
│   ├── state_machine.c     # State machine
│   ├── event_queue.c       # Event queue
│   └── failsafe.c          # Fail-safe
└── model/
    └── multinet_model/
        └── fst/
            └── commands_en.txt  # Daftar perintah
```