# Wiring Diagram - Voice-Controlled RC System

## Diagram Keseluruhan

```
                    ┌──────────────────────────────────────────────────────────────┐
                    │                    ESP32-S3 DevKit                            │
                    │                                                                │
                    │   ┌─────┐                                          ┌─────┐   │
   INMP441 ─────────│───┤GPIO2│ DIN                               GPIO48├─────│───│─── Status LED
      ↓             │   │GPIO41│ BCLK                                    │     │   │      ↓
   Microphone       │   │GPIO42│ WS                                      └─────┘   │   [LED + R330Ω]
                    │   └─────┘                                                     │
                    │                                                                │
                    │   ┌─────┐                                          ┌─────┐   │
   LCD 16x2 ────────│───┤GPIO8│ SDA                               GPIO4 ├─────│───│─── BTN UP
     I2C            │   │GPIO9│ SCL                               GPIO5 ├─────│───│─── BTN DOWN
                    │   └─────┘                                   GPIO6 ├─────│───│─── BTN OK
                    │                                                    └─────┘   │
                    │   ┌─────┐                                                     │
   Joystick ────────│───┤GPIO10│ VRx (Analog X)                                    │
                    │   │GPIO11│ VRy (Analog Y)                                    │
                    │   │GPIO7 │ SW (Button)                                       │
                    │   └─────┘                                                     │
                    │                                                                │
                    │   [5V]  [3.3V]  [GND]                                         │
                    └─────┬──────┬──────┬─────────────────────────────────────────┘
                          │      │      │
                          ▼      ▼      ▼
                    Power Distribution
```

---

## 1. I2S Microphone (INMP441)

**Format Pin (6 Pin):** GND, L/R, SCK, WS, VDD, SD

| INMP441 Pin | ESP32-S3 Pin | Keterangan |
|-------------|--------------|------------|
| GND | GND | Ground |
| L/R | GND | Mono Left Channel |
| SCK | GPIO 41 | Serial Clock (BCLK) |
| WS | GPIO 42 | Word Select (LRCLK) |
| VDD | 3.3V | Power supply |
| SD | GPIO 2 | Serial Data (DIN) |

**Wiring:**
```
┌────────────────────────────┐
│  INMP441 (6-Pin Module)    │
│                            │
│  GND ──────────── GND      │
│  L/R ──────────── GND      │
│  SCK ──────────── GPIO 41  │
│  WS  ──────────── GPIO 42  │
│  VDD ──────────── 3.3V     │
│  SD  ──────────── GPIO 2   │
│                            │
└────────────────────────────┘
```

> **Warning:** INMP441 membutuhkan 3.3V, jangan hubungkan ke 5V!

---

## 2. LCD 16x2 I2C (PCF8574)

| LCD Pin | ESP32-S3 Pin | Keterangan |
|---------|--------------|------------|
| VCC | 5V | Power supply |
| GND | GND | Ground |
| SDA | GPIO 8 | I2C Data |
| SCL | GPIO 9 | I2C Clock |

**Alamat I2C:** Otomatis deteksi `0x27` atau `0x3F`

---

## 3. Navigation Buttons (3x)

| Button | ESP32-S3 Pin | Keterangan |
|--------|--------------|------------|
| UP | GPIO 4 | Active LOW, Internal Pull-Up |
| DOWN | GPIO 5 | Active LOW, Internal Pull-Up |
| OK | GPIO 6 | Active LOW, Internal Pull-Up |

**Wiring:**
```
           ┌─────────┐
GPIO 4 ────┤ BTN UP  ├──── GND
           └─────────┘
           ┌─────────┐
GPIO 5 ────┤BTN DOWN ├──── GND
           └─────────┘
           ┌─────────┐
GPIO 6 ────┤ BTN OK  ├──── GND
           └─────────┘
```

> Internal pull-up resistor diaktifkan, tidak perlu resistor eksternal

---

## 4. Analog Joystick

| Joystick Pin | ESP32-S3 Pin | Keterangan |
|--------------|--------------|------------|
| VCC | 5V | Power supply |
| GND | GND | Ground |
| VRx | GPIO 10 | Analog X-axis (ADC1_CH9) |
| VRy | GPIO 11 | Analog Y-axis (ADC1_CH0) |
| SW | GPIO 7 | Push button (Active LOW) |

**Wiring:**
```
              ┌─────────────┐
    5V ───────┤ VCC         │
   GND ───────┤ GND         │
GPIO 10 ──────┤ VRx     SW  ├────── GPIO 7
GPIO 11 ──────┤ VRy         │
              └─────────────┘
```

---

## 5. Status LED

| LED | ESP32-S3 Pin | Keterangan |
|-----|--------------|------------|
| Anode (+) | GPIO 48 | Melalui resistor 330Ω |
| Cathode (-) | GND | Ground |

**Wiring:**
```
GPIO 48 ──── [330Ω] ──── LED ──── GND
```

---

## Diagram Power

```
┌──────────────────────────────────────────────────────────┐
│                    POWER DISTRIBUTION                     │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   USB/5V ────┬──────────── ESP32-S3 DevKit              │
│              │                   │                       │
│              ├──── LCD 16x2 VCC  │                       │
│              │                   │                       │
│              └──── Joystick VCC  │                       │
│                                  │                       │
│   3.3V (dari ESP32) ─────────────┼──── INMP441 VDD      │
│                                  │                       │
│   GND ───────┬───────────────────┘                       │
│              ├──── Semua komponen GND                    │
│              ├──── Button Common                         │
│              └──── LED Cathode                           │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## Pin Summary

| GPIO | Function | Component |
|------|----------|-----------|
| 2 | I2S DIN | INMP441 SD |
| 4 | Button | BTN UP |
| 5 | Button | BTN DOWN |
| 6 | Button | BTN OK |
| 7 | Digital Input | Joystick SW |
| 8 | I2C SDA | LCD |
| 9 | I2C SCL | LCD |
| 10 | ADC Input | Joystick VRx |
| 11 | ADC Input | Joystick VRy |
| 41 | I2S BCLK | INMP441 SCK |
| 42 | I2S WS | INMP441 WS |
| 48 | Digital Output | Status LED |

---

## Catatan Penting

> [!WARNING]
> **Tegangan:** INMP441 hanya 3.3V, LCD dan Joystick bisa 5V

> [!TIP]
> **Common Ground:** Pastikan semua GND terhubung bersama

> [!NOTE]
> **I2C Address:** LCD otomatis coba 0x27 dan 0x3F
