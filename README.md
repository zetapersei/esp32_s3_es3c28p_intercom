# ESP32-S3 Full-Duplex Intercom
# English translation and adding hardware project

🎙️ **Full-duplex audio intercom** based on the ESP32-S3 with hardware echo cancellation, noise reduction, and a touchscreen.

![ESP32-S3-ES3C28P](doc/ES3C28P-03.png)

## ✨ Features

### Audio Processing
- **Full-duplex** — simultaneous voice transmission and reception
- **ESP-SR AEC** — acoustic echo cancellation with ~500ms tail (filter_len=16)
- **ESP-SR NS** — soft mode noise reduction
- **ESP-SR AGC** — automatic gain control (target -2dBFS)
- **ADPCM compression** — 4:1 compression ratio for efficient network transmission
- **16kHz / 16-bit mono** — audio pipeline
- **32ms frame size** (512 samples)

### Network
- **UDP streaming** — low-bandwidth audio transmission Latency
- **mDNS Discovery** — Automatic device search on the local network
- **WiFi Setup** — via touch interface

### User Interface
- **2.8" IPS touchscreen display** (240×320, ILI9341)
- **LVGL 9.x** — graphics library
- **English language translation**
- **Real-time audio level visualization**
- **List of devices** with intercoms detected via mDNS
- **Settings screen** with audio parameters

## 🛠️ Hardware

### Supported Board
**ESP32-S3-ES3C28P** — development board with a 2.8-inch display

| Component | Specifications |
|-----------|----------------|
| MCU | ESP32-S3 (QFN56, dual-core @ 240 MHz) |
| RAM | 512 KB SRAM + 8 MB PSRAM |
| Flash | 16 MB |
| Display | 2.8" IPS TFT (ILI9341, 240×320) |
| Touch | Capacitive (FT6336G) |
| Audio Codec | ES8311 |
| Amplifier | FM8002E |
| Microphone | MEMS |
| Communications | WiFi 2.4 GHz, Bluetooth 5.0 |

### Additional Board Features
- MicroSD Card Slot (4-bit SDIO)
- RGB LED (WS2812B)
- USB Type-C (programming and power)
- Battery Connector (3.7V Li-Po with charging circuit)

## Pinout

### LCD (ILI9341V)
| Function | GPIO | Description |
|---------|----------|
| CS | IO10 | Chip Select (active low) |
| DC | IO46 | Data/Command Select |
| SCK | IO12 | SPI Clock |
| MOSI | IO11 | SPI Data Output |
| MISO | IO13 | SPI Data Input |
| RST | RST | Reset (shared with ESP32-S3) |
| BL | IO45 | Backlight Control |

### Touch Screen (FT6336G)
| Function | GPIO | Description |
|---------|----------|
| SDA | IO16 | I2C Data |
| SCL | IO15 | I2C Clock |
| RST | IO18 | Touch Reset |
| INT | IO17 | Touch Interrupt |

### SD Card (SDIO)
| Function | GPIO | Description |
|---------|----------|
| CLK | IO38 | SDIO Clock |
| CMD | IO40 | SDIO Command |
| D0 | IO39 | Data Line 0 |
| D1 | IO41 | Data Line 1 |
| D2 | IO48 | Data Line 2 |
| D3 | IO47 | Data Line 3 |

### Audio (I2S)
| Function | GPIO | Description |
|---------|----------|
| AMP_EN | IO1 | Amplifier Enable (active low) |
| MCLK | IO4 | Master Clock |
| BCLK | IO5 | Bit Clock |
| DOUT | IO6 | Data Output (to Speaker) |
| LRCK | IO7 | Left/Right Clock |
| DIN | IO8 | Data Input (from Microphone) |

### Other Peripherals
| Function | GPIO | Description |
|---------|-------|----------|
| RGB LED | IO42 | WS2812B LED |
| BOOT | IO0 | Boot Mode Button |
| UART0 TX | IO44 | Debug UART |
| UART0 RX | IO43 | Debug UART |
| Battery ADC | IO9 | Battery Voltage Measurement |
| Expansion | IO2/3/14/21 | General Purpose GPIO |

## Specifications

### ESP32-S3 Parameters
- **CPU**: Xtensa LX7 dual-core @ 240 MHz
- **ROM**: 384 KB
- **SRAM**: 512 KB + 16 KB RTC SRAM
- **PSRAM**: 8 MB (internal OPI)
- **Flash**: 16 MB (external SPI)
- **Operating Voltage**: 3.0-3.6 V

### Display Parameters
- **Size**: 2.8 inches
- **Type**: IPS TFT
- **Resolution**: 240 x 320 pixels
- **Colors**: 262K (RGB666) / 65K (RGB565)
- **Driver**: ILI9341V
- **Interface**: 4-wire SPI
- **Backlight**: 4× white LEDs
- **Operating Temperature**: -30°C to +80°C

### Touchscreen Parameters
- **Type**: Capacitive
- **Driver**: FT6336G
- **Interface**: I2C
- **Operating Temperature**: -30°C to +80°C

### Power Specifications
- **Operating Voltage**: 5V (USB Type-C)
- **Backlight Current**: 79mA
- **Display Only**: ~140mA
- **Full Operation**: ~560mA (Display + Speaker + Charging)
- **Battery**: 3.7V Li-Po (with charging chip) TP4054)
- **Charging Current**: Max. 500mA, Typical 290mA

## BSP Components

The BSP provides the following components:

### Main Header Files
- `bsp/esp32_s3_es3c28p.h` - Main BSP Header
- `bsp/esp-bsp.h` - General BSP Definitions
- `bsp/config.h` - Board Configuration
- `bsp/display.h` - Display Interface
- `bsp/touch.h` - Touch Interface

### Source Files
Located in the `src/` directory:
- Display Driver Implementation
- Touch Controller Interface
- SD Card Support
- Audio Codec Configuration
- Board Initialization Routines

## 📦 Dependencies

- **ESP-IDF** v5.4+
- **ESP-SR** v2.3+ ​​(AEC, NS, AGC)
- **LVGL** v9.x
- **esp_codec_dev** ~1.5
- **esp_lcd_ili9341** ^2.0.1
- **esp_lcd_touch_ft5x06** ^
