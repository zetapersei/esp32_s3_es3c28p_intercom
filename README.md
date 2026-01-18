# ESP32-S3-ES3C28P Board Support Package (BSP)

Board Support Package for the 2.8-inch ESP32-S3 Display Development Board (ES3C28P/ES3N28P).

## Overview

The ESP32-S3-ES3C28P is a comprehensive development board featuring a 2.8-inch IPS TFT display with optional capacitive touch screen, designed for embedded GUI applications, IoT devices, and multimedia projects.

## Features

- **Main Controller**: ESP32-S3 (Dual-core Xtensa LX7 @ 240MHz)
- **Memory**: 
  - 512KB SRAM
  - 8MB PSRAM
  - 16MB external SPI Flash
- **Display**: 
  - 2.8-inch IPS TFT LCD
  - Resolution: 240×320 pixels
  - Driver IC: ILI9341V
  - Interface: 4-Line SPI
  - Brightness: 280 cd/m²
- **Touch Screen** (ES3C28P variant):
  - Capacitive touch
  - Driver IC: FT6336G
  - Interface: I2C
- **Connectivity**:
  - WiFi 2.4GHz (802.11b/g/n)
  - Bluetooth 5.0 (BR/EDR and BLE)
- **Interfaces**:
  - MicroSD card slot (SDIO 4-bit)
  - Audio I2S (speaker output + microphone input)
  - RGB LED (WS2812B)
  - USB Type-C (programming and power)
  - UART, I2C expansion interfaces
  - Battery connector (3.7V Li-Po with charging circuit)
- **Audio**:
  - Codec: ES8311
  - Amplifier: FM8002E
  - MEMS Microphone
  - Speaker connector (1.5W/8Ω or 2W/4Ω)

## Pin Mapping

### LCD (ILI9341V)
| Function | GPIO | Description |
|----------|------|-------------|
| CS | IO10 | Chip select (active low) |
| DC | IO46 | Data/Command select |
| SCK | IO12 | SPI clock |
| MOSI | IO11 | SPI data out |
| MISO | IO13 | SPI data in |
| RST | RST | Reset (shared with ESP32-S3) |
| BL | IO45 | Backlight control |

### Touch Screen (FT6336G)
| Function | GPIO | Description |
|----------|------|-------------|
| SDA | IO16 | I2C data |
| SCL | IO15 | I2C clock |
| RST | IO18 | Touch reset |
| INT | IO17 | Touch interrupt |

### SD Card (SDIO)
| Function | GPIO | Description |
|----------|------|-------------|
| CLK | IO38 | SDIO clock |
| CMD | IO40 | SDIO command |
| D0 | IO39 | Data line 0 |
| D1 | IO41 | Data line 1 |
| D2 | IO48 | Data line 2 |
| D3 | IO47 | Data line 3 |

### Audio (I2S)
| Function | GPIO | Description |
|----------|------|-------------|
| AMP_EN | IO1 | Amplifier enable (active low) |
| MCLK | IO4 | Master clock |
| BCLK | IO5 | Bit clock |
| DOUT | IO6 | Data output (to speaker) |
| LRCK | IO7 | Left/Right clock |
| DIN | IO8 | Data input (from mic) |

### Other Peripherals
| Function | GPIO | Description |
|----------|------|-------------|
| RGB LED | IO42 | WS2812B LED |
| BOOT | IO0 | Boot mode button |
| UART0 TX | IO44 | Debug UART |
| UART0 RX | IO43 | Debug UART |
| Battery ADC | IO9 | Battery voltage sense |
| Expansion | IO2/3/14/21 | General purpose I/O |

## Hardware Specifications

### ESP32-S3 Parameters
- **CPU**: Xtensa LX7 dual-core @ 240MHz
- **ROM**: 384KB
- **SRAM**: 512KB + 16KB RTC SRAM
- **PSRAM**: 8MB (internal OPI)
- **Flash**: 16MB (external SPI)
- **Operating Voltage**: 3.0-3.6V

### Display Parameters
- **Size**: 2.8 inch
- **Type**: IPS TFT
- **Resolution**: 240×320 pixels
- **Colors**: 262K (RGB666) / 65K (RGB565)
- **Driver**: ILI9341V
- **Interface**: 4-wire SPI
- **Backlight**: 4× white LEDs
- **Operating Temp**: -30°C to +80°C

### Touch Screen Parameters
- **Type**: Capacitive
- **Driver**: FT6336G
- **Interface**: I2C
- **Operating Temp**: -30°C to +80°C

### Power Specifications
- **Operating Voltage**: 5V (USB Type-C)
- **Backlight Current**: 79mA
- **Display Only**: ~140mA
- **Full Operation**: ~560mA (display + speaker + charging)
- **Battery**: 3.7V Li-Po (with TP4054 charging IC)
- **Charging Current**: Max 500mA, Typical 290mA

## BSP Components

This BSP provides the following components:

### Core Headers
- `bsp/esp32_s3_es3c28p.h` - Main BSP header
- `bsp/esp-bsp.h` - Common BSP definitions
- `bsp/config.h` - Board configuration
- `bsp/display.h` - Display interface
- `bsp/touch.h` - Touch interface

### Source Files
Located in `src/` directory:
- Display driver implementation
- Touch controller interface
- SD card support
- Audio codec configuration
- Board initialization routines

## Dependencies

- **ESP-IDF**: >= 5.4
- **esp_lcd_ili9341**: ^2.0.1 (LCD driver)
- **esp_lcd_touch_ft5x06**: ^1.0.7 (Touch driver)
- **esp_codec_dev**: ~1.5 (Audio codec)
- **esp_lvgl_port**: ^2 (LVGL graphics library)

## Usage

### Basic Initialization

```c
#include "bsp/esp32_s3_es3c28p.h"

void app_main(void)
{
    // Initialize the board
    bsp_init();
    
    // Initialize display
    bsp_display_start();
    
    // Initialize touch (if available)
    bsp_touch_start();
    
    // Your application code here
}
```

### Display Control

```c
#include "bsp/display.h"

// Get display handle
esp_lcd_panel_handle_t lcd = bsp_display_get_handle();

// Backlight control
bsp_display_backlight_on();
bsp_display_backlight_off();
bsp_display_brightness_set(50); // 0-100%
```

### Touch Input

```c
#include "bsp/touch.h"

// Read touch coordinates
uint16_t x, y;
bool touched = bsp_touch_get_coordinates(&x, &y);
if (touched) {
    printf("Touch at: %d, %d\n", x, y);
}
```

## Configuration Options

Configuration options are available through menuconfig:

```bash
idf.py menuconfig
```

Navigate to: `Component config` → `Board Support Package (BSP)`

## Product Variants

- **ES3C28P**: With capacitive touch screen
- **ES3N28P**: Without touch screen

Both variants share the same pinout and features except for touch functionality.

## Documentation

For detailed information, refer to:
- **Full Documentation**: https://www.lcdwiki.com/2.8inch_ESP32-S3_Display
- **Hardware Schematic**: Available in documentation package
- **User Manual**: See product documentation
- **Example Code**: ESP-IDF examples included in data package

## Physical Dimensions

- **Product Size**: 50.00mm (W) × 86.00mm (H) × 10.60mm (D) with touch
- **Product Size**: 50.00mm (W) × 86.00mm (H) × 9.10mm (D) without touch
- **Weight**: 
  - ES3C28P: 111g
  - ES3N28P: 100g

## Additional Resources

### Development Environment
- Arduino IDE support
- MicroPython support  
- ESP-IDF support (recommended)

### Example Projects
- LVGL GUI demos
- Audio playback/recording
- Image/video display
- Touch input handling
- SD card file operations
- WiFi/Bluetooth connectivity

## License

Please refer to the LICENSE file in the component directory.

## Technical Support

For technical support and questions:
- Email: Lcdwiki@163.com
- Email: goodtft@163.com
- Wiki: https://www.lcdwiki.com/

## Version

Current BSP Version: 0.1.0

---

**Note**: This BSP is designed for ESP-IDF 5.4 and later. For older ESP-IDF versions, please check compatibility or use the appropriate driver versions.