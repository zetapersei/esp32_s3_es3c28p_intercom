/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP32-S3-ES3C28P
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "esp_codec_dev.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "bsp/touch.h"

#include "driver/i2s_std.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lvgl_port.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_ESP32_S3_ES3C28P
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        0
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_LED            1
#define BSP_CAPS_BAT            1
/** @} */ // end of capabilities

/**************************************************************************************************
 *  Board pinout
 **************************************************************************************************/

/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL     (GPIO_NUM_15)
#define BSP_I2C_SDA     (GPIO_NUM_16)
/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_SCLK          (GPIO_NUM_5)
#define BSP_I2S_MCLK          (GPIO_NUM_4)
#define BSP_I2S_LCLK          (GPIO_NUM_7)
#define BSP_I2S_DOUT          (GPIO_NUM_8)
#define BSP_I2S_DSIN          (GPIO_NUM_6)
#define BSP_POWER_AMP_IO      (GPIO_NUM_1)
/** @} */ // end of audio

/** @defgroup g04_display Display and Touch
 *  @brief Display BSP API
 *  @{
 */
#define BSP_LCD_MOSI          (GPIO_NUM_11)
#define BSP_LCD_MISO          (GPIO_NUM_13)
#define BSP_LCD_PCLK          (GPIO_NUM_12)
#define BSP_LCD_CS            (GPIO_NUM_10)
#define BSP_LCD_DC            (GPIO_NUM_46)
#define BSP_LCD_RST           (GPIO_NUM_NC) // share reset pin with ESP32-S3 main control
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_45) // high level to turn on backlight, low level to turn off backlight
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_17)
#define BSP_LCD_TOUCH_RST     (GPIO_NUM_18)
/** @} */ // end of display

/** @defgroup g02_storage SD Card and SPIFFS
 *  @brief SPIFFS and SD card BSP API
 *  @{
 */
/* uSD card MMC */
#define BSP_SD_CMD          (GPIO_NUM_40)
#define BSP_SD_CLK          (GPIO_NUM_38)
#define BSP_SD_D0           (GPIO_NUM_39)
#define BSP_SD_D1           (GPIO_NUM_41)
#define BSP_SD_D2           (GPIO_NUM_48)
#define BSP_SD_D3           (GPIO_NUM_47)

/* uSD card SPI */
#define BSP_SD_SPI_MISO       (GPIO_NUM_39)
#define BSP_SD_SPI_CS         (GPIO_NUM_42)
#define BSP_SD_SPI_MOSI       (GPIO_NUM_44)
#define BSP_SD_SPI_CLK        (GPIO_NUM_43)
/** @} */ // end of storage


/** @defgroup g09_battery Battery
 *  @brief Battery BSP API
 *  @{
 */
#define BSP_BATTERY_VOLTAGE_CHANNEL     (ADC_CHANNEL_8)  // GPIO_NUM_9
#define BSP_BATTERY_VOLTAGE_DIV         (2)
/** @} */ // end of battery


#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g01_i2c
 *  @{
 */

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - LCD Touch controller
 **************************************************************************************************/
#define BSP_I2C_NUM     CONFIG_BSP_I2C_NUM

/**
 * @brief Init I2C driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_deinit(void);

/**
 * @brief Get I2C driver handle
 *
 * @return
 *      - I2C handle
 *
 */
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/** @} */ // end of i2c


/** \addtogroup g03_audio
 *  @{
 */

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output(playback) and input(recording) path
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Init audio
 *
 * @note There is no deinit audio function. Users can free audio resources by calling i2s_del_channel()
 * @warning The type of i2s_config param is depending on IDF version.
 * @param[in]  i2s_config I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 *      - ESP_ERR_NO_MEM        No memory for storing the channel information
 *      - ESP_ERR_INVALID_STATE This channel has not initialized or already started
 */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);

/**
 * @brief Initialize speaker codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

/**
 * @brief Initialize microphone codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

/**
 * @brief Get I2S TX (speaker) channel handle for direct low-latency access
 *
 * @note Bypasses esp_codec_dev buffering for maximum speed
 * @return I2S TX channel handle or NULL if not initialized
 */
i2s_chan_handle_t bsp_audio_get_i2s_tx_channel(void);

/**
 * @brief Get I2S RX (microphone) channel handle for direct low-latency access
 *
 * @note Bypasses esp_codec_dev buffering for maximum speed
 * @return I2S RX channel handle or NULL if not initialized
 */
i2s_chan_handle_t bsp_audio_get_i2s_rx_channel(void);

/** @} */ // end of audio

/** \addtogroup g02_storage
 *  @{
 */

/**************************************************************************************************
 *
 * SPIFFS
 *
 * After mounting the SPIFFS, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SPIFFS_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello World!\n");
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SPIFFS_MOUNT_POINT      CONFIG_BSP_SPIFFS_MOUNT_POINT
#define BSP_SD_MOUNT_POINT          CONFIG_BSP_SD_MOUNT_POINT
#define BSP_SDSPI_HOST              SPI2_HOST

/**
 * @brief SD card configuration structure
 */
typedef struct {
    const sdmmc_host_t *host;                       /*!< SD/MMC host config */
    const esp_vfs_fat_sdmmc_mount_config_t *mount;  /*!< Mount config */
    union {
        const sdmmc_slot_config_t *sdmmc;           /*!< SDMMC slot config */
        const sdspi_device_config_t *sdspi;         /*!< SDSPI device config */
    } slot;
} bsp_sdcard_cfg_t;

/**
 * @brief Mount SPIFFS to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_spiffs_mount(void);

/**
 * @brief Unmount SPIFFS
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_spiffs_unmount(void);

/**
 * @brief Mount SD card via SDMMC interface
 *
 * @param[in] cfg SD card configuration (can be NULL for default config)
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg);

/**
 * @brief Mount SD card via SPI interface
 *
 * @param[in] cfg SD card configuration (can be NULL for default config)
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg);

/**
 * @brief Mount SD card (using SDMMC with default config)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_sdcard_mount(void);

/**
 * @brief Unmount SD card
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_sdcard_unmount(void);

/** @} */ // end of storage

/** \addtogroup g04_display
 *  @{
 */

/**************************************************************************************************
 *
 * Display
 *
 **************************************************************************************************/

#define BSP_LCD_SPI_NUM          (SPI2_HOST)
#define BSP_LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)

/** @brief LCD size in pixels */
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * 50)
#define BSP_LCD_DRAW_BUFF_DOUBLE (0)

/**
 * @brief BSP display configuration structure for LVGL
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg; /*!< LVGL port configuration. */
    uint32_t buffer_size;          /*!< Buffer size for LVGL in bytes. */
    bool double_buffer;            /*!< Use double buffering for LVGL. */
    struct {
        unsigned int buff_dma : 1;    /*!< Use DMA memory for LVGL buffer. */
        unsigned int buff_spiram : 1; /*!< Use SPIRAM for LVGL buffer. */
    } flags;                        /*!< LVGL buffer flags. */
} bsp_display_cfg_t;

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
/**
 * @brief Start display with LVGL
 *
 * @return
 *      - lv_display_t* Pointer to LVGL display
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Start display with LVGL and custom configuration
 *
 * @param[in] cfg Display configuration
 * @return
 *      - lv_display_t* Pointer to LVGL display
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Get LVGL touch input device
 *
 * @return
 *      - lv_indev_t* Pointer to LVGL input device
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Rotate display
 *
 * @param disp LVGL display
 * @param rotation Rotation value
 */
void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation);

/**
 * @brief Lock LVGL for thread-safe operations
 *
 * @param timeout_ms Timeout in milliseconds
 * @return
 *      - true on success
 *      - false on timeout
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Unlock LVGL
 */
void bsp_display_unlock(void);
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/** @} */ // end of display

/** \addtogroup g04_display
 *  @{
 */

/**************************************************************************************************
 *
 * Touch
 *
 **************************************************************************************************/

/**
 * @brief Start touch
 *
 * @return
 *      - esp_lcd_touch_handle_t* Pointer to touch handle
 */
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);

/** @} */ // end of display

#ifdef __cplusplus
}
#endif
