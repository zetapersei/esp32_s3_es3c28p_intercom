#include <math.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "bsp/esp32_s3_es3c28p.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"

// ESP-SR AFE для эхоподавления
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_agc.h"  // ESP-SR AGC API
#include "esp_aec.h"  // ESP-SR Low-level AEC API (lightweight)
#include "esp_ns.h"   // ESP-SR Noise Suppression API

// WebRTC и Speex удалены - используем только ESP-SR AEC/NS/AGC

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
// Шрифт с поддержкой кириллицы
LV_FONT_DECLARE(font_montserrat_14_cyrillic);
#endif

static const char *TAG = "es3c28p_test";

static esp_codec_dev_handle_t s_spk_dev = NULL;
static esp_codec_dev_handle_t s_mic_dev = NULL;
static volatile bool s_beep_request = false;
static volatile bool s_record_request = false;
static volatile int s_mic_level = 0;

// Буфер для записи (500мс, 16kHz, моно)
#define RECORD_DURATION_MS  500
#define SAMPLE_RATE         16000
#define RECORD_SAMPLES      ((SAMPLE_RATE * RECORD_DURATION_MS) / 1000)
static int16_t *s_record_buffer = NULL;
static volatile bool s_is_recording = false;
static volatile bool s_is_playing = false;

// Wi-Fi
#define MAX_SCAN_RESULTS 15
#define NVS_WIFI_NAMESPACE "wifi_cfg"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASSWORD "password"
#define NVS_KEY_TARGET_HOST "target_host"
#define NVS_KEY_ROOM_NAME "room_name"
// Audio settings NVS keys
#define NVS_KEY_AECM_ENABLED "aecm_en"
#define NVS_KEY_AECM_DELAY "aecm_delay"
#define NVS_KEY_AECM_MODE "aecm_mode"
#define NVS_KEY_AGC_ENABLED "agc_en"
#define NVS_KEY_AGC_GAIN "agc_gain"
#define NVS_KEY_NOISE_GATE "noise_gate"
#define NVS_KEY_USE_PCM "use_pcm"
static wifi_ap_record_t s_ap_records[MAX_SCAN_RESULTS];
static char s_target_host[32] = {0};  // hostname получателя (напр. "esp-AABBCC")
static char s_room_name[32] = {0};   // Название комнаты (напр. "Гостиная")
static char s_target_ip[16] = {0};   // IP адрес получателя (если известен из mDNS discovery)
static char s_my_hostname[32] = {0}; // свой hostname
static uint16_t s_ap_count = 0;
static volatile bool s_wifi_scanning = false;
static volatile bool s_wifi_scan_done = false;
static volatile bool s_wifi_connected = false;
static char s_wifi_ssid[33] = {0};
static char s_wifi_password[65] = {0};  // Пароль для автоматического переподключения
static int s_wifi_retry_count = 0;      // Счетчик попыток переподключения
static int8_t s_wifi_rssi = 0;  // RSSI текущей WiFi сети
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// UDP Audio Streaming
#define UDP_AUDIO_PORT 5000
#define AUDIO_SAMPLE_RATE 16000  // 16kHz - CPU не справляется с 32kHz + AEC (load 300%)
#define AUDIO_CHUNK_SIZE 512    // 512 сэмплов = 32ms при 16kHz (меньше накладных расходов)
#define JITTER_BUFFER_PACKETS 4  // Буферизуем 4 пакета перед стартом (~128ms)
#define JITTER_BUFFER_MIN 2     // Минимум пакетов для продолжения воспроизведения
#define PRE_EMPHASIS_COEFF 0.95f  // Коэффициент pre-emphasis

// ===== Настройки аудио (runtime-конфигурируемые) =====
static bool s_aecm_enabled = false;      // AECM отключено
static int s_aecm_delay_ms = 35;         // Задержка звуковой карты (мс)
static int s_aecm_mode = 2;              // Режим эхоподавления (0-4)
static bool s_agc_enabled = true;        // AGC включено для нормализации громкости
static int s_agc_gain_db = 40;           // Усиление AGC (dB) - увеличено для громкости
static int s_noise_gate_threshold = 500; // Noise gate - умеренный порог
static bool s_use_pcm = false;           // true = PCM (качество), false = ADPCM (экономия трафика)

// ===== Флаги для обработки звука =====
#define USE_ESP_AEC 1      // 1 = ESP-SR aec_pro_create (легкий) - синхронный, оптимизирован для S3
#define USE_AGC 1          // 1 = ESP-SR AGC - автоматическая регулировка громкости
#define USE_ESP_NS 1       // 1 = ESP-SR NS (Mild mode) - шумоподавление
#define USE_VAD 0          // 0 = отключено, 1 = ESP-SR VAD
#define USE_VOX 0          // 0 = FULL-DUPLEX (всегда!), 1 = half-duplex (ЗАПРЕЩЕНО)
#define USE_AFE_SYNC 1     // 1 = синхронный AEC в TX task, 0 = async task (deprecated)

static int s_udp_socket = -1;
static volatile bool s_streaming_tx = false;  // Передача звука
static volatile bool s_streaming_rx = false;  // Приём звука
static volatile bool s_rx_active = false;     // Есть входящий сигнал
static volatile uint32_t s_rx_last_packet_time = 0;  // Время последнего пакета
static TaskHandle_t s_tx_task_handle = NULL;
static TaskHandle_t s_rx_task_handle = NULL;
static TaskHandle_t s_audio_task_handle = NULL;  // Для приостановки audio_task во время TX

// Статические буферы TX для быстрого старта (выделяются один раз)
static int16_t *s_tx_buf = NULL;
static int16_t *s_tx_mono_buf = NULL;
static uint8_t *s_tx_adpcm_buf = NULL;

// Статический стек для TX task в PSRAM (32KB)
#define TX_TASK_STACK_SIZE 8192  // В словах = 32KB
static StackType_t *s_tx_task_stack = NULL;
static StaticTask_t s_tx_task_tcb;

// mDNS Device Discovery
#define MAX_DISCOVERED_DEVICES 10
#define MDNS_SERVICE_TYPE "_esp-audio"
#define MDNS_SERVICE_PROTO "_udp"
typedef struct {
    char hostname[32];
    char ip[16];
    bool valid;
} discovered_device_t;
static discovered_device_t s_discovered_devices[MAX_DISCOVERED_DEVICES];
static int s_discovered_count = 0;
static volatile bool s_mdns_scanning = false;
static volatile bool s_mdns_scan_done = false;
static lv_obj_t *s_device_list = NULL;
static mdns_search_once_t *s_mdns_browse_handle = NULL;  // mDNS browse handle для пассивного обнаружения

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_obj_t *s_label = NULL;
static lv_obj_t *s_btn_speaker = NULL;
static lv_obj_t *s_btn_record = NULL;
static lv_obj_t *s_btn_wifi = NULL;
static lv_obj_t *s_btn_tx = NULL;       // Кнопка TX с названием комнаты
static lv_obj_t *s_label_tx = NULL;     // Метка на кнопке TX
static lv_obj_t *s_label_rx = NULL;     // Индикатор входящего вызова
static lv_obj_t *s_wifi_list = NULL;
static lv_obj_t *s_wifi_panel = NULL;
static lv_obj_t *s_kb = NULL;
static lv_obj_t *s_pwd_ta = NULL;
static lv_obj_t *s_btn_ip = NULL;
static lv_obj_t *s_ip_panel = NULL;
static lv_obj_t *s_ip_ta = NULL;
static lv_indev_t *s_indev = NULL;
static char s_selected_ssid[33] = {0};
static esp_lcd_touch_handle_t s_touch_handle = NULL;  // For debug

// Audio settings UI elements
static lv_obj_t *s_btn_audio = NULL;    // Кнопка настроек аудио
static lv_obj_t *s_audio_panel = NULL;  // Панель настроек аудио
static lv_obj_t *s_slider_aecm_delay = NULL;
static lv_obj_t *s_slider_agc_gain = NULL;
static lv_obj_t *s_slider_noise_gate = NULL;
static lv_obj_t *s_sw_aecm = NULL;      // Переключатель AECM
static lv_obj_t *s_sw_agc = NULL;       // Переключатель AGC
static lv_obj_t *s_sw_pcm = NULL;       // Переключатель PCM/ADPCM
static lv_obj_t *s_label_aecm_delay = NULL;
static lv_obj_t *s_label_agc_gain = NULL;
static lv_obj_t *s_label_noise_gate = NULL;
static lv_obj_t *s_btn_calibrate = NULL;    // Кнопка калибровки
static lv_obj_t *s_label_calibrate = NULL;  // Метка калибровки

// Общая панель настроек
static lv_obj_t *s_settings_panel = NULL;   // Главная панель настроек
static lv_obj_t *s_btn_settings = NULL;     // Кнопка открытия настроек

// Сброс статических счётчиков логов калибровки
static volatile bool s_calib_reset_log_counters = false;

// Дата и время сборки (автоматически от компилятора)
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// ===== Физическая кнопка Cherry MX (PTT) =====
#define PTT_BUTTON_GPIO     GPIO_NUM_2      // GPIO2 - свободный пин для кнопки
static volatile bool s_ptt_pressed = false;

// Touch calibration debug
static lv_obj_t *s_touch_point = NULL;      // Точка для отображения касания
static lv_obj_t *s_touch_label = NULL;      // Метка с координатами
static bool s_touch_debug_enabled = false;  // Режим отладки тача

// Русская клавиатура - нижний регистр
static const char * const kb_map_ru_lc[] = {
    "1#", "й", "ц", "у", "к", "е", "н", "г", "ш", "щ", "з", "х", LV_SYMBOL_BACKSPACE, "\n",
    "АБВ", "ф", "ы", "в", "а", "п", "р", "о", "л", "д", "ж", "э", "\n",
    "en", "я", "ч", "с", "м", "и", "т", "ь", "б", "ю", ".", "ъ", "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_OK, ""
};

// Русская клавиатура - верхний регистр
static const char * const kb_map_ru_uc[] = {
    "1#", "Й", "Ц", "У", "К", "Е", "Н", "Г", "Ш", "Щ", "З", "Х", LV_SYMBOL_BACKSPACE, "\n",
    "абв", "Ф", "Ы", "В", "А", "П", "Р", "О", "Л", "Д", "Ж", "Э", "\n",
    "en", "Я", "Ч", "С", "М", "И", "Т", "Ь", "Б", "Ю", ".", "Ъ", "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_OK, ""
};

// Английская клавиатура - нижний регистр (с кнопкой ru)
static const char * const kb_map_en_lc[] = {
    "1#", "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", LV_SYMBOL_BACKSPACE, "\n",
    "ABC", "a", "s", "d", "f", "g", "h", "j", "k", "l", "\n",
    "ru", "z", "x", "c", "v", "b", "n", "m", ".", ",", "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_OK, ""
};

// Английская клавиатура - верхний регистр (с кнопкой ru)
static const char * const kb_map_en_uc[] = {
    "1#", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", LV_SYMBOL_BACKSPACE, "\n",
    "abc", "A", "S", "D", "F", "G", "H", "J", "K", "L", "\n",
    "ru", "Z", "X", "C", "V", "B", "N", "M", ".", ",", "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_OK, ""
};

// Контрольная карта для русской клавиатуры
static const lv_buttonmatrix_ctrl_t kb_ctrl_ru_map[] = {
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, LV_BUTTONMATRIX_CTRL_CHECKED | 5,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 2, LV_BUTTONMATRIX_CTRL_CHECKED | 2, 6, LV_BUTTONMATRIX_CTRL_CHECKED | 2, LV_KEYBOARD_CTRL_BUTTON_FLAGS | 2
};

// Контрольная карта для английской клавиатуры
static const lv_buttonmatrix_ctrl_t kb_ctrl_en_map[] = {
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, LV_BUTTONMATRIX_CTRL_CHECKED | 5,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 6, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    LV_KEYBOARD_CTRL_BUTTON_FLAGS | 2, LV_BUTTONMATRIX_CTRL_CHECKED | 2, 6, LV_BUTTONMATRIX_CTRL_CHECKED | 2, LV_KEYBOARD_CTRL_BUTTON_FLAGS | 2
};

// Флаг текущей раскладки
static bool s_kb_russian = false;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declarations
static esp_err_t save_audio_settings(void);

static void audio_beep_play(void)
{
    if (!s_spk_dev) {
        ESP_LOGE(TAG, "Speaker device is NULL!");
        return;
    }

    const int sample_rate = 16000;  // Совпадает с AUDIO_SAMPLE_RATE для ESP-SR AFE
    const int freq = 1000;  // 1000 Hz - короткий тон
    const int duration_ms = 100;  // 100ms - короткий писк
    const int samples = (sample_rate * duration_ms) / 1000;
    const float amplitude = 0.3f;  // 30% громкости

    // Моно буфер (channel=1)
    int16_t *buffer = (int16_t *)heap_caps_malloc(samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate beep buffer");
        return;
    }

    for (int i = 0; i < samples; ++i) {
        float t = (float)i / (float)sample_rate;
        float s = sinf(2.0f * (float)M_PI * (float)freq * t);
        buffer[i] = (int16_t)(s * amplitude * 32767.0f);
    }

    ESP_LOGI(TAG, "Playing beep: freq=%dHz, dur=%dms, samples=%d, buf_size=%d bytes",
             freq, duration_ms, samples, samples * (int)sizeof(int16_t));
    
    int ret = esp_codec_dev_write(s_spk_dev, buffer, samples * sizeof(int16_t));
    ESP_LOGI(TAG, "Beep write result: %d (0=OK)", ret);

    free(buffer);
}

// ===== Автокалибровка задержки AECM =====
// Работает поверх TX/RX задач без их остановки
// TX задача: пропускает отправку в сеть, копирует микрофон в калибровочный буфер
// RX задача: пропускает воспроизведение сетевых данных, воспроизводит калибровочные щелчки

static volatile bool s_calibration_running = false;
static volatile bool s_calibration_active = false;  // Флаг для TX/RX задач
static volatile int s_calibrated_delay_ms = -1;

// Буферы для обмена данными с TX/RX задачами во время калибровки
static int16_t *s_calib_spk_buf = NULL;      // Буфер щелчков для воспроизведения
static volatile int s_calib_spk_samples = 0;  // Сколько семплов в буфере
static volatile int s_calib_spk_pos = 0;      // Позиция воспроизведения
static volatile bool s_calib_click_playing = false;  // Флаг: щелчок воспроизводится
static volatile bool s_calib_click_done = false;     // Флаг: щелчок воспроизведён полностью

static int16_t *s_calib_mic_buf = NULL;       // Буфер записи микрофона
static volatile int s_calib_mic_max = 0;      // Размер буфера
static volatile int s_calib_mic_pos = 0;      // Позиция записи
static SemaphoreHandle_t s_calib_mic_ready = NULL;  // Сигнал что буфер заполнен

static int audio_calibrate_delay(void)
{
    if (!s_mic_dev || !s_spk_dev) {
        ESP_LOGE(TAG, "Audio devices not initialized for calibration!");
        return -1;
    }
    
    // Проверяем, что калибровка не запущена
    if (s_calibration_running) {
        ESP_LOGW(TAG, "Calibration already running!");
        return -1;
    }
    
    s_calibration_running = true;
    s_calib_reset_log_counters = true;  // Сброс статических счётчиков логов в RX/TX задачах
    s_calib_click_playing = false;
    s_calib_click_done = false;
    ESP_LOGI(TAG, "=== Starting AECM delay calibration ===");
    
    const int sample_rate = AUDIO_SAMPLE_RATE;  // 16kHz
    const int click_samples = 64;  // Щелчок ~4мс - увеличен для лучшей слышимости
    const int silence_samples = 256;  // 16мс тишины перед щелчком
    const int num_clicks = 3;
    const int capture_samples = 1600;  // 100мс записи после щелчка
    
    // Проверяем состояние стриминга
    // Для калибровки через буферы нужны ОБА потока (TX для микрофона, RX для динамика)
    // Если активен хотя бы один поток - прямой режим использовать НЕЛЬЗЯ (конфликт I2S)
    bool tx_active = s_streaming_tx;
    bool rx_active = s_streaming_rx;
    
    if (tx_active && rx_active) {
        // Оба потока активны - используем режим калибровки через буферы
        ESP_LOGI(TAG, "Calibration in streaming mode (TX=%d, RX=%d)", tx_active, rx_active);
        
        // Выделяем буферы для обмена с TX/RX задачами
        s_calib_spk_buf = (int16_t *)heap_caps_malloc((click_samples + silence_samples) * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        s_calib_mic_buf = (int16_t *)heap_caps_malloc(capture_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        s_calib_mic_ready = xSemaphoreCreateBinary();
        
        if (!s_calib_spk_buf || !s_calib_mic_buf || !s_calib_mic_ready) {
            ESP_LOGE(TAG, "Failed to allocate calibration buffers");
            if (s_calib_spk_buf) { free(s_calib_spk_buf); s_calib_spk_buf = NULL; }
            if (s_calib_mic_buf) { free(s_calib_mic_buf); s_calib_mic_buf = NULL; }
            if (s_calib_mic_ready) { vSemaphoreDelete(s_calib_mic_ready); s_calib_mic_ready = NULL; }
            s_calibration_running = false;
            return -1;
        }
        
        int total_delay_samples = 0;
        int valid_measurements = 0;
        
        for (int click = 0; click < num_clicks; click++) {
            // Готовим буфер щелчка: тишина + импульс МАКСИМАЛЬНОЙ громкости
            memset(s_calib_spk_buf, 0, (click_samples + silence_samples) * sizeof(int16_t));
            for (int i = 0; i < click_samples; i++) {
                // Прямоугольный импульс максимальной амплитуды (±32000)
                s_calib_spk_buf[silence_samples + i] = (i < click_samples/2) ? 32000 : -32000;
            }
            s_calib_spk_samples = silence_samples + click_samples;
            s_calib_spk_pos = 0;
            
            // Готовим буфер микрофона (НЕ начинаем запись пока щелчок не воспроизведён)
            memset(s_calib_mic_buf, 0, capture_samples * sizeof(int16_t));
            s_calib_mic_max = capture_samples;
            s_calib_mic_pos = 0;
            
            // Сбрасываем флаги синхронизации
            s_calib_click_playing = true;   // Начинаем воспроизведение
            s_calib_click_done = false;     // Ещё не воспроизведено
            
            ESP_LOGI(TAG, "Click %d: prepared %d samples (silence=%d, click=%d)", 
                     click + 1, s_calib_spk_samples, silence_samples, click_samples);
            
            // Активируем режим калибровки - RX начнёт воспроизводить, TX будет ждать click_done
            s_calibration_active = true;
            
            // Ждём заполнения буфера микрофона (макс 500мс)
            if (xSemaphoreTake(s_calib_mic_ready, pdMS_TO_TICKS(500)) == pdTRUE) {
                // Деактивируем режим калибровки
                s_calibration_active = false;
                
                // Поиск пика в записи
                // Теперь запись начинается ПОСЛЕ воспроизведения щелчка,
                // поэтому пик должен быть в первых ~50мс (800 сэмплов при 16кГц)
                // Считаем фон по концу записи (там точно нет щелчка)
                int32_t background = 0;
                int bg_start = s_calib_mic_pos > 512 ? s_calib_mic_pos - 256 : 0;
                int bg_count = 0;
                for (int i = bg_start; i < s_calib_mic_pos && bg_count < 256; i++, bg_count++) {
                    background += abs(s_calib_mic_buf[i]);
                }
                if (bg_count > 0) background /= bg_count;
                int threshold = background * 3 + 500;  // Понизили порог с 800 до 500
                
                // Ищем пик с самого начала записи
                int peak_sample = -1;
                for (int i = 0; i < s_calib_mic_pos; i++) {
                    if (abs(s_calib_mic_buf[i]) > threshold) {
                        peak_sample = i;
                        break;
                    }
                }
                
                // Логируем уровень сигнала в первых 100 сэмплах для диагностики
                int32_t max_signal = 0;
                for (int i = 0; i < 100 && i < s_calib_mic_pos; i++) {
                    if (abs(s_calib_mic_buf[i]) > max_signal) {
                        max_signal = abs(s_calib_mic_buf[i]);
                    }
                }
                ESP_LOGI(TAG, "Click %d: recorded=%d samples, bg=%ld, threshold=%d, max_first_100=%ld",
                         click + 1, s_calib_mic_pos, (long)background, threshold, (long)max_signal);
                
                if (peak_sample >= 0) {
                    // Задержка = позиция пика (в сэмплах от начала записи)
                    // Это время от конца воспроизведения щелчка до его появления в микрофоне
                    int delay_samples = peak_sample;
                    int delay_ms = (delay_samples * 1000) / sample_rate;
                    
                    ESP_LOGI(TAG, "Click %d: peak at %d samples = %d ms",
                             click + 1, peak_sample, delay_ms);
                    
                    total_delay_samples += delay_samples;
                    valid_measurements++;
                } else {
                    ESP_LOGW(TAG, "Click %d: no peak found", click + 1);
                }
            } else {
                ESP_LOGW(TAG, "Click %d: timeout waiting for mic data", click + 1);
                s_calibration_active = false;
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Освобождаем ресурсы
        free(s_calib_spk_buf); s_calib_spk_buf = NULL;
        free(s_calib_mic_buf); s_calib_mic_buf = NULL;
        vSemaphoreDelete(s_calib_mic_ready); s_calib_mic_ready = NULL;
        s_calibration_active = false;
        s_calib_click_playing = false;
        s_calib_click_done = false;
        
        if (valid_measurements > 0) {
            int avg_delay_ms = ((total_delay_samples / valid_measurements) * 1000) / sample_rate + 10;
            if (avg_delay_ms < 10) avg_delay_ms = 10;
            if (avg_delay_ms > 200) avg_delay_ms = 200;
            
            ESP_LOGI(TAG, "=== Calibration complete (streaming): %d ms ===", avg_delay_ms);
            s_calibrated_delay_ms = avg_delay_ms;
            s_calibration_running = false;
            return avg_delay_ms;
        } else {
            ESP_LOGW(TAG, "=== Calibration failed (streaming): no valid measurements ===");
            s_calibration_running = false;
            return -1;
        }
    } else if (rx_active && !tx_active) {
        // ===== Гибридный режим: RX активен, TX нет =====
        // Динамик занят RX задачей -> щелчки через буфер s_calib_spk_buf
        // Микрофон свободен -> читаем напрямую через esp_codec_dev_read
        ESP_LOGI(TAG, "Calibration in hybrid mode (RX only): clicks via buffer, mic direct");
        
        s_calib_spk_buf = (int16_t *)heap_caps_malloc((click_samples + silence_samples) * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        int16_t *mic_buf = (int16_t *)heap_caps_malloc(capture_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        
        if (!s_calib_spk_buf || !mic_buf) {
            ESP_LOGE(TAG, "Failed to allocate calibration buffers");
            if (s_calib_spk_buf) { free(s_calib_spk_buf); s_calib_spk_buf = NULL; }
            if (mic_buf) free(mic_buf);
            s_calibration_running = false;
            return -1;
        }
        
        int total_delay_samples = 0;
        int valid_measurements = 0;
        
        // Прогреваем микрофон
        esp_codec_dev_read(s_mic_dev, mic_buf, capture_samples * 2 * sizeof(int16_t));
        vTaskDelay(pdMS_TO_TICKS(30));
        
        for (int click = 0; click < num_clicks && s_calibration_running; click++) {
            // Готовим буфер щелчка для RX задачи
            memset(s_calib_spk_buf, 0, (click_samples + silence_samples) * sizeof(int16_t));
            for (int i = 0; i < click_samples; i++) {
                s_calib_spk_buf[silence_samples + i] = (i < click_samples/2) ? 32000 : -32000;
            }
            s_calib_spk_samples = silence_samples + click_samples;
            s_calib_spk_pos = 0;
            
            // Инициализируем флаги синхронизации
            s_calib_click_playing = true;
            s_calib_click_done = false;
            
            // Активируем воспроизведение щелчка через RX
            s_calibration_active = true;
            
            // Ждём пока RX воспроизведёт щелчок (ждём флаг s_calib_click_done)
            int wait_count = 0;
            while (!s_calib_click_done && wait_count < 50) {  // Макс 250мс
                vTaskDelay(pdMS_TO_TICKS(5));
                wait_count++;
            }
            
            // Читаем микрофон напрямую
            memset(mic_buf, 0, capture_samples * 2 * sizeof(int16_t));
            esp_codec_dev_read(s_mic_dev, mic_buf, capture_samples * 2 * sizeof(int16_t));
            
            s_calibration_active = false;
            
            // Анализируем (стерео буфер)
            int32_t background = 0;
            for (int i = 0; i < 256; i++) {
                background += abs(mic_buf[i * 2]);
            }
            background /= 256;
            int threshold = background * 3 + 800;
            
            int peak_sample = -1;
            for (int i = 0; i < capture_samples; i++) {
                if (abs(mic_buf[i * 2]) > threshold) {
                    peak_sample = i;
                    break;
                }
            }
            
            if (peak_sample >= 0) {
                int delay_ms = (peak_sample * 1000) / sample_rate;
                ESP_LOGI(TAG, "Click %d: peak at %d, delay=%d ms", click + 1, peak_sample, delay_ms);
                total_delay_samples += peak_sample;
                valid_measurements++;
            } else {
                ESP_LOGW(TAG, "Click %d: no peak", click + 1);
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        free(s_calib_spk_buf); s_calib_spk_buf = NULL;
        free(mic_buf);
        s_calibration_active = false;
        s_calib_click_playing = false;
        s_calib_click_done = false;
        
        if (valid_measurements > 0) {
            int avg_delay_ms = ((total_delay_samples / valid_measurements) * 1000) / sample_rate + 10;
            if (avg_delay_ms < 10) avg_delay_ms = 10;
            if (avg_delay_ms > 200) avg_delay_ms = 200;
            
            ESP_LOGI(TAG, "=== Calibration complete (hybrid RX): %d ms ===", avg_delay_ms);
            s_calibrated_delay_ms = avg_delay_ms;
            s_calibration_running = false;
            return avg_delay_ms;
        } else {
            ESP_LOGW(TAG, "=== Calibration failed (hybrid RX) ===");
            s_calibration_running = false;
            return -1;
        }
    } else if (tx_active && !rx_active) {
        // ===== Гибридный режим: TX активен, RX нет =====
        // Микрофон занят TX задачей -> данные через буфер s_calib_mic_buf
        // Динамик свободен -> играем напрямую через esp_codec_dev_write
        ESP_LOGI(TAG, "Calibration in hybrid mode (TX only): clicks direct, mic via buffer");
        
        s_calib_mic_buf = (int16_t *)heap_caps_malloc(capture_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        s_calib_mic_ready = xSemaphoreCreateBinary();
        int16_t *click_buf = (int16_t *)heap_caps_malloc(click_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
        int16_t *silence_buf = (int16_t *)heap_caps_calloc(silence_samples, sizeof(int16_t), MALLOC_CAP_DEFAULT);
        
        if (!s_calib_mic_buf || !s_calib_mic_ready || !click_buf || !silence_buf) {
            ESP_LOGE(TAG, "Failed to allocate calibration buffers");
            if (s_calib_mic_buf) { free(s_calib_mic_buf); s_calib_mic_buf = NULL; }
            if (s_calib_mic_ready) { vSemaphoreDelete(s_calib_mic_ready); s_calib_mic_ready = NULL; }
            if (click_buf) free(click_buf);
            if (silence_buf) free(silence_buf);
            s_calibration_running = false;
            return -1;
        }
        
        // Готовим буфер щелчка
        for (int i = 0; i < click_samples; i++) {
            click_buf[i] = (i < click_samples/2) ? 32000 : -32000;
        }
        
        int total_delay_samples = 0;
        int valid_measurements = 0;
        
        for (int click = 0; click < num_clicks && s_calibration_running; click++) {
            // Готовим буфер микрофона для TX задачи
            memset(s_calib_mic_buf, 0, capture_samples * sizeof(int16_t));
            s_calib_mic_max = capture_samples;
            s_calib_mic_pos = 0;
            
            // В этом режиме щелчок играется напрямую, поэтому сразу разрешаем TX записывать
            s_calib_click_playing = false;
            s_calib_click_done = true;
            
            // Активируем запись микрофона через TX
            s_calibration_active = true;
            
            // Играем тишину + щелчок напрямую
            esp_codec_dev_write(s_spk_dev, silence_buf, silence_samples * sizeof(int16_t));
            esp_codec_dev_write(s_spk_dev, click_buf, click_samples * sizeof(int16_t));
            
            // Ждём заполнения буфера микрофона (макс 500мс)
            if (xSemaphoreTake(s_calib_mic_ready, pdMS_TO_TICKS(500)) == pdTRUE) {
                s_calibration_active = false;
                
                // Анализируем
                int32_t background = 0;
                for (int i = 0; i < 256 && i < s_calib_mic_pos; i++) {
                    background += abs(s_calib_mic_buf[i]);
                }
                background /= 256;
                int threshold = background * 3 + 800;
                
                int peak_sample = -1;
                for (int i = 0; i < s_calib_mic_pos; i++) {
                    if (abs(s_calib_mic_buf[i]) > threshold) {
                        peak_sample = i;
                        break;
                    }
                }
                
                if (peak_sample >= 0) {
                    int delay_ms = (peak_sample * 1000) / sample_rate;
                    ESP_LOGI(TAG, "Click %d: peak at %d, delay=%d ms", click + 1, peak_sample, delay_ms);
                    total_delay_samples += peak_sample;
                    valid_measurements++;
                } else {
                    ESP_LOGW(TAG, "Click %d: no peak", click + 1);
                }
            } else {
                ESP_LOGW(TAG, "Click %d: timeout waiting for mic data", click + 1);
                s_calibration_active = false;
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        free(s_calib_mic_buf); s_calib_mic_buf = NULL;
        vSemaphoreDelete(s_calib_mic_ready); s_calib_mic_ready = NULL;
        free(click_buf);
        free(silence_buf);
        s_calibration_active = false;
        s_calib_click_playing = false;
        s_calib_click_done = false;
        
        if (valid_measurements > 0) {
            int avg_delay_ms = ((total_delay_samples / valid_measurements) * 1000) / sample_rate + 10;
            if (avg_delay_ms < 10) avg_delay_ms = 10;
            if (avg_delay_ms > 200) avg_delay_ms = 200;
            
            ESP_LOGI(TAG, "=== Calibration complete (hybrid TX): %d ms ===", avg_delay_ms);
            s_calibrated_delay_ms = avg_delay_ms;
            s_calibration_running = false;
            return avg_delay_ms;
        } else {
            ESP_LOGW(TAG, "=== Calibration failed (hybrid TX) ===");
            s_calibration_running = false;
            return -1;
        }
    }
    
    // ===== Обычный режим (без стриминга) - прямой доступ к аудио =====
    // Используется только когда TX=0 и RX=0
    ESP_LOGI(TAG, "Calibration in direct mode (no streaming)");
    
    // Общий таймаут для всей калибровки - 5 секунд
    TickType_t calib_start_time = xTaskGetTickCount();
    const TickType_t calib_timeout = pdMS_TO_TICKS(5000);
    
    // Буфер для одного щелчка (моно)
    int16_t click_buf[64];  // click_samples = 64
    for (int i = 0; i < click_samples; i++) {
        click_buf[i] = (i < click_samples/2) ? 32000 : -32000;
    }
    
    // Буфер тишины
    int16_t *silence_buf = (int16_t *)heap_caps_calloc(silence_samples, sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!silence_buf) {
        ESP_LOGE(TAG, "Failed to allocate silence buffer");
        s_calibration_running = false;
        return -1;
    }
    
    // Буфер для записи микрофона (стерео)
    int16_t *mic_buf = (int16_t *)heap_caps_malloc(capture_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!mic_buf) {
        ESP_LOGE(TAG, "Failed to allocate mic capture buffer");
        free(silence_buf);
        s_calibration_running = false;
        return -1;
    }
    
    int total_delay_samples = 0;
    int valid_measurements = 0;
    
    // Прогреваем I2S (с проверкой таймаута)
    if ((xTaskGetTickCount() - calib_start_time) < calib_timeout) {
        esp_codec_dev_read(s_mic_dev, mic_buf, capture_samples * 2 * sizeof(int16_t));
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    
    for (int click = 0; click < num_clicks && s_calibration_running; click++) {
        // Проверяем общий таймаут
        if ((xTaskGetTickCount() - calib_start_time) > calib_timeout) {
            ESP_LOGW(TAG, "Calibration timeout!");
            break;
        }
        
        memset(mic_buf, 0, capture_samples * 2 * sizeof(int16_t));
        
        // Тишина + щелчок
        esp_codec_dev_write(s_spk_dev, silence_buf, silence_samples * sizeof(int16_t));
        esp_codec_dev_write(s_spk_dev, click_buf, click_samples * sizeof(int16_t));
        
        // Запись
        esp_codec_dev_read(s_mic_dev, mic_buf, capture_samples * 2 * sizeof(int16_t));
        
        // Фоновый уровень
        int32_t background = 0;
        for (int i = 0; i < 256; i++) {
            background += abs(mic_buf[i * 2]);
        }
        background /= 256;
        int threshold = background * 3 + 800;
        
        // Ищем пик
        int peak_sample = -1;
        for (int i = 0; i < capture_samples; i++) {
            if (abs(mic_buf[i * 2]) > threshold) {
                peak_sample = i;
                break;
            }
        }
        
        if (peak_sample >= 0) {
            int delay_ms = (peak_sample * 1000) / sample_rate;
            ESP_LOGI(TAG, "Click %d: peak at %d, delay=%d ms", click + 1, peak_sample, delay_ms);
            total_delay_samples += peak_sample;
            valid_measurements++;
        } else {
            ESP_LOGW(TAG, "Click %d: no peak", click + 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    free(silence_buf);
    free(mic_buf);
    
    if (valid_measurements > 0) {
        int avg_delay_ms = ((total_delay_samples / valid_measurements) * 1000) / sample_rate + 10;
        if (avg_delay_ms < 10) avg_delay_ms = 10;
        if (avg_delay_ms > 200) avg_delay_ms = 200;
        
        ESP_LOGI(TAG, "=== Calibration complete (direct): %d ms ===", avg_delay_ms);
        s_calibrated_delay_ms = avg_delay_ms;
        s_calibration_running = false;
        return avg_delay_ms;
    } else {
        ESP_LOGW(TAG, "=== Calibration failed (direct) ===");
        s_calibration_running = false;
        return -1;
    }
}

// ===== Физическая кнопка PTT (Push-To-Talk) Cherry MX =====
// Forward declarations для функций управления передачей
static void stream_tx_start(void);
static void stream_tx_stop(void);

// Задача опроса состояния PTT кнопки (polling вместо ISR для надёжности)
static void ptt_button_task(void *arg)
{
    bool last_stable_state = false;
    bool current_reading = false;
    int debounce_count = 0;
    const int DEBOUNCE_THRESHOLD = 3;  // 3 * 20ms = 60ms стабильного состояния
    
    ESP_LOGI(TAG, "PTT button task started on GPIO%d (polling mode)", PTT_BUTTON_GPIO);
    
    while (1) {
        // Читаем GPIO напрямую (0 = нажата, 1 = отпущена)
        bool gpio_pressed = (gpio_get_level(PTT_BUTTON_GPIO) == 0);
        
        // Программный дебаунс
        if (gpio_pressed == current_reading) {
            debounce_count++;
        } else {
            current_reading = gpio_pressed;
            debounce_count = 0;
        }
        
        // Состояние считается стабильным после DEBOUNCE_THRESHOLD последовательных чтений
        if (debounce_count >= DEBOUNCE_THRESHOLD && current_reading != last_stable_state) {
            last_stable_state = current_reading;
            s_ptt_pressed = current_reading;  // Обновляем глобальный флаг
            
            if (current_reading) {
                // Кнопка нажата - начинаем передачу
                ESP_LOGI(TAG, "PTT pressed - starting TX");
                if (!s_streaming_tx) {
                    stream_tx_start();
                }
                // Обновляем UI кнопку - красный цвет при передаче
                if (lvgl_port_lock(100)) {
                    if (s_btn_tx) {
                        lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(200, 50, 50), 0);
                    }
                    lvgl_port_unlock();
                }
            } else {
                // Кнопка отпущена - останавливаем передачу
                ESP_LOGI(TAG, "PTT released - stopping TX");
                if (s_streaming_tx) {
                    stream_tx_stop();
                }
                // Возвращаем цвет UI кнопки
                if (lvgl_port_lock(100)) {
                    if (s_btn_tx) {
                        if (s_rx_active) {
                            lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(255, 150, 50), 0);  // Оранжевый
                        } else {
                            lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(50, 150, 50), 0);   // Зелёный
                        }
                    }
                    lvgl_port_unlock();
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));  // Опрос каждые 20мс
    }
}

// Инициализация кнопки PTT
static void ptt_button_init(void)
{
    // Конфигурация GPIO для кнопки (без прерываний - используем polling)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // Без прерываний - polling надёжнее для мех. кнопок
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PTT_BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,    // Внутренняя подтяжка к 3.3В
    };
    gpio_config(&io_conf);
    
    // Запускаем задачу обработки кнопки (8KB стека - stream_tx функции требуют много места)
    xTaskCreate(ptt_button_task, "ptt_btn", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "PTT button initialized on GPIO%d (Cherry MX, polling mode)", PTT_BUTTON_GPIO);
}

// Запуск калибровки и применение результата
// Forward declarations для stream_rx функций
static void stream_rx_stop(void);
static void stream_rx_start(void);

static void audio_calibrate_and_apply(void)
{
    // НЕ останавливаем стриминг - калибровка работает через перехват данных в TX/RX задачах
    // Если стриминг активен, audio_calibrate_delay() использует буферы s_calib_*
    // Если стриминг НЕ активен, используется прямой доступ к I2S
    
    ESP_LOGI(TAG, "Starting calibration (TX=%d, RX=%d)...", s_streaming_tx, s_streaming_rx);
    
    // Выполняем калибровку
    int delay = audio_calibrate_delay();
    
    if (delay > 0) {
        s_aecm_delay_ms = delay;
        ESP_LOGI(TAG, "AECM delay set to calibrated value: %d ms", delay);
        
        // Сохраняем в NVS
        save_audio_settings();
        
        // Обновляем UI если панель открыта
        if (s_slider_aecm_delay) {
            lv_slider_set_value(s_slider_aecm_delay, delay, LV_ANIM_OFF);
        }
        if (s_label_aecm_delay) {
            lv_label_set_text_fmt(s_label_aecm_delay, "Задержка: %d мс", delay);
        }
    } else {
        ESP_LOGW(TAG, "Calibration failed or cancelled");
    }
}

static void audio_record_and_play(void)
{
    if (!s_mic_dev || !s_spk_dev) {
        ESP_LOGE(TAG, "Audio devices not initialized!");
        return;
    }

    // Выделяем моно буфер для воспроизведения
    if (!s_record_buffer) {
        s_record_buffer = (int16_t *)heap_caps_malloc(RECORD_SAMPLES * sizeof(int16_t), 
                                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_record_buffer) {
            s_record_buffer = (int16_t *)heap_caps_malloc(RECORD_SAMPLES * sizeof(int16_t), 
                                                           MALLOC_CAP_SPIRAM);
        }
        if (!s_record_buffer) {
            ESP_LOGE(TAG, "Failed to allocate record buffer (%d bytes)", 
                     RECORD_SAMPLES * (int)sizeof(int16_t));
            return;
        }
        ESP_LOGI(TAG, "Allocated record buffer: %d bytes (DMA: %s)", 
                 RECORD_SAMPLES * (int)sizeof(int16_t),
                 esp_ptr_dma_capable(s_record_buffer) ? "yes" : "no");
    }

    // Буфер для чанков записи - большой для непрерывного потока
    #define REC_CHUNK_STEREO_SAMPLES 1024
    int16_t *chunk_buf = (int16_t *)heap_caps_malloc(REC_CHUNK_STEREO_SAMPLES * 2 * sizeof(int16_t), 
                                                      MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!chunk_buf) {
        ESP_LOGE(TAG, "Failed to allocate chunk buffer");
        return;
    }

    // Запись чанками (стерео -> моно) с повышенным приоритетом
    ESP_LOGI(TAG, "Recording %d ms...", RECORD_DURATION_MS);
    s_is_recording = true;
    
    // Повышаем приоритет на время записи
    UBaseType_t rec_old_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    
    size_t mono_samples_recorded = 0;
    size_t total_mono_samples = RECORD_SAMPLES;
    
    while (mono_samples_recorded < total_mono_samples) {
        size_t chunk_mono = total_mono_samples - mono_samples_recorded;
        if (chunk_mono > REC_CHUNK_STEREO_SAMPLES) {
            chunk_mono = REC_CHUNK_STEREO_SAMPLES;
        }
        
        // Читаем стерео чанк
        int ret = esp_codec_dev_read(s_mic_dev, chunk_buf, chunk_mono * 2 * sizeof(int16_t));
        if (ret < 0) {
            ESP_LOGE(TAG, "Read error: %d", ret);
            break;
        }
        
        // Конвертируем в моно (левый канал)
        for (size_t i = 0; i < chunk_mono; i++) {
            s_record_buffer[mono_samples_recorded + i] = chunk_buf[i * 2];
        }
        
        mono_samples_recorded += chunk_mono;
    }
    
    free(chunk_buf);
    s_is_recording = false;
    
    // Восстанавливаем приоритет после записи
    vTaskPrioritySet(NULL, rec_old_priority);
    
    ESP_LOGI(TAG, "Recording done! %d mono samples", (int)mono_samples_recorded);

    // Нормализация записи - находим максимум и масштабируем
    size_t total_samples = mono_samples_recorded;
    int16_t max_sample = 0;
    for (size_t i = 0; i < total_samples; i++) {
        int16_t abs_val = s_record_buffer[i] < 0 ? -s_record_buffer[i] : s_record_buffer[i];
        if (abs_val > max_sample) {
            max_sample = abs_val;
        }
    }
    
    ESP_LOGI(TAG, "Max sample before normalization: %d", max_sample);
    
    // Нормализуем только если есть что нормализовать (max > порог шума)
    if (max_sample > 500) {  // Порог шума
        // Масштабируем до 90% от максимума (30000 из 32767) чтобы избежать клиппинга
        float scale = 30000.0f / (float)max_sample;
        for (size_t i = 0; i < total_samples; i++) {
            int32_t scaled = (int32_t)(s_record_buffer[i] * scale);
            // Клиппинг на всякий случай
            if (scaled > 32767) scaled = 32767;
            if (scaled < -32768) scaled = -32768;
            s_record_buffer[i] = (int16_t)scaled;
        }
        ESP_LOGI(TAG, "Normalized with scale: %.2f", scale);
    } else {
        ESP_LOGI(TAG, "Signal too weak, skipping normalization");
    }

    // Небольшая пауза перед воспроизведением
    vTaskDelay(pdMS_TO_TICKS(50));

    // Воспроизведение - конвертируем моно в стерео (I2S работает в стерео)
    ESP_LOGI(TAG, "Playing back %d samples (mono->stereo)...", (int)total_samples);
    s_is_playing = true;
    
    // Повышаем приоритет задачи на время воспроизведения
    UBaseType_t old_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    
    // Воспроизводим большими чанками для непрерывности
    #define PLAY_CHUNK_MONO 2048
    size_t stereo_chunk_size = PLAY_CHUNK_MONO * 2 * sizeof(int16_t);
    int16_t *stereo_buf = (int16_t *)heap_caps_malloc(stereo_chunk_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!stereo_buf) {
        ESP_LOGE(TAG, "Failed to allocate stereo playback buffer");
        s_is_playing = false;
        vTaskPrioritySet(NULL, old_priority);
        return;
    }
    
    size_t mono_played = 0;
    while (mono_played < total_samples) {
        size_t chunk = total_samples - mono_played;
        if (chunk > PLAY_CHUNK_MONO) {
            chunk = PLAY_CHUNK_MONO;
        }
        
        // Конвертируем чанк моно в стерео
        for (size_t i = 0; i < chunk; i++) {
            int16_t sample = s_record_buffer[mono_played + i];
            stereo_buf[i * 2] = sample;      // Left
            stereo_buf[i * 2 + 1] = sample;  // Right
        }
        
        // Воспроизводим чанк
        esp_codec_dev_write(s_spk_dev, stereo_buf, chunk * 2 * sizeof(int16_t));
        mono_played += chunk;
    }
    
    free(stereo_buf);
    
    // Восстанавливаем приоритет
    vTaskPrioritySet(NULL, old_priority);
    
    ESP_LOGI(TAG, "Playback done!");
    s_is_playing = false;
}

// =============== UDP Audio Streaming ===============

static void udp_init(void)
{
    s_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_socket < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return;
    }

    // Увеличиваем буфер отправки чтобы избежать ENOMEM при быстрой передаче
    int sndbuf = 16384;  // 16KB буфер отправки
    setsockopt(s_udp_socket, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    
    // Bind to listen for incoming audio
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_AUDIO_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    
    if (bind(s_udp_socket, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket");
        close(s_udp_socket);
        s_udp_socket = -1;
        return;
    }

    // Non-blocking mode
    int flags = fcntl(s_udp_socket, F_GETFL, 0);
    fcntl(s_udp_socket, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "UDP socket initialized on port %d", UDP_AUDIO_PORT);
}

// Получить broadcast адрес из текущего IP
static void get_broadcast_addr(char *broadcast_ip, size_t len)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        // Broadcast = IP | ~Netmask
        uint32_t broadcast = ip_info.ip.addr | ~ip_info.netmask.addr;
        snprintf(broadcast_ip, len, "%d.%d.%d.%d", 
                 (int)((broadcast >> 0) & 0xFF),
                 (int)((broadcast >> 8) & 0xFF),
                 (int)((broadcast >> 16) & 0xFF),
                 (int)((broadcast >> 24) & 0xFF));
    } else {
        strncpy(broadcast_ip, "255.255.255.255", len);
    }
}

// =============== IMA ADPCM Codec ===============
// Таблицы для IMA ADPCM (стандарт)
static const int16_t ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static const int16_t ima_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

typedef struct {
    int16_t predicted;
    int8_t index;
} adpcm_state_t;

// Кодирование одного сэмпла в 4 бита
static uint8_t adpcm_encode_sample(int16_t sample, adpcm_state_t *state)
{
    int step = ima_step_table[state->index];
    int diff = sample - state->predicted;
    
    uint8_t code = 0;
    if (diff < 0) {
        code = 8;
        diff = -diff;
    }
    
    if (diff >= step) { code |= 4; diff -= step; }
    if (diff >= step >> 1) { code |= 2; diff -= step >> 1; }
    if (diff >= step >> 2) { code |= 1; }
    
    // Декодируем для обновления состояния
    diff = step >> 3;
    if (code & 4) diff += step;
    if (code & 2) diff += step >> 1;
    if (code & 1) diff += step >> 2;
    
    if (code & 8)
        state->predicted -= diff;
    else
        state->predicted += diff;
    
    if (state->predicted > 32767) state->predicted = 32767;
    if (state->predicted < -32768) state->predicted = -32768;
    
    state->index += ima_index_table[code];
    if (state->index < 0) state->index = 0;
    if (state->index > 88) state->index = 88;
    
    return code & 0x0F;
}

// Декодирование одного 4-битного кода в сэмпл
static int16_t adpcm_decode_sample(uint8_t code, adpcm_state_t *state)
{
    int step = ima_step_table[state->index];
    
    int diff = step >> 3;
    if (code & 4) diff += step;
    if (code & 2) diff += step >> 1;
    if (code & 1) diff += step >> 2;
    
    if (code & 8)
        state->predicted -= diff;
    else
        state->predicted += diff;
    
    if (state->predicted > 32767) state->predicted = 32767;
    if (state->predicted < -32768) state->predicted = -32768;
    
    state->index += ima_index_table[code];
    if (state->index < 0) state->index = 0;
    if (state->index > 88) state->index = 88;
    
    return state->predicted;
}

// Кодирование блока PCM в ADPCM (сжатие 4:1)
// Входные данные: samples сэмплов int16_t
// Выход: samples/2 байт (2 сэмпла на байт)
static void adpcm_encode_block(const int16_t *pcm, uint8_t *adpcm, int samples, adpcm_state_t *state)
{
    for (int i = 0; i < samples; i += 2) {
        uint8_t low = adpcm_encode_sample(pcm[i], state);
        uint8_t high = adpcm_encode_sample(pcm[i + 1], state);
        adpcm[i / 2] = low | (high << 4);
    }
}

// Декодирование блока ADPCM в PCM
static void adpcm_decode_block(const uint8_t *adpcm, int16_t *pcm, int samples, adpcm_state_t *state)
{
    for (int i = 0; i < samples; i += 2) {
        uint8_t byte = adpcm[i / 2];
        pcm[i] = adpcm_decode_sample(byte & 0x0F, state);
        pcm[i + 1] = adpcm_decode_sample(byte >> 4, state);
    }
}

// =============== Audio Tasks ===============

// ESP-SR AGC handle (используем отдельный AGC API вместо встроенного в AFE)
static void *s_esp_agc_handle = NULL;
static int16_t *s_agc_out_buf = NULL;  // Буфер для выходных данных AGC
#define ESP_AGC_FRAME_MS 10   // ESP-SR AGC работает с 10ms фреймами
#define ESP_AGC_FRAME_SIZE (AUDIO_SAMPLE_RATE * ESP_AGC_FRAME_MS / 1000)  // 160 samples при 16kHz

// ESP-SR AFE для эхоподавления (только AEC)
#if USE_AFE_AEC
static esp_afe_sr_iface_t *s_afe_iface = NULL;
static esp_afe_sr_data_t *s_afe_data = NULL;
static bool s_afe_initialized = false;

// Буфер для AFE (работает с 32мс фреймами при 16kHz = 512 samples)
#define AFE_FRAME_MS 32
#define AFE_FRAME_SIZE (AUDIO_SAMPLE_RATE * AFE_FRAME_MS / 1000)  // 512 samples при 16kHz
static int16_t s_afe_feed_buf[AFE_FRAME_SIZE * 2];  // interleaved: mic + ref

// Reference ring buffer (звук динамика для AEC) - должен быть большим для синхронизации
#define AFE_REF_BUF_SIZE (AFE_FRAME_SIZE * 16)  // 16 фреймов = 512ms буфер
static int16_t *s_afe_ref_ring_buf = NULL;
static volatile int s_afe_ref_write_pos = 0;
static volatile int s_afe_ref_read_pos = 0;

// AFE асинхронная обработка через ring buffer
#define AFE_RING_BUF_SIZE (AFE_FRAME_SIZE * 8)  // 8 фреймов = 256ms
static int16_t *s_afe_mic_ring_buf = NULL;      // Входной ring buffer (mic)
static int16_t *s_afe_out_ring_buf = NULL;      // Выходной ring buffer (обработанный)
static volatile int s_afe_mic_write_pos = 0;
static volatile int s_afe_mic_read_pos = 0;
static volatile int s_afe_out_write_pos = 0;
static volatile int s_afe_out_read_pos = 0;
static TaskHandle_t s_afe_task_handle = NULL;
static volatile bool s_afe_task_running = false;
#endif // USE_AFE_AEC

// ===== ESP-SR Lightweight AEC (aec_pro_create) =====
// Это НАМНОГО легче чем полный AFE (feed/fetch)
#if USE_ESP_AEC
#define ESP_AEC_FRAME_MS 32  // 32ms как в AFE
#define ESP_AEC_FRAME_SIZE_MAX 512  // Максимальный размер фрейма
// ESP-SR 2.x: используем enum aec_mode_t вместо числа
#define ESP_AEC_FILTER_LEN 16  // ~500ms эхо-хвост для лучшего подавления при сетевой задержке 
static aec_handle_t *s_esp_aec_handle = NULL;  // Указатель на структуру AEC
static volatile bool s_esp_aec_initialized = false;  // volatile для межзадачной синхронизации
static int s_esp_aec_frame_size = 0;           // Реальный размер фрейма от AEC
static int16_t *s_esp_aec_ref_buf = NULL;      // Reference буфер (звук динамика)
static int16_t *s_esp_aec_out_buf = NULL;      // Output буфер
static int16_t *s_esp_aec_mic_buf = NULL;      // Mic буфер для выравнивания
static SemaphoreHandle_t s_esp_aec_mutex = NULL;  // Мьютекс для защиты AEC от race condition
// Reference ring buffer для синхронизации mic/speaker
#define ESP_AEC_REF_BUF_SIZE (ESP_AEC_FRAME_SIZE_MAX * 32)  // 1024ms буфер - увеличен для сетевой задержки
static int16_t *s_esp_aec_ref_ring = NULL;
static volatile int s_esp_aec_ref_write = 0;
static volatile int s_esp_aec_ref_read = 0;
#endif

// ===== ESP-SR Noise Suppression =====
#if USE_ESP_NS
static ns_handle_t s_esp_ns_handle = NULL;
static bool s_esp_ns_initialized = false;
#define ESP_NS_FRAME_MS 10  // ESP-SR NS работает только с 10ms фреймами
#define ESP_NS_FRAME_SIZE (AUDIO_SAMPLE_RATE * ESP_NS_FRAME_MS / 1000)  // 160 samples при 16kHz
static int16_t *s_esp_ns_out_buf = NULL;  // Выходной буфер

static void esp_ns_init_custom(void)
{
    if (s_esp_ns_initialized) return;
    
    ESP_LOGI(TAG, "Initializing ESP-SR Noise Suppression...");
    ESP_LOGI(TAG, "Free heap: %lu, free PSRAM: %lu", 
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // ns_pro_create(frame_length_ms, mode, sample_rate)
    // mode: 0=Mild, 1=Medium, 2=Aggressive
    s_esp_ns_handle = ns_pro_create(ESP_NS_FRAME_MS, 1, AUDIO_SAMPLE_RATE);  // Medium mode - для подавления feedback
    if (!s_esp_ns_handle) {
        ESP_LOGE(TAG, "ns_pro_create failed!");
        return;
    }
    
    // Выделяем выходной буфер
    s_esp_ns_out_buf = heap_caps_malloc(ESP_NS_FRAME_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!s_esp_ns_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate NS output buffer");
        ns_destroy(s_esp_ns_handle);
        s_esp_ns_handle = NULL;
        return;
    }
    
    s_esp_ns_initialized = true;
    ESP_LOGI(TAG, "ESP-SR NS initialized (frame=%d samples, mode=Medium)", ESP_NS_FRAME_SIZE);
}

static void esp_ns_deinit_custom(void)
{
    if (s_esp_ns_handle) {
        ns_destroy(s_esp_ns_handle);
        s_esp_ns_handle = NULL;
    }
    if (s_esp_ns_out_buf) {
        heap_caps_free(s_esp_ns_out_buf);
        s_esp_ns_out_buf = NULL;
    }
    s_esp_ns_initialized = false;
    ESP_LOGI(TAG, "ESP-SR NS deinitialized");
}

// Обработка чанка через ESP-SR NS (обрабатываем по 160 samples = 10ms)
// Оптимизация: пропускаем каждый 2-й вызов для снижения нагрузки CPU
static void esp_ns_process_custom(int16_t *samples, int count)
{
    if (!s_esp_ns_initialized || !s_esp_ns_handle || !s_esp_ns_out_buf) return;
    
    // Пропускаем каждый 2-й вызов (экономия ~15-20% CPU)
    static int ns_skip_counter = 0;
    if (++ns_skip_counter % 2 != 0) return;
    
    // Обрабатываем по 160 samples (10ms)
    int processed = 0;
    while (processed < count) {
        int chunk = count - processed;
        if (chunk > ESP_NS_FRAME_SIZE) chunk = ESP_NS_FRAME_SIZE;
        if (chunk < ESP_NS_FRAME_SIZE) break;  // Неполный фрейм - пропускаем
        
        ns_process(s_esp_ns_handle, &samples[processed], s_esp_ns_out_buf);
        memcpy(&samples[processed], s_esp_ns_out_buf, chunk * sizeof(int16_t));
        processed += chunk;
    }
}
#endif // USE_ESP_NS

// Инициализация ESP-SR AGC (отдельно от AFE)
static void esp_agc_init_custom(void)
{
    if (s_esp_agc_handle) return;
    
    ESP_LOGI(TAG, "Initializing ESP-SR AGC...");
    
    // agc_mode: 0=fixed digital gain, 1=adaptive digital gain, 2=fixed analog gain, 3=adaptive analog gain
    s_esp_agc_handle = esp_agc_open(3, AUDIO_SAMPLE_RATE);
    if (s_esp_agc_handle) {
        // set_agc_config(handle, gain_dB, limiter_enable, target_level_dbfs)
        // gain_dB: дополнительное усиление в dB (0-90)
        // limiter_enable: 1 = включить лимитер
        // target_level_dbfs: целевой уровень в dBFS (0 = максимум, отрицательные значения тише)
        set_agc_config(s_esp_agc_handle, s_agc_gain_db, 1, -2);  // target=-2dBFS для максимальной громкости
        
        s_agc_out_buf = heap_caps_malloc(ESP_AGC_FRAME_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "ESP-SR AGC initialized: gain=%ddB, limiter=on, target=-2dBFS", s_agc_gain_db);
    } else {
        ESP_LOGE(TAG, "Failed to initialize ESP-SR AGC!");
    }
}

// Деинициализация ESP-SR AGC
static void esp_agc_deinit_custom(void)
{
    if (s_esp_agc_handle) {
        esp_agc_close(s_esp_agc_handle);
        s_esp_agc_handle = NULL;
    }
    if (s_agc_out_buf) {
        free(s_agc_out_buf);
        s_agc_out_buf = NULL;
    }
}

// Обработка через ESP-SR AGC
// Обрабатывает in-place, возвращает 0 при успехе
// Оптимизация: пропускаем каждый 2-й вызов для снижения нагрузки CPU
static int esp_agc_process_custom(int16_t *samples, int count)
{
    if (!s_agc_enabled || !s_esp_agc_handle || !s_agc_out_buf) return -1;
    
    // Пропускаем каждый 2-й вызов (экономия ~10% CPU)
    static int agc_skip_counter = 0;
    if (++agc_skip_counter % 2 != 0) return 0;
    
    // ESP-SR AGC работает с фреймами по 10ms (160 samples при 16kHz)
    int processed = 0;
    while (processed < count) {
        int chunk = count - processed;
        if (chunk > ESP_AGC_FRAME_SIZE) chunk = ESP_AGC_FRAME_SIZE;
        
        // Если chunk меньше ESP_AGC_FRAME_SIZE, пропускаем (AGC требует ровно 10ms)
        if (chunk < ESP_AGC_FRAME_SIZE) {
            break;
        }
        
        int ret = esp_agc_process(s_esp_agc_handle, samples + processed, s_agc_out_buf, ESP_AGC_FRAME_SIZE, AUDIO_SAMPLE_RATE);
        if (ret == ESP_AGC_SUCCESS) {
            // Копируем результат обратно
            memcpy(samples + processed, s_agc_out_buf, ESP_AGC_FRAME_SIZE * sizeof(int16_t));
        }
        
        processed += ESP_AGC_FRAME_SIZE;
    }
    
    return 0;
}

// ===== ESP-SR VAD для full-duplex =====
#if USE_AFE_AEC
#include "esp_vad.h"

static vad_handle_t s_vad_handle = NULL;
#define VAD_FRAME_MS 30  // VAD работает с 30мс фреймами
#define VAD_FRAME_SIZE (AUDIO_SAMPLE_RATE * VAD_FRAME_MS / 1000)  // 480 samples при 16kHz

// Счётчик для сглаживания VAD (не отключаемся сразу после короткой паузы)
static int s_vad_speech_frames = 0;
static int s_vad_silence_frames = 0;
#define VAD_SPEECH_HOLD_FRAMES 10  // Держим "речь" ещё 10 фреймов (~300мс) после последней детекции

static void vad_init_custom(void)
{
    if (s_vad_handle) return;
    
    // VAD_MODE_3 = самый агрессивный (меньше ложных срабатываний на шум динамика)
    s_vad_handle = vad_create(VAD_MODE_3);
    if (s_vad_handle) {
        ESP_LOGI(TAG, "ESP-SR VAD initialized (mode=3, frame=%dms)", VAD_FRAME_MS);
    } else {
        ESP_LOGE(TAG, "Failed to initialize ESP-SR VAD!");
    }
}

static void vad_deinit_custom(void)
{
    if (s_vad_handle) {
        vad_destroy(s_vad_handle);
        s_vad_handle = NULL;
    }
    s_vad_speech_frames = 0;
    s_vad_silence_frames = 0;
}

// Проверяет, есть ли голосовая активность в буфере
// Возвращает true если обнаружена речь (с hysteresis)
static bool vad_check_speech(int16_t *samples, int count)
{
    if (!s_vad_handle) return true;  // Если VAD не инициализирован - передаём всё
    
    // VAD работает с 30мс фреймами
    bool any_speech = false;
    int processed = 0;
    
    while (processed + VAD_FRAME_SIZE <= count) {
        vad_state_t state = vad_process(s_vad_handle, samples + processed, AUDIO_SAMPLE_RATE, VAD_FRAME_MS);
        if (state == VAD_SPEECH) {
            any_speech = true;
            break;
        }
        processed += VAD_FRAME_SIZE;
    }
    
    // Hysteresis: держим "речь" ещё несколько фреймов после детекции
    if (any_speech) {
        s_vad_speech_frames = VAD_SPEECH_HOLD_FRAMES;
        s_vad_silence_frames = 0;
    } else {
        if (s_vad_speech_frames > 0) {
            s_vad_speech_frames--;
        } else {
            s_vad_silence_frames++;
        }
    }
    
    // Возвращаем true если недавно была речь
    return (s_vad_speech_frames > 0);
}

static void afe_init(void)
{
    if (s_afe_initialized) return;
    
    ESP_LOGI(TAG, "Initializing ESP-SR AFE for echo cancellation...");
    ESP_LOGI(TAG, "Free heap: %lu, free PSRAM: %lu", 
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // Создаём конфиг с нуля (не используем макрос который ставит mic_num=2 для ESP32S3)
    afe_config_t afe_config = {0};
    
    // Базовые настройки
    afe_config.aec_init = true;                     // AEC включён
    afe_config.se_init = false;                     // Шумоподавление выключено
    afe_config.vad_init = false;                    // VAD не нужен
    afe_config.wakenet_init = false;                // Wake word не нужен
    afe_config.voice_communication_init = true;     // Режим голосовой связи
    afe_config.voice_communication_agc_init = false; // AGC выключен (используем отдельный esp_agc)
    afe_config.voice_communication_agc_gain = 15;
    afe_config.vad_mode = VAD_MODE_3;
    afe_config.wakenet_model_name = NULL;
    afe_config.wakenet_model_name_2 = NULL;
    afe_config.wakenet_mode = DET_MODE_90;
    afe_config.afe_mode = SR_MODE_LOW_COST;         // Экономный режим
    afe_config.afe_perferred_core = 1;              // На втором ядре
    afe_config.afe_perferred_priority = 5;
    afe_config.afe_ringbuf_size = 50;
    afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config.afe_linear_gain = 1.0f;
    afe_config.agc_mode = AFE_MN_PEAK_NO_AGC;       // Без AGC для SR
    afe_config.debug_init = false;
    afe_config.debug_hook[0].hook_type = AFE_DEBUG_HOOK_MASE_TASK_IN;
    afe_config.debug_hook[0].hook_callback = NULL;
    afe_config.debug_hook[1].hook_type = AFE_DEBUG_HOOK_FETCH_TASK_IN;
    afe_config.debug_hook[1].hook_callback = NULL;
    afe_config.afe_ns_mode = NS_MODE_SSP;
    afe_config.afe_ns_model_name = NULL;
    afe_config.fixed_first_channel = true;
    
    // PCM конфигурация: 1 микрофон + 1 reference канал (ВАЖНО: должно быть total = mic + ref)
    afe_config.pcm_config.total_ch_num = 2;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 1;
    afe_config.pcm_config.sample_rate = AUDIO_SAMPLE_RATE;  // 16000 Hz
    
    ESP_LOGI(TAG, "AFE config: aec=%d, se=%d, vad=%d, wakenet=%d, vc=%d, vc_agc=%d",
             afe_config.aec_init, afe_config.se_init, afe_config.vad_init,
             afe_config.wakenet_init, afe_config.voice_communication_init,
             afe_config.voice_communication_agc_init);
    ESP_LOGI(TAG, "AFE PCM: total_ch=%d, mic=%d, ref=%d, rate=%d",
             afe_config.pcm_config.total_ch_num, afe_config.pcm_config.mic_num,
             afe_config.pcm_config.ref_num, afe_config.pcm_config.sample_rate);
    
    // Используем VC (Voice Communication) handle
    s_afe_iface = &ESP_AFE_VC_HANDLE;
    
    ESP_LOGI(TAG, "Creating AFE instance...");
    s_afe_data = s_afe_iface->create_from_config(&afe_config);
    
    if (s_afe_data) {
        s_afe_initialized = true;
        ESP_LOGI(TAG, "ESP-SR AFE initialized successfully!");
    } else {
        ESP_LOGE(TAG, "Failed to initialize ESP-SR AFE!");
    }
}

static void afe_deinit(void)
{
    if (s_afe_initialized && s_afe_iface && s_afe_data) {
        s_afe_iface->destroy(s_afe_data);
        s_afe_data = NULL;
        s_afe_initialized = false;
        ESP_LOGI(TAG, "ESP-SR AFE deinitialized");
    }
}
#endif // USE_AFE_AEC

// ===== ESP-SR Lightweight AEC (aec_pro_create) =====
#if USE_ESP_AEC
static void esp_aec_lightweight_init(void)
{
    if (s_esp_aec_initialized) return;
    
    ESP_LOGI(TAG, "Initializing ESP-SR Lightweight AEC...");
    ESP_LOGI(TAG, "Free heap: %lu, free PSRAM: %lu", 
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // Создаём мьютекс для защиты AEC от race condition
    if (!s_esp_aec_mutex) {
        s_esp_aec_mutex = xSemaphoreCreateMutex();
        if (!s_esp_aec_mutex) {
            ESP_LOGE(TAG, "Failed to create AEC mutex!");
            return;
        }
    }
    
    // ESP-SR 2.x: aec_pro_create(filter_length, channel_num, aec_mode_t mode)
    // AEC_MODE_VOIP_LOW_COST = 0 - для VoIP/интеркома (легче для CPU)
    // AEC_MODE_VOIP_HIGH_PERF = 1 - высокое качество подавления эха
    s_esp_aec_handle = aec_pro_create(ESP_AEC_FILTER_LEN, 1, AEC_MODE_VOIP_HIGH_PERF);
    if (!s_esp_aec_handle) {
        ESP_LOGE(TAG, "aec_pro_create failed!");
        return;
    }
    
    // Получаем реальный размер фрейма от AEC
    s_esp_aec_frame_size = aec_get_chunksize(s_esp_aec_handle);
    ESP_LOGI(TAG, "AEC frame size from library: %d samples", s_esp_aec_frame_size);
    
    if (s_esp_aec_frame_size <= 0 || s_esp_aec_frame_size > ESP_AEC_FRAME_SIZE_MAX) {
        ESP_LOGE(TAG, "Invalid AEC frame size: %d, using default 512", s_esp_aec_frame_size);
        s_esp_aec_frame_size = 512;
    }
    
    // Выделяем буферы (должны быть выровнены по 16 байт)
    s_esp_aec_ref_buf = heap_caps_aligned_alloc(16, s_esp_aec_frame_size * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    s_esp_aec_out_buf = heap_caps_aligned_alloc(16, s_esp_aec_frame_size * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    s_esp_aec_mic_buf = heap_caps_aligned_alloc(16, s_esp_aec_frame_size * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    s_esp_aec_ref_ring = heap_caps_calloc(ESP_AEC_REF_BUF_SIZE, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    
    if (!s_esp_aec_ref_buf || !s_esp_aec_out_buf || !s_esp_aec_mic_buf || !s_esp_aec_ref_ring) {
        ESP_LOGE(TAG, "Failed to allocate AEC buffers!");
        if (s_esp_aec_handle) aec_destroy(s_esp_aec_handle);
        if (s_esp_aec_ref_buf) heap_caps_free(s_esp_aec_ref_buf);
        if (s_esp_aec_out_buf) heap_caps_free(s_esp_aec_out_buf);
        if (s_esp_aec_mic_buf) heap_caps_free(s_esp_aec_mic_buf);
        if (s_esp_aec_ref_ring) heap_caps_free(s_esp_aec_ref_ring);
        s_esp_aec_handle = NULL;
        s_esp_aec_ref_buf = NULL;
        s_esp_aec_out_buf = NULL;
        s_esp_aec_mic_buf = NULL;
        s_esp_aec_ref_ring = NULL;
        return;
    }
    
    s_esp_aec_ref_write = 0;
    s_esp_aec_ref_read = 0;
    s_esp_aec_initialized = true;
    
    ESP_LOGI(TAG, "ESP-SR Lightweight AEC initialized! filter_len=%d, mode=VOIP_HIGH_PERF, frame_size=%d", 
             ESP_AEC_FILTER_LEN, s_esp_aec_frame_size);
}

static void esp_aec_lightweight_deinit(void)
{
    if (s_esp_aec_initialized) {
        if (s_esp_aec_handle) {
            aec_destroy(s_esp_aec_handle);
            s_esp_aec_handle = NULL;
        }
        if (s_esp_aec_ref_buf) {
            heap_caps_free(s_esp_aec_ref_buf);
            s_esp_aec_ref_buf = NULL;
        }
        if (s_esp_aec_out_buf) {
            heap_caps_free(s_esp_aec_out_buf);
            s_esp_aec_out_buf = NULL;
        }
        if (s_esp_aec_mic_buf) {
            heap_caps_free(s_esp_aec_mic_buf);
            s_esp_aec_mic_buf = NULL;
        }
        if (s_esp_aec_ref_ring) {
            heap_caps_free(s_esp_aec_ref_ring);
            s_esp_aec_ref_ring = NULL;
        }
        s_esp_aec_initialized = false;
        ESP_LOGI(TAG, "ESP-SR Lightweight AEC deinitialized");
    }
}

// Сброс состояния AEC (очистка буферов) для чистого старта
static void esp_aec_reset(void)
{
    if (!s_esp_aec_initialized || !s_esp_aec_handle || !s_esp_aec_mutex) return;
    
    // Захватываем мьютекс для безопасного сброса
    if (xSemaphoreTake(s_esp_aec_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "AEC reset: mutex timeout");
        return;
    }
    
    // Только сбрасываем ring buffer, не пересоздаём AEC handle (дорого!)
    s_esp_aec_ref_write = 0;
    s_esp_aec_ref_read = 0;
    if (s_esp_aec_ref_ring) {
        memset(s_esp_aec_ref_ring, 0, ESP_AEC_REF_BUF_SIZE * sizeof(int16_t));
    }
    
    xSemaphoreGive(s_esp_aec_mutex);
    
    ESP_LOGI(TAG, "ESP-SR AEC reset (buffers cleared)");
}

// Добавить reference (speaker) samples в ring buffer
static void esp_aec_add_reference(const int16_t *samples, int count)
{
    // Проверяем что AEC инициализирован И буфер создан
    if (!s_esp_aec_initialized || !s_esp_aec_ref_ring) return;
    
    for (int i = 0; i < count; i++) {
        s_esp_aec_ref_ring[s_esp_aec_ref_write] = samples[i];
        int next_write = (s_esp_aec_ref_write + 1) % ESP_AEC_REF_BUF_SIZE;
        // Если буфер переполнен - двигаем read pointer чтобы не потерять синхронизацию
        if (next_write == s_esp_aec_ref_read) {
            s_esp_aec_ref_read = (s_esp_aec_ref_read + 1) % ESP_AEC_REF_BUF_SIZE;
        }
        s_esp_aec_ref_write = next_write;
    }
}

// Получить количество доступных reference samples
static int esp_aec_ref_available(void)
{
    int avail = s_esp_aec_ref_write - s_esp_aec_ref_read;
    if (avail < 0) avail += ESP_AEC_REF_BUF_SIZE;
    return avail;
}

// Обработка микрофонных данных через AEC (синхронно)
// mic_samples: входные данные с микрофона, результат записывается обратно
static void esp_aec_process_mic(int16_t *mic_samples, int count)
{
    if (!s_esp_aec_initialized || !s_esp_aec_handle || s_esp_aec_frame_size <= 0 || !s_esp_aec_mutex) return;
    
    // Пытаемся захватить мьютекс без блокировки
    if (xSemaphoreTake(s_esp_aec_mutex, 0) != pdTRUE) {
        // Мьютекс занят (AEC reset в процессе) - пропускаем обработку
        return;
    }
    
    // Повторная проверка после захвата мьютекса
    if (!s_esp_aec_handle) {
        xSemaphoreGive(s_esp_aec_mutex);
        return;
    }
    
    // Минимум reference данных для работы AEC (1 фрейм достаточно)
    int min_ref = s_esp_aec_frame_size;
    
    // Проверяем достаточно ли reference данных для корректной работы
    int total_ref_avail = esp_aec_ref_available();
    
    // Диагностика - логируем раз в секунду если reference мало
    static uint32_t last_ref_log = 0;
    static int aec_skipped = 0;
    static int aec_processed = 0;
    
    if (total_ref_avail < min_ref) {
        aec_skipped++;
        uint32_t now = xTaskGetTickCount();
        if (now - last_ref_log > pdMS_TO_TICKS(2000)) {
            ESP_LOGW(TAG, "AEC: ref_avail=%d < min=%d, skipped=%d, processed=%d", 
                     total_ref_avail, min_ref, aec_skipped, aec_processed);
            last_ref_log = now;
            aec_skipped = 0;
            aec_processed = 0;
        }
        xSemaphoreGive(s_esp_aec_mutex);
        return;
    }
    
    int processed = 0;
    // Обрабатываем ВСЕ данные без ограничений (512 samples = 1 фрейм обычно)
    while (processed < count) {
        int chunk = count - processed;
        if (chunk > s_esp_aec_frame_size) chunk = s_esp_aec_frame_size;
        
        // AEC требует ПОЛНЫЙ фрейм - если меньше, пропускаем
        if (chunk < s_esp_aec_frame_size) {
            break;
        }
        
        // Готовим reference из ring buffer (ТОЧНО s_esp_aec_frame_size samples)
        int ref_avail = esp_aec_ref_available();
        if (ref_avail < s_esp_aec_frame_size) {
            // Недостаточно reference для полного фрейма - выходим
            break;
        }
        
        for (int i = 0; i < s_esp_aec_frame_size; i++) {
            s_esp_aec_ref_buf[i] = s_esp_aec_ref_ring[s_esp_aec_ref_read];
            s_esp_aec_ref_read = (s_esp_aec_ref_read + 1) % ESP_AEC_REF_BUF_SIZE;
        }
        
        // Копируем mic данные в выровненный буфер
        memcpy(s_esp_aec_mic_buf, &mic_samples[processed], s_esp_aec_frame_size * sizeof(int16_t));
        
        // AEC обработка: aec_process(handle, mic, ref, out)
        aec_process(s_esp_aec_handle, s_esp_aec_mic_buf, s_esp_aec_ref_buf, s_esp_aec_out_buf);
        
        // Копируем результат обратно
        memcpy(&mic_samples[processed], s_esp_aec_out_buf, s_esp_aec_frame_size * sizeof(int16_t));
        
        processed += s_esp_aec_frame_size;
        aec_processed++;
    }
    
    xSemaphoreGive(s_esp_aec_mutex);
}
#endif  // USE_ESP_AEC

// Проверить сколько reference данных доступно
#if USE_AFE_AEC
static int afe_ref_available(void)
{
    int avail = s_afe_ref_write_pos - s_afe_ref_read_pos;
    if (avail < 0) avail += AFE_REF_BUF_SIZE;
    return avail;
}

// Добавить reference samples (воспроизводимый звук) в кольцевой буфер
static void afe_add_reference_samples(const int16_t *samples, int count)
{
    if (!s_afe_ref_ring_buf) return;
    for (int i = 0; i < count; i++) {
        s_afe_ref_ring_buf[s_afe_ref_write_pos] = samples[i];
        s_afe_ref_write_pos = (s_afe_ref_write_pos + 1) % AFE_REF_BUF_SIZE;
    }
}

// Обработать микрофонные данные через AFE для удаления эха
// mic_samples: входные данные с микрофона (16kHz, моно)
// count: количество сэмплов
// Результат записывается обратно в mic_samples
static void afe_process_mic(int16_t *mic_samples, int count)
{
    if (!s_afe_initialized || !s_afe_data || !s_afe_iface) return;
    
    // AFE требует данные в формате interleaved: [mic0, ref0, mic1, ref1, ...]
    // Обрабатываем по AFE_FRAME_SIZE сэмплов за раз
    
    int processed = 0;
    while (processed < count) {
        int chunk = count - processed;
        if (chunk > AFE_FRAME_SIZE) chunk = AFE_FRAME_SIZE;
        
        // Подготавливаем interleaved буфер
        int ref_avail = afe_ref_available();
        for (int i = 0; i < chunk; i++) {
            s_afe_feed_buf[i * 2] = mic_samples[processed + i];     // mic
            if (ref_avail > 0 && s_afe_ref_ring_buf) {
                s_afe_feed_buf[i * 2 + 1] = s_afe_ref_ring_buf[s_afe_ref_read_pos];
                s_afe_ref_read_pos = (s_afe_ref_read_pos + 1) % AFE_REF_BUF_SIZE;
                ref_avail--;
            } else {
                s_afe_feed_buf[i * 2 + 1] = 0;  // Нет reference - тишина
            }
        }
        
        // Дополняем нулями если chunk < AFE_FRAME_SIZE
        for (int i = chunk; i < AFE_FRAME_SIZE; i++) {
            s_afe_feed_buf[i * 2] = 0;
            s_afe_feed_buf[i * 2 + 1] = 0;
        }
        
        // Подаём данные в AFE
        s_afe_iface->feed(s_afe_data, s_afe_feed_buf);
        
        // Получаем обработанные данные
        afe_fetch_result_t *result = s_afe_iface->fetch(s_afe_data);
        if (result && result->data && result->data_size > 0) {
            // Копируем результат обратно (AEC + NS уже применены)
            int out_samples = result->data_size / sizeof(int16_t);
            if (out_samples > chunk) out_samples = chunk;
            for (int i = 0; i < out_samples; i++) {
                mic_samples[processed + i] = result->data[i];
            }
        }
        
        processed += chunk;
    }
}

// ========== AFE ASYNC TASK ==========
// Отдельная задача для асинхронной обработки AFE (не блокирует TX)

// Добавить mic samples в AFE ring buffer (вызывается из TX)
static int afe_ring_buf_available(void)
{
    int avail = s_afe_mic_write_pos - s_afe_mic_read_pos;
    if (avail < 0) avail += AFE_RING_BUF_SIZE;
    return avail;
}

static int afe_out_buf_available(void)
{
    int avail = s_afe_out_write_pos - s_afe_out_read_pos;
    if (avail < 0) avail += AFE_RING_BUF_SIZE;
    return avail;
}

static void afe_push_mic_samples(const int16_t *samples, int count)
{
    if (!s_afe_mic_ring_buf) return;
    for (int i = 0; i < count; i++) {
        s_afe_mic_ring_buf[s_afe_mic_write_pos] = samples[i];
        s_afe_mic_write_pos = (s_afe_mic_write_pos + 1) % AFE_RING_BUF_SIZE;
    }
}

static int afe_pop_processed_samples(int16_t *samples, int max_count)
{
    if (!s_afe_out_ring_buf) return 0;
    int avail = afe_out_buf_available();
    int count = (avail < max_count) ? avail : max_count;
    for (int i = 0; i < count; i++) {
        samples[i] = s_afe_out_ring_buf[s_afe_out_read_pos];
        s_afe_out_read_pos = (s_afe_out_read_pos + 1) % AFE_RING_BUF_SIZE;
    }
    return count;
}

// AFE задача - обрабатывает mic данные и пишет результат в out buffer
static void afe_async_task(void *arg)
{
    ESP_LOGI(TAG, "AFE async task started");
    
    int16_t mic_frame[AFE_FRAME_SIZE];
    
    while (s_afe_task_running) {
        // Ждём пока накопится AFE_FRAME_SIZE сэмплов
        if (afe_ring_buf_available() >= AFE_FRAME_SIZE) {
            // Читаем фрейм из mic ring buffer
            for (int i = 0; i < AFE_FRAME_SIZE; i++) {
                mic_frame[i] = s_afe_mic_ring_buf[s_afe_mic_read_pos];
                s_afe_mic_read_pos = (s_afe_mic_read_pos + 1) % AFE_RING_BUF_SIZE;
            }
            
            // Готовим interleaved буфер [mic, ref, mic, ref, ...]
            // Если reference не готов - используем 0 (тишина)
            int ref_avail = afe_ref_available();
            for (int i = 0; i < AFE_FRAME_SIZE; i++) {
                s_afe_feed_buf[i * 2] = mic_frame[i];
                if (ref_avail > 0 && s_afe_ref_ring_buf) {
                    s_afe_feed_buf[i * 2 + 1] = s_afe_ref_ring_buf[s_afe_ref_read_pos];
                    s_afe_ref_read_pos = (s_afe_ref_read_pos + 1) % AFE_REF_BUF_SIZE;
                    ref_avail--;
                } else {
                    s_afe_feed_buf[i * 2 + 1] = 0;  // Нет reference - тишина
                }
            }
            
            // AFE feed/fetch (блокирует, но в отдельной задаче)
            s_afe_iface->feed(s_afe_data, s_afe_feed_buf);
            afe_fetch_result_t *result = s_afe_iface->fetch(s_afe_data);
            
            if (result && result->data && result->data_size > 0) {
                int out_samples = result->data_size / sizeof(int16_t);
                // Пишем в выходной ring buffer
                for (int i = 0; i < out_samples; i++) {
                    s_afe_out_ring_buf[s_afe_out_write_pos] = result->data[i];
                    s_afe_out_write_pos = (s_afe_out_write_pos + 1) % AFE_RING_BUF_SIZE;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));  // Ждём данные
        }
    }
    
    ESP_LOGI(TAG, "AFE async task stopped");
    vTaskDelete(NULL);
}

// Запуск AFE async задачи
static void afe_async_start(void)
{
    if (s_afe_task_running || !s_afe_initialized) return;
    
    // Выделяем ring buffers в PSRAM
    if (!s_afe_mic_ring_buf) {
        s_afe_mic_ring_buf = heap_caps_malloc(AFE_RING_BUF_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    }
    if (!s_afe_out_ring_buf) {
        s_afe_out_ring_buf = heap_caps_malloc(AFE_RING_BUF_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    }
    if (!s_afe_ref_ring_buf) {
        s_afe_ref_ring_buf = heap_caps_malloc(AFE_REF_BUF_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    }
    
    if (!s_afe_mic_ring_buf || !s_afe_out_ring_buf || !s_afe_ref_ring_buf) {
        ESP_LOGE(TAG, "Failed to allocate AFE ring buffers");
        return;
    }
    
    // Сбрасываем позиции
    s_afe_mic_write_pos = 0;
    s_afe_mic_read_pos = 0;
    s_afe_out_write_pos = 0;
    s_afe_out_read_pos = 0;
    s_afe_ref_write_pos = 0;
    s_afe_ref_read_pos = 0;
    
    s_afe_task_running = true;
    // ESP-SR AFE requires large stack (feed/fetch use deep call stacks)
    xTaskCreatePinnedToCore(afe_async_task, "afe_async", 8192, NULL, 10, &s_afe_task_handle, 1);
    ESP_LOGI(TAG, "AFE async processing started");
}

// Остановка AFE async задачи
static void afe_async_stop(void)
{
    if (!s_afe_task_running) return;
    s_afe_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));  // Ждём завершения
    s_afe_task_handle = NULL;
    ESP_LOGI(TAG, "AFE async processing stopped");
}
#endif // USE_AFE_AEC

// Старый кастомный AGC удалён - теперь используется ESP-SR AGC API (esp_agc.h)

// Задача передачи аудио (PCM через esp_codec_dev)
static void audio_tx_task(void *arg)
{
    ESP_LOGI(TAG, "Audio TX task started");
    
    // Пытаемся отключить watchdog (игнорируем ошибку если task не зарегистрирован)
    esp_err_t wdt_err = esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "WDT delete: %s", esp_err_to_name(wdt_err));
    }
    
    // AGC, VAD, NS и AEC уже инициализированы в app_main (персистентные)
    // НЕ инициализируем их здесь чтобы сэкономить стек!
    
    // Минимальная задержка для синхронизации с audio_task
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Проверяем сокет
    if (s_udp_socket < 0) {
        ESP_LOGE(TAG, "TX: UDP socket not initialized!");
        s_streaming_tx = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Используем статические буферы (выделяются один раз при первом запуске)
    if (!s_tx_buf) {
        s_tx_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (!s_tx_mono_buf) {
        s_tx_mono_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    }
    if (!s_tx_adpcm_buf) {
        s_tx_adpcm_buf = (uint8_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE / 2, MALLOC_CAP_DEFAULT);
    }
    
    int16_t *tx_buf = s_tx_buf;
    int16_t *mono_buf = s_tx_mono_buf;
    uint8_t *adpcm_buf = s_tx_adpcm_buf;
    
    if (!tx_buf || !mono_buf || !adpcm_buf) {
        ESP_LOGE(TAG, "Failed to allocate TX buffers");
        s_streaming_tx = false;
        vTaskDelete(NULL);
        return;
    }
    
    // ADPCM state для кодирования
    adpcm_state_t adpcm_enc_state = {0, 0};

    // Определяем адрес назначения: сохранённый IP или broadcast
    char dest_ip[16] = {0};
    bool use_broadcast = (s_target_host[0] == '\0');
    
    if (!use_broadcast) {
        // Используем ТОЛЬКО кэшированный IP (обновляется фоновым mDNS resolver)
        // НЕ делаем mDNS query здесь - это блокирует CPU1 и вызывает WDT!
        if (s_target_ip[0] != '\0') {
            strncpy(dest_ip, s_target_ip, sizeof(dest_ip) - 1);
            ESP_LOGI(TAG, "Using cached IP for %s -> %s", s_target_host, dest_ip);
        } else {
            // Кэш пустой - не можем передавать без IP
            ESP_LOGE(TAG, "No cached IP for %s! Aborting TX.", s_target_host);
            ESP_LOGW(TAG, "Wait for mDNS background resolver to find the device...");
            s_streaming_tx = false;
            vTaskDelete(NULL);
            return;
        }
    }
    
    // Проверяем, есть ли IP для отправки
    bool has_dest_ip = (dest_ip[0] != '\0');
    
    if (use_broadcast) {
        get_broadcast_addr(dest_ip, sizeof(dest_ip));
        has_dest_ip = true;
        ESP_LOGI(TAG, "Broadcasting audio to %s:%d", dest_ip, UDP_AUDIO_PORT);
    } else if (has_dest_ip) {
        ESP_LOGI(TAG, "Unicast audio to %s (%s):%d", s_target_host, dest_ip, UDP_AUDIO_PORT);
    } else {
        ESP_LOGW(TAG, "No destination IP yet, will resolve during TX...");
    }

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_AUDIO_PORT),
    };
    inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr);

    if (use_broadcast) {
        int broadcast_enable = 1;
        setsockopt(s_udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));
    }

    uint32_t tx_packets = 0;
    uint32_t tx_last_log = 0;
    uint32_t tx_start_time = 0;
    uint32_t mdns_retry_time = 0;
    
    // CPU load measurement using tick count
    uint32_t process_ticks_total = 0;
    uint32_t process_ticks_max = 0;
    uint32_t process_count = 0;
    
    while (s_streaming_tx) {
        // Если IP ещё не разрешён - проверяем кэш (mDNS query блокирует CPU!)
        if (!has_dest_ip && !use_broadcast) {
            uint32_t now = xTaskGetTickCount();
            if ((now - mdns_retry_time) > pdMS_TO_TICKS(500)) {
                mdns_retry_time = now;
                // Проверяем, не разрешился ли IP фоново
                if (s_target_ip[0] != '\0') {
                    strncpy(dest_ip, s_target_ip, sizeof(dest_ip) - 1);
                    inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr);
                    has_dest_ip = true;
                    ESP_LOGI(TAG, "Got cached IP from background: %s", dest_ip);
                } else {
                    ESP_LOGW(TAG, "Still waiting for mDNS background resolution of %s...", s_target_host);
                }
            }
            // Не отправляем пока нет IP - читаем и отбрасываем
            int ret = esp_codec_dev_read(s_mic_dev, tx_buf, AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t));
            (void)ret;
            vTaskDelay(pdMS_TO_TICKS(16));  // ~1 frame time
            continue;
        }
        
        int ret = esp_codec_dev_read(s_mic_dev, tx_buf, AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t));
        if (ret < 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        
        // Start CPU time measurement (using CPU cycle count)
        uint32_t t_start = xthal_get_ccount();

        // Стерео→моно
        for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
            int32_t sample = ((int32_t)tx_buf[i * 2] + tx_buf[i * 2 + 1]) / 2;
            mono_buf[i] = (int16_t)sample;
        }
        
        // ===== ESP-SR AFE AEC: Удаление эха от динамика =====
#if USE_AFE_AEC
#if USE_AFE_SYNC
        // Синхронный режим: обрабатываем прямо здесь
        if (s_afe_initialized && s_afe_data && s_afe_iface) {
            afe_process_mic(mono_buf, AUDIO_CHUNK_SIZE);
        }
#else
        // Async режим (deprecated)
        if (s_afe_initialized && s_afe_task_running) {
            afe_push_mic_samples(mono_buf, AUDIO_CHUNK_SIZE);
            int processed = afe_pop_processed_samples(mono_buf, AUDIO_CHUNK_SIZE);
            if (processed < AUDIO_CHUNK_SIZE) {
                // AFE ещё не обработал - используем raw данные
            }
        }
#endif
#endif

        // ===== ESP-SR Lightweight AEC: Удаление эха от динамика =====
        // AEC применяется ТОЛЬКО когда есть активный RX (звук из динамика)
#if USE_ESP_AEC
        if (s_esp_aec_initialized && s_rx_active) {
            esp_aec_process_mic(mono_buf, AUDIO_CHUNK_SIZE);
        }
#endif
        
        // ===== Noise Suppression =====
#if USE_ESP_NS
        esp_ns_process_custom(mono_buf, AUDIO_CHUNK_SIZE);
#endif
        
#if USE_AGC
        // ESP-SR AGC: усиление голоса +36dB
        esp_agc_process_custom(mono_buf, AUDIO_CHUNK_SIZE);
#endif

        // ===== Noise Gate: подавление тихих звуков =====
        if (s_noise_gate_threshold > 0) {
            // Находим максимальную амплитуду в чанке
            int16_t max_amp = 0;
            for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
                int16_t amp = mono_buf[i] < 0 ? -mono_buf[i] : mono_buf[i];
                if (amp > max_amp) max_amp = amp;
            }
            // Логируем для отладки
            static uint32_t last_ng_log = 0;
            static int ng_muted = 0;
            static int ng_passed = 0;
            // Если амплитуда ниже порога - заглушаем весь чанк
            if (max_amp < s_noise_gate_threshold) {
                memset(mono_buf, 0, AUDIO_CHUNK_SIZE * sizeof(int16_t));
                ng_muted++;
            } else {
                ng_passed++;
            }
            uint32_t now = xTaskGetTickCount();
            if (now - last_ng_log > pdMS_TO_TICKS(2000)) {
                ESP_LOGI(TAG, "TX NoiseGate: max=%d thresh=%d, muted=%d passed=%d", 
                         max_amp, s_noise_gate_threshold, ng_muted, ng_passed);
                last_ng_log = now;
                ng_muted = 0;
                ng_passed = 0;
            }
        }
        
        // ===== Full-Duplex: VAD или простой VOX =====
#if USE_VAD
        // VAD определяет говорит ли ЛОКАЛЬНЫЙ пользователь
        // Если есть речь - передаём, даже если динамик играет
        // VAD отфильтрует звук с динамика как "не речь"
        bool local_speech = vad_check_speech(mono_buf, AUDIO_CHUNK_SIZE);
        if (!local_speech && s_rx_active) {
            // Нет локальной речи И играет звук с другого устройства
            // -> скорее всего это эхо от динамика, глушим
            memset(mono_buf, 0, AUDIO_CHUNK_SIZE * sizeof(int16_t));
        }
#elif USE_VOX
        // Простой VOX: подавление передачи когда активен приём
        if (s_rx_active) {
            memset(mono_buf, 0, AUDIO_CHUNK_SIZE * sizeof(int16_t));
        }
#endif
        // USE_VOX=0 и USE_VAD=0: всегда передаём (для тестирования)

        // ===== Проверка режима калибровки =====
        if (s_calibration_active && s_calib_mic_buf) {
            // Ждём пока RX воспроизведёт щелчок (s_calib_click_done)
            // Не начинаем запись пока щелчок не отыграл
            if (!s_calib_click_done) {
                // Пропускаем этот chunk - щелчок ещё не воспроизведён
                continue;
            }
            
            // Копируем данные микрофона в калибровочный буфер вместо отправки
            int to_copy = AUDIO_CHUNK_SIZE;
            if (s_calib_mic_pos + to_copy > s_calib_mic_max) {
                to_copy = s_calib_mic_max - s_calib_mic_pos;
            }
            if (to_copy > 0) {
                memcpy(&s_calib_mic_buf[s_calib_mic_pos], mono_buf, to_copy * sizeof(int16_t));
                s_calib_mic_pos += to_copy;
                
                // Лог первой записи (сбрасывается в начале калибровки)
                static int calib_rec_log = 0;
                if (s_calib_reset_log_counters) {
                    calib_rec_log = 0;
                    // Не сбрасываем флаг здесь - он сбросится в RX задаче
                }
                if (calib_rec_log < 5) {
                    ESP_LOGI(TAG, "CALIB TX: recording %d samples, pos=%d/%d", to_copy, s_calib_mic_pos, s_calib_mic_max);
                    calib_rec_log++;
                }
            }
            // Когда буфер заполнен - сигнализируем калибровке
            if (s_calib_mic_pos >= s_calib_mic_max && s_calib_mic_ready) {
                xSemaphoreGive(s_calib_mic_ready);
            }
            // Пропускаем сетевую отправку
            continue;
        }

        // End CPU time measurement for audio processing
        uint32_t t_end = xthal_get_ccount();
        uint32_t process_cycles = t_end - t_start;
        // Convert to microseconds (ESP32-S3 runs at 240MHz)
        uint32_t process_time_us = process_cycles / 240;
        process_ticks_total += process_time_us;
        if (process_time_us > process_ticks_max) process_ticks_max = process_time_us;
        process_count++;

        // Отправляем PCM или ADPCM в зависимости от настройки
        int sent;
        int expected_bytes;
        if (s_use_pcm) {
            // PCM: 512 байт на пакет (256 samples * 2 bytes)
            expected_bytes = AUDIO_CHUNK_SIZE * sizeof(int16_t);
            sent = sendto(s_udp_socket, mono_buf, expected_bytes, 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        } else {
            // ADPCM: 128 байт на пакет (256 samples * 4 bits / 8)
            expected_bytes = AUDIO_CHUNK_SIZE / 2;
            adpcm_encode_block(mono_buf, adpcm_buf, AUDIO_CHUNK_SIZE, &adpcm_enc_state);
            sent = sendto(s_udp_socket, adpcm_buf, expected_bytes, 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
        
        // Диагностика ошибок отправки
        if (sent < 0) {
            // ENOMEM (12) = буфер переполнен, просто пропускаем пакет
            if (errno != ENOMEM) {
                static uint32_t last_err_log = 0;
                if ((xTaskGetTickCount() - last_err_log) > pdMS_TO_TICKS(1000)) {
                    ESP_LOGE(TAG, "TX sendto failed: errno=%d (%s)", errno, strerror(errno));
                    last_err_log = xTaskGetTickCount();
                }
            }
            // При переполнении буфера - небольшая пауза
            vTaskDelay(1);
        } else if (sent != expected_bytes) {
            ESP_LOGW(TAG, "TX partial send: %d/%d bytes", sent, expected_bytes);
        }
        
        // Логирование раз в 500мс для диагностики
        uint32_t now = xTaskGetTickCount();
        if (tx_packets == 0) tx_start_time = now;
        tx_packets++;
        if ((now - tx_last_log) > pdMS_TO_TICKS(500)) {
            uint32_t elapsed_ms = (now - tx_start_time) * portTICK_PERIOD_MS;
            float pps = elapsed_ms > 0 ? (float)tx_packets * 1000.0f / elapsed_ms : 0;
            
            // Calculate CPU load: processing time vs available time
            // At 16kHz, 256 samples = 16ms available per frame
            uint32_t frame_time_us = (AUDIO_CHUNK_SIZE * 1000000UL) / AUDIO_SAMPLE_RATE;  // frame time in microseconds
            uint32_t avg_process_time = process_count > 0 ? process_ticks_total / process_count : 0;
            int task_load_pct = (int)((avg_process_time * 100) / frame_time_us);
            
            // Heap info для мониторинга памяти
            size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            
            ESP_LOGI(TAG, "TX: sent=%d, pkts=%lu, pps=%.1f | Task: avg=%luus max=%luus load=%d%% (frame=16ms) | Heap: %luKB PSRAM: %luKB", 
                     sent, (unsigned long)tx_packets, pps,
                     (unsigned long)avg_process_time, (unsigned long)process_ticks_max, task_load_pct,
                     (unsigned long)(free_heap/1024), (unsigned long)(free_psram/1024));
            
            // Reset CPU stats
            process_ticks_total = 0;
            process_ticks_max = 0;
            process_count = 0;
            
            tx_last_log = now;
        }
        
        // Даём шанс RX task (fair scheduling между TX и RX на CPU1)
        taskYIELD();
    }

#if USE_AFE_AEC
#if !USE_AFE_SYNC
    // Останавливаем AFE async (только для async режима)
    afe_async_stop();
#endif
    // Для sync режима ничего не нужно - AFE остаётся инициализированным
#endif
    
    // AGC и VAD НЕ освобождаем - они персистентные и переиспользуются
    
    // Буферы НЕ освобождаем - они статические и переиспользуются
    // free(tx_buf);
    // free(mono_buf);
    // free(adpcm_buf);
    
    ESP_LOGI(TAG, "Audio TX task stopped");
    vTaskDelete(NULL);
}

// Задача приёма и воспроизведения аудио (PCM с de-emphasis и джиттер-буфером)
static void audio_rx_task(void *arg)
{
    ESP_LOGI(TAG, "=== Audio RX task started (direct I2S) ===");
    ESP_LOGI(TAG, "RX: socket=%d, port=%d, waiting for packets...", s_udp_socket, UDP_AUDIO_PORT);
    
    // Проверяем сокет
    if (s_udp_socket < 0) {
        ESP_LOGE(TAG, "RX: Invalid socket! Cannot receive.");
        s_streaming_rx = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Получаем прямой доступ к I2S TX каналу (обход esp_codec_dev)
    i2s_chan_handle_t i2s_tx = bsp_audio_get_i2s_tx_channel();
    if (!i2s_tx) {
        ESP_LOGE(TAG, "Failed to get I2S TX channel!");
        s_streaming_rx = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Включаем канал TX
    i2s_channel_enable(i2s_tx);
    
    // Отключаем watchdog для этой задачи (игнорируем если не зарегистрирован)
    esp_err_t wdt_err = esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
    (void)wdt_err;  // Игнорируем результат
    
    // Размер чанка воспроизведения - такой же как TX для синхронизации
    #define PLAYBACK_CHUNK_MONO AUDIO_CHUNK_SIZE
    
    // Джиттер-буфер: небольшой для экономии памяти (8 пакетов = 4096 сэмплов = 8KB)
    const int jitter_total_samples = AUDIO_CHUNK_SIZE * 8;
    
    // Используем обычную память - SPIRAM может быть недоступен
    int16_t *jitter_buf = (int16_t *)heap_caps_malloc(jitter_total_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *rx_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    
    // Stereo буфер для воспроизведения - пробуем DMA, fallback на DEFAULT
    int16_t *stereo_buf = (int16_t *)heap_caps_malloc(PLAYBACK_CHUNK_MONO * 2 * sizeof(int16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!stereo_buf) {
        stereo_buf = (int16_t *)heap_caps_malloc(PLAYBACK_CHUNK_MONO * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    }
    
    // Буфер для декодирования ADPCM
    uint8_t *adpcm_rx_buf = (uint8_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE / 2, MALLOC_CAP_DEFAULT);
    int16_t *decoded_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    
    ESP_LOGI(TAG, "RX buffers: jitter=%p (%d samples, %d bytes), rx=%p, stereo=%p, adpcm=%p", 
             jitter_buf, jitter_total_samples, jitter_total_samples * (int)sizeof(int16_t), rx_buf, stereo_buf, adpcm_rx_buf);
    
    if (!jitter_buf || !rx_buf || !stereo_buf || !adpcm_rx_buf || !decoded_buf) {
        ESP_LOGE(TAG, "Failed to allocate RX buffers");
        if (jitter_buf) free(jitter_buf);
        if (rx_buf) free(rx_buf);
        if (stereo_buf) free(stereo_buf);
        if (adpcm_rx_buf) free(adpcm_rx_buf);
        if (decoded_buf) free(decoded_buf);
        s_streaming_rx = false;
        vTaskDelete(NULL);
        return;
    }
    
    // ADPCM state для декодирования
    adpcm_state_t adpcm_dec_state = {0, 0};
    
    // Джиттер-буфер состояние
    int jb_write_pos = 0;
    int jb_read_pos = 0;
    int jb_buffered_samples = 0;
    bool jb_playing = false;  // Начинаем воспроизведение после накопления

    struct sockaddr_in src_addr;
    socklen_t addr_len;
    uint32_t rx_packets = 0;
    uint32_t last_log_time = 0;
    uint32_t status_log_time = 0;
    
    // Делаем сокет НЕБЛОКИРУЮЩИМ - критично для приёма всех пакетов!
    int flags = fcntl(s_udp_socket, F_GETFL, 0);
    fcntl(s_udp_socket, F_SETFL, flags | O_NONBLOCK);
    
    // Увеличиваем UDP receive buffer для буферизации пакетов
    int rcvbuf = 65536;  // 64KB - поместится ~270 пакетов по 240 байт
    setsockopt(s_udp_socket, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // Флаг для отслеживания перехода в режим калибровки
    bool was_calibrating = false;
    
    while (s_streaming_rx) {
        // ===== Режим калибровки: воспроизводим щелчки вместо сетевого аудио =====
        if (s_calibration_active && s_calib_spk_buf) {
            // При ВХОДЕ в режим калибровки - очищаем джиттер-буфер
            if (!was_calibrating) {
                was_calibrating = true;
                jb_buffered_samples = 0;
                jb_write_pos = 0;
                jb_read_pos = 0;
                jb_playing = false;
                ESP_LOGI(TAG, "CALIB RX: entered calibration mode, cleared jitter buffer");
            }
            
            // Воспроизводим калибровочный буфер
            int to_play = AUDIO_CHUNK_SIZE;
            if (s_calib_spk_pos + to_play > s_calib_spk_samples) {
                to_play = s_calib_spk_samples - s_calib_spk_pos;
            }
            
            if (to_play > 0) {
                // Моно→стерео с МАКСИМАЛЬНОЙ громкостью
                for (int i = 0; i < to_play; i++) {
                    stereo_buf[i * 2] = s_calib_spk_buf[s_calib_spk_pos + i];
                    stereo_buf[i * 2 + 1] = s_calib_spk_buf[s_calib_spk_pos + i];
                }
                s_calib_spk_pos += to_play;
                
                // Лог первого воспроизведения щелчка (сбрасывается в начале калибровки)
                static int calib_play_log = 0;
                if (s_calib_reset_log_counters) {
                    calib_play_log = 0;
                    s_calib_reset_log_counters = false;
                }
                if (calib_play_log < 10) {
                    ESP_LOGI(TAG, "CALIB RX: playing %d samples, pos=%d/%d", to_play, s_calib_spk_pos, s_calib_spk_samples);
                    calib_play_log++;
                }
                
                size_t bytes_written = 0;
                i2s_channel_write(i2s_tx, stereo_buf, to_play * 2 * sizeof(int16_t), &bytes_written, pdMS_TO_TICKS(50));
                
                // Проверяем - щелчок был на позициях silence_samples...silence_samples+click_samples
                // Когда воспроизвели щелчок - сигнализируем TX что можно начинать запись
                if (s_calib_spk_pos >= s_calib_spk_samples && s_calib_click_playing) {
                    s_calib_click_done = true;
                    s_calib_click_playing = false;
                    ESP_LOGI(TAG, "CALIB RX: click fully played, TX can start recording");
                }
            } else {
                // Буфер воспроизведён - играем тишину пока калибровка активна
                // И сигнализируем что щелчок воспроизведён
                if (s_calib_click_playing) {
                    s_calib_click_done = true;
                    s_calib_click_playing = false;
                    ESP_LOGI(TAG, "CALIB RX: buffer done, TX can start recording");
                }
                memset(stereo_buf, 0, AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t));
                size_t bytes_written = 0;
                i2s_channel_write(i2s_tx, stereo_buf, AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t), &bytes_written, pdMS_TO_TICKS(50));
            }
            
            // Пропускаем обработку сетевых пакетов - калибровка в приоритете
            continue;
        } else {
            // Вышли из режима калибровки - сбрасываем флаг
            if (was_calibrating) {
                was_calibrating = false;
                ESP_LOGI(TAG, "CALIB RX: exited calibration mode");
            }
        }
        
        // Периодический статус лог (каждые 10 секунд)
        uint32_t now = xTaskGetTickCount();
        if ((now - status_log_time) > pdMS_TO_TICKS(10000)) {
            ESP_LOGI(TAG, "RX status: listening on port %d, packets=%lu, active=%d", 
                     UDP_AUDIO_PORT, (unsigned long)rx_packets, s_rx_active);
            status_log_time = now;
        }
        
        // ПРИОРИТЕТ 1: Читаем ВСЕ доступные пакеты (non-blocking socket)
        int packets_received = 0;
        for (int max_reads = 50; max_reads > 0; max_reads--) {
            if (jb_buffered_samples >= jitter_total_samples - AUDIO_CHUNK_SIZE) break;
            
            addr_len = sizeof(src_addr);
            // Читаем в больший буфер (PCM), ADPCM поместится тоже
            int len = recvfrom(s_udp_socket, rx_buf, AUDIO_CHUNK_SIZE * sizeof(int16_t), 0,
                              (struct sockaddr *)&src_addr, &addr_len);
            
            if (len <= 0) break;  // EAGAIN/EWOULDBLOCK - нет пакетов
            
            int samples;
            int16_t *src_samples;
            
            // Автодетект: PCM = 1024 байт (512*2), ADPCM = 256 байт (512/2) для 512 samples
            if (len == AUDIO_CHUNK_SIZE * (int)sizeof(int16_t)) {
                // PCM пакет - используем напрямую
                samples = len / sizeof(int16_t);
                src_samples = rx_buf;
            } else if (len == AUDIO_CHUNK_SIZE / 2) {
                // ADPCM пакет - декодируем
                memcpy(adpcm_rx_buf, rx_buf, len);
                adpcm_decode_block(adpcm_rx_buf, decoded_buf, AUDIO_CHUNK_SIZE, &adpcm_dec_state);
                samples = AUDIO_CHUNK_SIZE;
                src_samples = decoded_buf;
            } else {
                // Неизвестный размер - логируем и пропускаем
                static uint32_t last_unknown_log = 0;
                if (xTaskGetTickCount() - last_unknown_log > pdMS_TO_TICKS(2000)) {
                    ESP_LOGW(TAG, "RX: unknown packet size %d (expected PCM=%d or ADPCM=%d)", 
                             len, AUDIO_CHUNK_SIZE * (int)sizeof(int16_t), AUDIO_CHUNK_SIZE / 2);
                    last_unknown_log = xTaskGetTickCount();
                }
                continue;
            }
            
            rx_packets++;
            packets_received++;
            
            s_rx_last_packet_time = xTaskGetTickCount();
            s_rx_active = true;
            
            // Копируем в джиттер-буфер (кольцевой)
            for (int i = 0; i < samples && jb_buffered_samples < jitter_total_samples; i++) {
                jitter_buf[jb_write_pos] = src_samples[i];
                jb_write_pos = (jb_write_pos + 1) % jitter_total_samples;
                jb_buffered_samples++;
            }
        }
        
        // Логируем если получили пакеты
        now = xTaskGetTickCount();
        if (packets_received > 0 && (rx_packets <= 5 || (now - last_log_time) > pdMS_TO_TICKS(2000))) {
            ESP_LOGI(TAG, "RX: %d pkts (total=%lu), buffered=%d, playing=%d", 
                     packets_received, (unsigned long)rx_packets, jb_buffered_samples, jb_playing);
            last_log_time = now;
        }
        
        // Начинаем воспроизведение после накопления достаточного буфера
        if (!jb_playing && jb_buffered_samples >= AUDIO_CHUNK_SIZE * JITTER_BUFFER_PACKETS) {
            jb_playing = true;
            ESP_LOGI(TAG, "Jitter buffer ready (%d samples), starting playback", jb_buffered_samples);
        }
        
        // Проверка таймаута (500мс без пакетов = конец передачи)
        if (s_rx_active && packets_received == 0 && (xTaskGetTickCount() - s_rx_last_packet_time) > pdMS_TO_TICKS(500)) {
            s_rx_active = false;
            jb_playing = false;
            jb_buffered_samples = 0;
            jb_write_pos = 0;
            jb_read_pos = 0;
#if USE_ESP_AEC
            // Сбрасываем AEC для очистки накопленных искажений
            esp_aec_reset();
#endif
            ESP_LOGI(TAG, "RX timeout, reset jitter buffer and AEC");
        }
        
        // FULL-DUPLEX: TX и RX работают параллельно
        // TX использует I2S RX (микрофон), RX использует I2S TX (динамик) - разные каналы
        
        // ПРИОРИТЕТ 2: Воспроизводим только если нет пакетов для чтения
        if (jb_playing && jb_buffered_samples >= AUDIO_CHUNK_SIZE) {
            // Маленький чанк для быстрого возврата к приёму пакетов
            int play_chunk = AUDIO_CHUNK_SIZE;  // 120 samples = ~7.5ms при 16kHz
            
            // Читаем чанк из джиттер-буфера (моно)
            int16_t ref_mono[AUDIO_CHUNK_SIZE];
            for (int i = 0; i < play_chunk; i++) {
                ref_mono[i] = jitter_buf[jb_read_pos];
                jb_read_pos = (jb_read_pos + 1) % jitter_total_samples;
            }
            jb_buffered_samples -= play_chunk;
            
            // Моно→стерео (без усиления для чистого звука)
            for (int i = 0; i < play_chunk; i++) {
                stereo_buf[i * 2] = ref_mono[i];
                stereo_buf[i * 2 + 1] = ref_mono[i];
            }
            
            // ВАЖНО: Добавляем reference ДО записи в I2S
            // Это гарантирует что AEC получит данные даже если I2S задержится
#if USE_AFE_AEC
            afe_add_reference_samples(ref_mono, play_chunk);
#endif

#if USE_ESP_AEC
            esp_aec_add_reference(ref_mono, play_chunk);
#endif
            
            // I2S write - блокирующий чтобы гарантировать воспроизведение
            size_t bytes_written = 0;
            esp_err_t wr_ret = i2s_channel_write(i2s_tx, stereo_buf, play_chunk * 2 * sizeof(int16_t), &bytes_written, pdMS_TO_TICKS(50));
            
            // Диагностика первых нескольких записей
            static int write_log_count = 0;
            if (write_log_count < 5) {
                ESP_LOGI(TAG, "I2S write: ret=%d, written=%d/%d bytes", 
                         wr_ret, (int)bytes_written, play_chunk * 2 * (int)sizeof(int16_t));
                write_log_count++;
            }
            
            if (wr_ret != ESP_OK || bytes_written < play_chunk * 2 * sizeof(int16_t)) {
                // Не всё записалось - возвращаем незаписанные данные
                int samples_written = bytes_written / (2 * sizeof(int16_t));
                int samples_not_written = play_chunk - samples_written;
                jb_read_pos = (jb_read_pos - samples_not_written + jitter_total_samples) % jitter_total_samples;
                jb_buffered_samples += samples_not_written;
            }
        } else if (packets_received == 0) {
            // Нет пакетов и нечего воспроизводить - даём время другим задачам
            taskYIELD();  // Отдаём CPU даже без delay для round-robin с TX task
            vTaskDelay(1);
        } else {
            // Были пакеты - всё равно даём шанс TX task
            taskYIELD();
        }
    }

    s_rx_active = false;
    free(jitter_buf);
    free(rx_buf);
    free(stereo_buf);
    free(adpcm_rx_buf);
    free(decoded_buf);
    ESP_LOGI(TAG, "Audio RX task stopped");
    vTaskDelete(NULL);
}

// Запуск/остановка передачи
// Задача для асинхронного mDNS resolve и запуска TX
static void mdns_resolve_and_tx_task(void *arg)
{
    (void)arg;
    esp_ip4_addr_t resolved_ip;
    ESP_LOGI(TAG, "Async resolving %s via mDNS...", s_target_host);
    esp_err_t err = mdns_query_a(s_target_host, 1000, &resolved_ip);  // 1 сек таймаут
    if (err == ESP_OK) {
        snprintf(s_target_ip, sizeof(s_target_ip), IPSTR, IP2STR(&resolved_ip));
        ESP_LOGI(TAG, "Resolved %s -> %s", s_target_host, s_target_ip);
    } else {
        ESP_LOGE(TAG, "mDNS resolve failed for %s!", s_target_host);
        vTaskDelete(NULL);
        return;
    }
    
    // Теперь запускаем TX (IP уже в кэше)
    stream_tx_start();
    vTaskDelete(NULL);
}

static void stream_tx_start(void)
{
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected!");
        return;
    }
    
    if (s_udp_socket < 0) {
        udp_init();
    }

    // Если target задан, но IP не в кэше - запускаем асинхронный resolve
    if (s_target_host[0] != '\0' && s_target_ip[0] == '\0') {
        ESP_LOGI(TAG, "Starting async mDNS resolve for %s...", s_target_host);
        xTaskCreate(mdns_resolve_and_tx_task, "mdns_tx", 4096, NULL, 5, NULL);
        return;  // TX запустится после resolve
    }

    if (!s_streaming_tx) {
        s_streaming_tx = true;
        
#if USE_ESP_AEC
        // Сбрасываем AEC при каждом старте TX для чистого начала
        esp_aec_reset();
#endif
        
        // Выделяем стек в PSRAM если ещё не выделен
        if (!s_tx_task_stack) {
            s_tx_task_stack = heap_caps_malloc(TX_TASK_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
            if (!s_tx_task_stack) {
                ESP_LOGE(TAG, "Failed to allocate TX task stack in PSRAM!");
                s_streaming_tx = false;
                return;
            }
            ESP_LOGI(TAG, "TX task stack allocated in PSRAM: %d KB", TX_TASK_STACK_SIZE * 4 / 1024);
        }
        
        // CPU1 полностью под аудио, МАКСИМАЛЬНЫЙ приоритет
        // Используем статический стек в PSRAM (32KB)
        s_tx_task_handle = xTaskCreateStaticPinnedToCore(
            audio_tx_task, 
            "audio_tx", 
            TX_TASK_STACK_SIZE,
            NULL, 
            configMAX_PRIORITIES - 1, 
            s_tx_task_stack,
            &s_tx_task_tcb,
            1);
        if (!s_tx_task_handle) {
            ESP_LOGE(TAG, "Failed to create audio_tx task!");
            s_streaming_tx = false;
            return;
        }
        ESP_LOGI(TAG, "Started audio TX on CPU1, priority=%d (MAX), stack=32KB (PSRAM)", configMAX_PRIORITIES - 1);
    }
}

static void stream_tx_stop(void)
{
    if (s_streaming_tx) {
        s_streaming_tx = false;
        // Даём задаче время завершиться
        vTaskDelay(pdMS_TO_TICKS(50));
        s_tx_task_handle = NULL;

#if USE_ESP_AEC
        // Сбрасываем ESP-SR AEC при остановке TX
        esp_aec_reset();
#endif
        
#if USE_AFE_AEC
        // Сбрасываем ring buffer AFE reference
        s_afe_ref_write_pos = 0;
        s_afe_ref_read_pos = 0;
#endif
        
        ESP_LOGI(TAG, "Stopped audio TX, buffers cleared");
    }
}

// Запуск/остановка приёма
static void stream_rx_start(void)
{
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected!");
        return;
    }
    
    if (s_udp_socket < 0) {
        udp_init();
    }
    
#if USE_AFE_AEC
    // ESP-SR AFE для подавления эха
    afe_init();
#endif

    if (!s_streaming_rx) {
        s_streaming_rx = true;
        // CPU1 полностью под аудио, МАКСИМАЛЬНЫЙ приоритет
        xTaskCreatePinnedToCore(audio_rx_task, "audio_rx", 8192, NULL, configMAX_PRIORITIES - 1, &s_rx_task_handle, 1);
        ESP_LOGI(TAG, "Started audio RX on CPU1, socket=%d, port=%d, priority=%d (MAX)", s_udp_socket, UDP_AUDIO_PORT, configMAX_PRIORITIES - 1);
    } else {
        ESP_LOGI(TAG, "Audio RX already running");
    }
}

static void stream_rx_stop(void)
{
    if (s_streaming_rx) {
        s_streaming_rx = false;
        vTaskDelay(pdMS_TO_TICKS(50));
        s_rx_task_handle = NULL;
        s_rx_active = false;
        ESP_LOGI(TAG, "Stopped audio RX");
    }
}

static void audio_task(void *arg)
{
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = AUDIO_SAMPLE_RATE,  // 16kHz для ESP-SR AFE AEC
        .channel = 1,
        .bits_per_sample = 16,
    };

    s_spk_dev = bsp_audio_codec_speaker_init();
    s_mic_dev = bsp_audio_codec_microphone_init();

    if (!s_spk_dev || !s_mic_dev) {
        ESP_LOGE(TAG, "Audio codec init failed: spk=%p, mic=%p", s_spk_dev, s_mic_dev);
        vTaskDelete(NULL);
        return;
    }

    esp_codec_dev_set_out_vol(s_spk_dev, 100);  // Громкость 100%
    esp_codec_dev_set_in_gain(s_mic_dev, 80.0f);  // Усиление микрофона 80%

    // PA управляется автоматически через BSP (pa_reverted = true для active low)
    ESP_LOGI(TAG, "PA control handled by BSP (active low)");

    int ret_spk = esp_codec_dev_open(s_spk_dev, &fs);
    int ret_mic = esp_codec_dev_open(s_mic_dev, &fs);
    ESP_LOGI(TAG, "Codec open: spk=%d, mic=%d", ret_spk, ret_mic);

    ESP_LOGI(TAG, "Audio task started");

    // Тестовый beep при старте
    ESP_LOGI(TAG, "Playing test beep...");
    audio_beep_play();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Стерео буфер для чтения (I2S работает в стерео режиме)
    const size_t stereo_samples = 256;  // Стерео пары
    int16_t *mic_buf = (int16_t *)heap_caps_malloc(stereo_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!mic_buf) {
        ESP_LOGE(TAG, "Failed to allocate mic buffer");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        if (s_beep_request) {
            s_beep_request = false;
            audio_beep_play();
        }

        if (s_record_request) {
            s_record_request = false;
            audio_record_and_play();
        }

        // Пропускаем чтение микрофона когда TX активен (TX сам читает)
        if (s_streaming_tx) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Читаем стерео данные
        int ret = esp_codec_dev_read(s_mic_dev, mic_buf, stereo_samples * 2 * sizeof(int16_t));
        if (ret >= 0) {
            int stereo_pairs = (ret > 0) ? (ret / (2 * sizeof(int16_t))) : stereo_samples;
            uint64_t sum = 0;
            int16_t max_val = 0;
            int16_t max_left = 0, max_right = 0;
            // Берём максимум из обоих каналов
            for (int i = 0; i < stereo_pairs; ++i) {
                int32_t left = mic_buf[i * 2];      // Left channel
                int32_t right = mic_buf[i * 2 + 1]; // Right channel
                // Берём большее значение
                int32_t v = (abs(left) > abs(right)) ? left : right;
                sum += (uint64_t)(v * v);
                int16_t abs_v = (v < 0) ? -v : v;
                if (abs_v > max_val) max_val = abs_v;
                if (abs(left) > max_left) max_left = abs(left);
                if (abs(right) > max_right) max_right = abs(right);
            }
            float rms = sqrtf((float)sum / (float)stereo_pairs) / 32767.0f;
            int level = (int)(rms * 100.0f);
            if (level > 100) {
                level = 100;
            }
            s_mic_level = level;
            
            // Логирование для отладки
            static int log_cnt = 0;
            if (++log_cnt >= 50) {
                log_cnt = 0;
                ESP_LOGI(TAG, "Mic: L=%d R=%d max=%d level=%d%%", max_left, max_right, max_val, level);
                // mDNS browse работает пассивно - IP будет получен через callback
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============== Wi-Fi Functions ===============

// Сохранение Wi-Fi настроек в NVS
static esp_err_t wifi_save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(nvs_handle, NVS_KEY_SSID, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_set_str(nvs_handle, NVS_KEY_PASSWORD, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Wi-Fi credentials saved to NVS");
    }

    nvs_close(nvs_handle);
    return err;
}

// Загрузка Wi-Fi настроек из NVS
static esp_err_t wifi_load_credentials(char *ssid, size_t ssid_len, char *password, size_t pwd_len)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved Wi-Fi credentials found");
        return err;
    }

    size_t len = ssid_len;
    err = nvs_get_str(nvs_handle, NVS_KEY_SSID, ssid, &len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved SSID found");
        nvs_close(nvs_handle);
        return err;
    }

    len = pwd_len;
    err = nvs_get_str(nvs_handle, NVS_KEY_PASSWORD, password, &len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved password found");
        nvs_close(nvs_handle);
        return err;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded Wi-Fi credentials for: %s", ssid);
    return ESP_OK;
}

// Сохранение целевого hostname и room_name в NVS
static esp_err_t save_target_host(const char *host, const char *room)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(nvs_handle, NVS_KEY_TARGET_HOST, host);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Target host saved: %s", host);
    }
    
    if (room && room[0] != '\0') {
        err = nvs_set_str(nvs_handle, NVS_KEY_ROOM_NAME, room);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Room name saved: %s", room);
        }
    }
    
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// Загрузка целевого hostname и room_name из NVS
static esp_err_t load_target_host(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t len = sizeof(s_target_host);
    err = nvs_get_str(nvs_handle, NVS_KEY_TARGET_HOST, s_target_host, &len);
    if (err == ESP_OK && s_target_host[0] != '\0') {
        ESP_LOGI(TAG, "Loaded target host: %s", s_target_host);
    }
    
    len = sizeof(s_room_name);
    err = nvs_get_str(nvs_handle, NVS_KEY_ROOM_NAME, s_room_name, &len);
    if (err == ESP_OK && s_room_name[0] != '\0') {
        ESP_LOGI(TAG, "Loaded room name: %s", s_room_name);
    }

    nvs_close(nvs_handle);
    return err;
}

// Сохранение настроек аудио в NVS
static esp_err_t save_audio_settings(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for audio settings: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_u8(nvs_handle, NVS_KEY_AECM_ENABLED, s_aecm_enabled ? 1 : 0);
    nvs_set_i16(nvs_handle, NVS_KEY_AECM_DELAY, (int16_t)s_aecm_delay_ms);
    nvs_set_u8(nvs_handle, NVS_KEY_AECM_MODE, (uint8_t)s_aecm_mode);
    nvs_set_u8(nvs_handle, NVS_KEY_AGC_ENABLED, s_agc_enabled ? 1 : 0);
    nvs_set_i16(nvs_handle, NVS_KEY_AGC_GAIN, (int16_t)s_agc_gain_db);
    nvs_set_i16(nvs_handle, NVS_KEY_NOISE_GATE, (int16_t)s_noise_gate_threshold);
    nvs_set_u8(nvs_handle, NVS_KEY_USE_PCM, s_use_pcm ? 1 : 0);
    
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Audio settings saved: AECM=%d/%dms/mode%d, AGC=%d/%ddB, noise=%d, pcm=%d",
             s_aecm_enabled, s_aecm_delay_ms, s_aecm_mode,
             s_agc_enabled, s_agc_gain_db, s_noise_gate_threshold, s_use_pcm);
    return ESP_OK;
}

// Загрузка настроек аудио из NVS
static esp_err_t load_audio_settings(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No audio settings in NVS, using defaults");
        return err;
    }

    uint8_t u8_val;
    int16_t i16_val;
    
    if (nvs_get_u8(nvs_handle, NVS_KEY_AECM_ENABLED, &u8_val) == ESP_OK) {
        s_aecm_enabled = (u8_val != 0);
    }
    if (nvs_get_i16(nvs_handle, NVS_KEY_AECM_DELAY, &i16_val) == ESP_OK) {
        s_aecm_delay_ms = i16_val;
    }
    if (nvs_get_u8(nvs_handle, NVS_KEY_AECM_MODE, &u8_val) == ESP_OK) {
        s_aecm_mode = u8_val;
    }
    if (nvs_get_u8(nvs_handle, NVS_KEY_AGC_ENABLED, &u8_val) == ESP_OK) {
        s_agc_enabled = (u8_val != 0);
    }
    if (nvs_get_i16(nvs_handle, NVS_KEY_AGC_GAIN, &i16_val) == ESP_OK) {
        s_agc_gain_db = i16_val;
    }
    if (nvs_get_i16(nvs_handle, NVS_KEY_NOISE_GATE, &i16_val) == ESP_OK) {
        s_noise_gate_threshold = i16_val;
    }
    if (nvs_get_u8(nvs_handle, NVS_KEY_USE_PCM, &u8_val) == ESP_OK) {
        s_use_pcm = (u8_val != 0);
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Audio settings loaded: AECM=%d/%dms/mode%d, AGC=%d/%ddB, noise=%d, pcm=%d",
             s_aecm_enabled, s_aecm_delay_ms, s_aecm_mode,
             s_agc_enabled, s_agc_gain_db, s_noise_gate_threshold, s_use_pcm);
    return ESP_OK;
}

// Генерация hostname из MAC адреса
static void generate_hostname(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_my_hostname, sizeof(s_my_hostname), "esp-%02X%02X%02X", mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "My hostname: %s", s_my_hostname);
}

// Сканирование mDNS устройств
static void mdns_scan_devices(void)
{
    if (s_mdns_scanning) return;
    s_mdns_scanning = true;
    s_mdns_scan_done = false;
    
    ESP_LOGI(TAG, "Scanning for ESP audio devices...");
    
    // Очищаем список
    s_discovered_count = 0;
    for (int i = 0; i < MAX_DISCOVERED_DEVICES; i++) {
        s_discovered_devices[i].valid = false;
    }
    
    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr(MDNS_SERVICE_TYPE, MDNS_SERVICE_PROTO, 3000, MAX_DISCOVERED_DEVICES, &results);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        s_mdns_scanning = false;
        s_mdns_scan_done = true;
        return;
    }
    
    if (!results) {
        ESP_LOGI(TAG, "No devices found");
        s_mdns_scanning = false;
        s_mdns_scan_done = true;
        return;
    }
    
    mdns_result_t *r = results;
    while (r && s_discovered_count < MAX_DISCOVERED_DEVICES) {
        // Пропускаем своё устройство
        if (r->hostname && strcmp(r->hostname, s_my_hostname) != 0) {
            strncpy(s_discovered_devices[s_discovered_count].hostname, r->hostname, sizeof(s_discovered_devices[0].hostname) - 1);
            
            // Получаем IP
            if (r->addr) {
                snprintf(s_discovered_devices[s_discovered_count].ip, sizeof(s_discovered_devices[0].ip),
                         IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
            } else {
                s_discovered_devices[s_discovered_count].ip[0] = '\0';
            }
            
            s_discovered_devices[s_discovered_count].valid = true;
            ESP_LOGI(TAG, "Found device: %s (%s)", 
                     s_discovered_devices[s_discovered_count].hostname,
                     s_discovered_devices[s_discovered_count].ip);
            s_discovered_count++;
        }
        r = r->next;
    }
    
    mdns_query_results_free(results);
    ESP_LOGI(TAG, "Found %d devices", s_discovered_count);
    
    s_mdns_scanning = false;
    s_mdns_scan_done = true;
}

// Задача периодического сканирования
static void mdns_scan_task(void *arg)
{
    (void)arg;
    mdns_scan_devices();
    vTaskDelete(NULL);
}

static void mdns_scan_start(void)
{
    if (s_mdns_scanning) return;
    xTaskCreate(mdns_scan_task, "mdns_scan", 4096, NULL, 5, NULL);
}

// mDNS Browse callback - вызывается при обнаружении нового устройства
static void mdns_browse_notify_cb(mdns_search_once_t *search)
{
    mdns_result_t *results = NULL;
    
    // Получаем результаты browse
    if (mdns_query_async_get_results(search, 0, &results, NULL) != ESP_OK || !results) {
        return;
    }
    
    mdns_result_t *r = results;
    while (r) {
        // Пропускаем своё устройство
        if (r->hostname && strcmp(r->hostname, s_my_hostname) != 0) {
            ESP_LOGI(TAG, "mDNS Browse found: %s", r->hostname);
            
            // Если это наш target host и IP ещё не известен
            if (s_target_host[0] != '\0' && s_target_ip[0] == '\0') {
                // Сравниваем hostname (без .local)
                if (strncmp(r->hostname, s_target_host, strlen(s_target_host)) == 0) {
                    if (r->addr) {
                        snprintf(s_target_ip, sizeof(s_target_ip), IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
                        ESP_LOGI(TAG, "mDNS Browse resolved target: %s -> %s", s_target_host, s_target_ip);
                    }
                }
            }
            
            // Добавляем в список обнаруженных устройств
            bool found = false;
            for (int i = 0; i < s_discovered_count; i++) {
                if (strcmp(s_discovered_devices[i].hostname, r->hostname) == 0) {
                    found = true;
                    // Обновляем IP если изменился
                    if (r->addr) {
                        snprintf(s_discovered_devices[i].ip, sizeof(s_discovered_devices[0].ip),
                                 IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
                    }
                    break;
                }
            }
            
            if (!found && s_discovered_count < MAX_DISCOVERED_DEVICES) {
                strncpy(s_discovered_devices[s_discovered_count].hostname, r->hostname, 
                        sizeof(s_discovered_devices[0].hostname) - 1);
                if (r->addr) {
                    snprintf(s_discovered_devices[s_discovered_count].ip, sizeof(s_discovered_devices[0].ip),
                             IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
                }
                s_discovered_devices[s_discovered_count].valid = true;
                s_discovered_count++;
            }
        }
        r = r->next;
    }
    
    mdns_query_results_free(results);
}

// Запуск mDNS browse для пассивного обнаружения устройств
static void mdns_start_browse(void)
{
    if (s_mdns_browse_handle) {
        // Уже запущен
        return;
    }
    
    // Запускаем асинхронный browse с callback
    // Timeout 0 = бесконечно, max_results 0 = неограничено
    s_mdns_browse_handle = mdns_query_async_new(NULL, MDNS_SERVICE_TYPE, MDNS_SERVICE_PROTO, 
                                                  MDNS_TYPE_PTR, 0, 0, mdns_browse_notify_cb);
    if (s_mdns_browse_handle) {
        ESP_LOGI(TAG, "mDNS browse started for %s.%s", MDNS_SERVICE_TYPE, MDNS_SERVICE_PROTO);
    } else {
        ESP_LOGE(TAG, "Failed to start mDNS browse!");
    }
}

// Остановка mDNS browse
static void mdns_stop_browse(void)
{
    if (s_mdns_browse_handle) {
        mdns_query_async_delete(s_mdns_browse_handle);
        s_mdns_browse_handle = NULL;
        ESP_LOGI(TAG, "mDNS browse stopped");
    }
}

// Инициализация mDNS
static void mdns_init_service(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(err));
        return;
    }

    mdns_hostname_set(s_my_hostname);
    mdns_instance_name_set("ESP Audio Device");
    
    // Регистрируем сервис для обнаружения
    mdns_service_add(NULL, "_esp-audio", "_udp", UDP_AUDIO_PORT, NULL, 0);
    
    // Запускаем пассивный browse для обнаружения других устройств
    mdns_start_browse();
    
    ESP_LOGI(TAG, "mDNS initialized: %s.local (browse active)", s_my_hostname);
}

// Задача автоподключения (выполняется в фоне после инициализации)
static void wifi_auto_connect_task(void *arg)
{
    (void)arg;
    char ssid[33] = {0};
    char password[65] = {0};

    if (wifi_load_credentials(ssid, sizeof(ssid), password, sizeof(password)) != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }
    
    if (strlen(ssid) == 0) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Auto-connecting to saved Wi-Fi: %s", ssid);
    
    // Сканируем сети чтобы найти AP с лучшим сигналом
    wifi_scan_config_t scan_config = {
        .ssid = (uint8_t *)ssid,  // Сканируем только нужный SSID
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Scan failed (%s), connecting without BSSID selection", esp_err_to_name(err));
        // Если сканирование не удалось - подключаемся как раньше
        wifi_config_t wifi_config = {0};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
        strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_connect();
        vTaskDelete(NULL);
        return;
    }
    
    // Получаем результаты сканирования
    uint16_t ap_count = MAX_SCAN_RESULTS;
    wifi_ap_record_t ap_records[MAX_SCAN_RESULTS];
    err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    
    if (err != ESP_OK || ap_count == 0) {
        ESP_LOGW(TAG, "No APs found for SSID: %s", ssid);
        // Пробуем подключиться всё равно
        wifi_config_t wifi_config = {0};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
        strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_connect();
        vTaskDelete(NULL);
        return;
    }
    
    // Находим AP с лучшим сигналом
    int best_idx = 0;
    int8_t best_rssi = -127;
    for (int i = 0; i < ap_count; i++) {
        if (strcmp((char *)ap_records[i].ssid, ssid) == 0) {
            ESP_LOGI(TAG, "  Found AP: %s BSSID=%02x:%02x:%02x:%02x:%02x:%02x RSSI=%d",
                     ap_records[i].ssid,
                     ap_records[i].bssid[0], ap_records[i].bssid[1], ap_records[i].bssid[2],
                     ap_records[i].bssid[3], ap_records[i].bssid[4], ap_records[i].bssid[5],
                     ap_records[i].rssi);
            if (ap_records[i].rssi > best_rssi) {
                best_rssi = ap_records[i].rssi;
                best_idx = i;
            }
        }
    }
    
    ESP_LOGI(TAG, "Selected best AP: BSSID=%02x:%02x:%02x:%02x:%02x:%02x RSSI=%d",
             ap_records[best_idx].bssid[0], ap_records[best_idx].bssid[1], ap_records[best_idx].bssid[2],
             ap_records[best_idx].bssid[3], ap_records[best_idx].bssid[4], ap_records[best_idx].bssid[5],
             best_rssi);
    
    // Подключаемся к выбранному AP с указанием BSSID
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    memcpy(wifi_config.sta.bssid, ap_records[best_idx].bssid, 6);
    wifi_config.sta.bssid_set = true;  // Указываем ESP подключиться к конкретному BSSID
    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err == ESP_OK) {
        esp_wifi_connect();
    } else {
        ESP_LOGE(TAG, "Failed to set Wi-Fi config: %s", esp_err_to_name(err));
    }
    
    vTaskDelete(NULL);
}

// Запуск автоподключения в фоновой задаче
static void wifi_auto_connect(void)
{
    xTaskCreate(wifi_auto_connect_task, "wifi_auto", 4096, NULL, 5, NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi STA started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason: %d), retry count: %d", 
                 disconnected->reason, s_wifi_retry_count);
        
        // Бесконечные попытки переподключения с увеличивающейся задержкой
        s_wifi_retry_count++;
        int delay_ms = 1000;  // Базовая задержка 1 секунда
        if (s_wifi_retry_count > 10) {
            delay_ms = 5000;  // После 10 попыток - 5 секунд
        } else if (s_wifi_retry_count > 5) {
            delay_ms = 2000;  // После 5 попыток - 2 секунды
        }
        
        ESP_LOGI(TAG, "Reconnecting to Wi-Fi in %d ms...", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        esp_wifi_connect();
        
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        s_wifi_retry_count = 0;  // Сброс счетчика при успешном подключении
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Gratuitous ARP для обновления ARP таблиц на роутере и других устройствах
        // Это помогает при перезагрузке одного устройства пока другое передаёт
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            // Отправляем несколько GARP для надёжности
            for (int i = 0; i < 3; i++) {
                esp_netif_action_connected(netif, NULL, 0, NULL);  // Это триггерит GARP внутри lwIP
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            ESP_LOGI(TAG, "Gratuitous ARP sent");
        }
        
        // Инициализируем UDP сокет сразу
        if (s_udp_socket < 0) {
            udp_init();
            ESP_LOGI(TAG, "UDP socket initialized after WiFi connect: %d", s_udp_socket);
        }
        
        // ВАЖНО: Запускаем RX СРАЗУ после создания сокета, ДО mDNS resolve!
        // Иначе пакеты будут теряться пока идёт mDNS (до 10+ секунд)
        ESP_LOGI(TAG, "Starting audio RX after WiFi connect...");
        stream_rx_start();
        
        // Предварительный mDNS резолвинг для быстрого старта TX
        // Пробуем несколько раз, т.к. второе устройство может ещё не подключиться
        if (s_target_host[0] != '\0' && s_target_ip[0] == '\0') {
            esp_ip4_addr_t resolved_ip;
            ESP_LOGI(TAG, "Pre-resolving %s via mDNS...", s_target_host);
            
            esp_err_t err = ESP_ERR_NOT_FOUND;
            for (int attempt = 0; attempt < 5 && err != ESP_OK; attempt++) {
                if (attempt > 0) {
                    ESP_LOGI(TAG, "mDNS pre-resolve retry %d...", attempt + 1);
                    vTaskDelay(pdMS_TO_TICKS(2000));  // Ждём 2 сек между попытками
                }
                err = mdns_query_a(s_target_host, 2000, &resolved_ip);
            }
            
            if (err == ESP_OK) {
                snprintf(s_target_ip, sizeof(s_target_ip), IPSTR, IP2STR(&resolved_ip));
                ESP_LOGI(TAG, "Pre-resolved %s -> %s", s_target_host, s_target_ip);
            } else {
                ESP_LOGW(TAG, "Pre-resolve failed for %s (will retry in background)", s_target_host);
            }
        }
        
        // Сохраняем настройки в NVS после успешного подключения
        wifi_config_t wifi_cfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK) {
            wifi_save_credentials((const char *)wifi_cfg.sta.ssid, (const char *)wifi_cfg.sta.password);
        }
    }
}

static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                    &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler, NULL, &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialized");

    // Генерируем hostname из MAC
    generate_hostname();
    
    // Инициализируем mDNS
    mdns_init_service();

    // Загружаем целевой hostname из NVS
    load_target_host();

    // Автоподключение к сохранённой сети
    wifi_auto_connect();
}

static void wifi_scan_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting Wi-Fi scan...");

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scan start failed: %s", esp_err_to_name(err));
        s_ap_count = 0;
    } else {
        s_ap_count = MAX_SCAN_RESULTS;
        err = esp_wifi_scan_get_ap_records(&s_ap_count, s_ap_records);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi scan get records failed: %s", esp_err_to_name(err));
            s_ap_count = 0;
        }

        ESP_LOGI(TAG, "Found %d access points", s_ap_count);
        
        // Сортируем по RSSI (лучший сигнал сверху)
        for (int i = 0; i < s_ap_count - 1; i++) {
            for (int j = i + 1; j < s_ap_count; j++) {
                if (s_ap_records[j].rssi > s_ap_records[i].rssi) {
                    wifi_ap_record_t tmp = s_ap_records[i];
                    s_ap_records[i] = s_ap_records[j];
                    s_ap_records[j] = tmp;
                }
            }
        }
        
        for (int i = 0; i < s_ap_count; i++) {
            ESP_LOGI(TAG, "  %d: %s (RSSI: %d)", i + 1, s_ap_records[i].ssid, s_ap_records[i].rssi);
        }
    }

    s_wifi_scan_done = true;
    s_wifi_scanning = false;
    vTaskDelete(NULL);
}

static void wifi_scan_start(void)
{
    if (s_wifi_scanning) return;
    
    s_wifi_scanning = true;
    s_wifi_scan_done = false;
    s_ap_count = 0;
    
    xTaskCreate(wifi_scan_task, "wifi_scan", 4096, NULL, 5, NULL);
}

static void wifi_connect(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Connecting to %s...", ssid);

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Wi-Fi config: %s", esp_err_to_name(err));
        return;
    }
    
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect: %s", esp_err_to_name(err));
    }
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
// Forward declarations
static void wifi_list_cb(lv_event_t *e);
static void device_list_cb(lv_event_t *e);

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (!s_label) {
        return;
    }

    // Проверяем, завершилось ли сканирование Wi-Fi
    if (s_wifi_scan_done && s_wifi_list) {
        s_wifi_scan_done = false;
        
        // Очищаем список и заполняем результатами
        lv_obj_clean(s_wifi_list);

        if (s_ap_count == 0) {
            lv_list_add_text(s_wifi_list, "Сети не найдены");
        } else {
            for (int i = 0; i < s_ap_count; i++) {
                if (s_ap_records[i].ssid[0] != '\0') {  // Пропускаем пустые SSID
                    // Формируем текст с SSID и качеством сигнала
                    char wifi_item[48];
                    int rssi = s_ap_records[i].rssi;
                    int quality = (rssi >= -50) ? 100 : (rssi <= -100) ? 0 : 2 * (rssi + 100);
                    snprintf(wifi_item, sizeof(wifi_item), "%s (%d%%)", s_ap_records[i].ssid, quality);
                    lv_obj_t *btn = lv_list_add_button(s_wifi_list, LV_SYMBOL_WIFI, wifi_item);
                    lv_obj_add_event_cb(btn, wifi_list_cb, LV_EVENT_CLICKED, NULL);
                }
            }
        }
    }

    // Проверяем, завершилось ли сканирование mDNS устройств
    if (s_mdns_scan_done && s_device_list) {
        s_mdns_scan_done = false;
        
        lv_obj_clean(s_device_list);

        if (s_discovered_count == 0) {
            lv_list_add_text(s_device_list, "Устройства не найдены");
            lv_list_add_text(s_device_list, "(Другое устройство должно быть включено)");
        } else {
            for (int i = 0; i < s_discovered_count; i++) {
                if (s_discovered_devices[i].valid) {
                    // Формируем текст: hostname (IP)
                    char item_text[64];
                    if (s_discovered_devices[i].ip[0] != '\0') {
                        snprintf(item_text, sizeof(item_text), "%s", s_discovered_devices[i].hostname);
                    } else {
                        snprintf(item_text, sizeof(item_text), "%s", s_discovered_devices[i].hostname);
                    }
                    lv_obj_t *btn = lv_list_add_button(s_device_list, LV_SYMBOL_AUDIO, item_text);
                    // Сохраняем индекс устройства как user_data
                    lv_obj_set_user_data(btn, (void*)(intptr_t)i);
                    lv_obj_add_event_cb(btn, device_list_cb, LV_EVENT_CLICKED, NULL);
                }
            }
        }
    }

    // Обновляем статус
    const char *status = "Готово";
    if (s_wifi_scanning) {
        status = "Поиск WiFi...";
    } else if (s_mdns_scanning) {
        status = "Поиск устройств...";
    } else if (s_streaming_tx) {
        status = "ПЕРЕДАЧА...";
    } else if (s_is_recording) {
        status = "Запись...";
    } else if (s_is_playing) {
        status = "Воспроизв...";
    } else if (s_wifi_connected) {
        static char wifi_status[64];
        // Получаем текущий RSSI
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_wifi_rssi = ap_info.rssi;
        }
        int quality = (s_wifi_rssi >= -50) ? 100 : (s_wifi_rssi <= -100) ? 0 : 2 * (s_wifi_rssi + 100);
        // Индикатор качества сигнала (5 уровней) с помощью точек
        const char *signal_bars;
        if (quality >= 80) {
            signal_bars = "*****";      // Отличный (80-100%)
        } else if (quality >= 60) {
            signal_bars = "****-";      // Хороший (60-79%)  
        } else if (quality >= 40) {
            signal_bars = "***--";      // Средний (40-59%)
        } else if (quality >= 20) {
            signal_bars = "**---";      // Слабый (20-39%)
        } else {
            signal_bars = "*----";      // Очень слабый (0-19%)
        }
        snprintf(wifi_status, sizeof(wifi_status), "%s %s %s", LV_SYMBOL_WIFI, s_wifi_ssid, signal_bars);
        status = wifi_status;
    }

    // Индикация входящего сигнала - показываем метку "ПРИЁМ" когда приём активен
    static bool last_rx_active = false;
    if (s_rx_active != last_rx_active) {
        last_rx_active = s_rx_active;
        if (s_rx_active && s_label_rx) {
            // Входящий сигнал - показываем индикатор
            lv_obj_clear_flag(s_label_rx, LV_OBJ_FLAG_HIDDEN);
        } else if (s_label_rx) {
            // Нет сигнала - скрываем индикатор
            lv_obj_add_flag(s_label_rx, LV_OBJ_FLAG_HIDDEN);
        }
        // Также меняем цвет кнопки, если не передаём
        if (!s_streaming_tx && s_btn_tx) {
            if (s_rx_active) {
                lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(255, 150, 50), 0);  // Оранжевый
            } else {
                lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(50, 150, 50), 0);   // Зелёный
            }
        }
    }

    lv_label_set_text_fmt(s_label,
                          "%s\nМик: %d%%",
                          status, s_mic_level);
}

static void btn_speaker_cb(lv_event_t *e)
{
    (void)e;
    
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "Test Button - WiFi not connected, playing beep");
        s_beep_request = true;
        return;
    }
    
    if (!s_streaming_tx) {
        // Не передаём - начинаем передачу
        ESP_LOGI(TAG, "Test Button - Start TX to %s", 
                 s_room_name[0] ? s_room_name : "broadcast");
        stream_tx_start();
        
        // Меняем цвет кнопки Тест на красный
        if (s_btn_speaker) {
            lv_obj_set_style_bg_color(s_btn_speaker, lv_color_make(200, 50, 50), 0);
        }
        // Меняем цвет кнопки TX на красный
        if (s_btn_tx) {
            lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(200, 50, 50), 0);
        }
    } else {
        // Передаём - останавливаем
        ESP_LOGI(TAG, "Test Button - Stop TX");
        stream_tx_stop();
        
        // Возвращаем цвета кнопок
        if (s_btn_speaker) {
            lv_obj_set_style_bg_color(s_btn_speaker, lv_color_hex(0x2196F3), 0);  // Стандартный синий
        }
        if (s_btn_tx) {
            lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(50, 150, 50), 0);  // Зелёный
        }
    }
}

static void btn_record_cb(lv_event_t *e)
{
    (void)e;
    if (!s_is_recording && !s_is_playing) {
        ESP_LOGI(TAG, "Record button pressed");
        s_record_request = true;
    }
}

// Callback для кнопки TX - PTT режим (Press-to-Talk)
static void btn_tx_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_PRESSED) {
        // Кнопка нажата - начинаем передачу
        if (!s_wifi_connected) {
            ESP_LOGW(TAG, "TX Button - WiFi not connected, ignoring");
            return;
        }
        ESP_LOGI(TAG, "TX Button PRESSED - Start TX to %s", 
                 s_room_name[0] ? s_room_name : "broadcast");
        stream_tx_start();
        
        // Меняем UI - красный цвет при передаче
        if (s_streaming_tx && s_btn_tx) {
            lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(200, 50, 50), 0);
        }
    } 
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        // Кнопка отпущена - останавливаем передачу
        ESP_LOGI(TAG, "TX Button RELEASED - Stop TX");
        stream_tx_stop();
        
        // Возвращаем цвет в зависимости от состояния RX
        if (s_btn_tx) {
            if (s_rx_active) {
                // Если приём активен - оранжевый
                lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(255, 150, 50), 0);
            } else {
                // Иначе - зелёный
                lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(50, 150, 50), 0);
            }
        }
    }
}

// Wi-Fi UI callbacks
static void wifi_panel_close(void)
{
    if (s_kb) {
        lv_obj_delete(s_kb);
        s_kb = NULL;
    }
    if (s_pwd_ta) {
        lv_obj_delete(s_pwd_ta);
        s_pwd_ta = NULL;
    }
    if (s_wifi_panel) {
        lv_obj_delete(s_wifi_panel);
        s_wifi_panel = NULL;
        s_wifi_list = NULL;
    }
}

// Device Selection UI - выбор устройства из списка
static void ip_panel_close(void)
{
    if (s_kb) {
        lv_obj_delete(s_kb);
        s_kb = NULL;
    }
    if (s_ip_ta) {
        lv_obj_delete(s_ip_ta);
        s_ip_ta = NULL;
    }
    if (s_ip_panel) {
        lv_obj_delete(s_ip_panel);
        s_ip_panel = NULL;
        s_device_list = NULL;
    }
}

// ===== Audio Settings Panel =====
static void audio_panel_close(void)
{
    if (s_audio_panel) {
        lv_obj_delete(s_audio_panel);
        s_audio_panel = NULL;
        s_slider_aecm_delay = NULL;
        s_slider_agc_gain = NULL;
        s_slider_noise_gate = NULL;
        s_sw_aecm = NULL;
        s_sw_agc = NULL;
        s_label_aecm_delay = NULL;
        s_label_agc_gain = NULL;
        s_label_noise_gate = NULL;
        s_btn_calibrate = NULL;
        s_label_calibrate = NULL;
    }
}

static void btn_audio_close_cb(lv_event_t *e)
{
    (void)e;
    // Сохраняем настройки при закрытии
    save_audio_settings();
    audio_panel_close();
}

static void slider_aecm_delay_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    s_aecm_delay_ms = (int)lv_slider_get_value(slider);
    if (s_label_aecm_delay) {
        lv_label_set_text_fmt(s_label_aecm_delay, "Задержка: %d мс", s_aecm_delay_ms);
    }
}

static void slider_agc_gain_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    s_agc_gain_db = (int)lv_slider_get_value(slider);
    if (s_label_agc_gain) {
        lv_label_set_text_fmt(s_label_agc_gain, "Усиление: %d дБ", s_agc_gain_db);
    }
    // Обновляем настройки AGC на лету
    if (s_esp_agc_handle) {
        set_agc_config(s_esp_agc_handle, s_agc_gain_db, 1, -3);
    }
}

static void slider_noise_gate_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    s_noise_gate_threshold = (int)lv_slider_get_value(slider);
    if (s_label_noise_gate) {
        lv_label_set_text_fmt(s_label_noise_gate, "Шумоподавл: %d", s_noise_gate_threshold);
    }
}

static void sw_aecm_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    s_aecm_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "AECM %s", s_aecm_enabled ? "enabled" : "disabled");
}

static void sw_agc_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    s_agc_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "AGC %s", s_agc_enabled ? "enabled" : "disabled");
}

static void sw_pcm_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    s_use_pcm = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "Codec: %s", s_use_pcm ? "PCM (quality)" : "ADPCM (compact)");
}

// Callback для кнопки калибровки
// Задача калибровки (для запуска в отдельном потоке)
static TaskHandle_t s_calibration_task_handle = NULL;

static void calibration_task(void *arg)
{
    (void)arg;
    
    // Устанавливаем таймаут для всей задачи
    TickType_t task_start = xTaskGetTickCount();
    const TickType_t task_timeout = pdMS_TO_TICKS(8000);  // 8 секунд макс
    
    audio_calibrate_and_apply();
    
    // Проверяем, не превышен ли таймаут
    if ((xTaskGetTickCount() - task_start) > task_timeout) {
        ESP_LOGW(TAG, "Calibration task took too long");
    }
    
    // Небольшая задержка чтобы UI успел обработать предыдущие события
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Обновляем UI в LVGL контексте (увеличен таймаут)
    ESP_LOGI(TAG, "Calibration task finishing, updating UI...");
    if (bsp_display_lock(1000)) {
        ESP_LOGI(TAG, "Display locked, panel=%p, label=%p", s_audio_panel, s_label_calibrate);
        // Обновляем только если панель аудио ещё открыта
        if (s_audio_panel != NULL && s_label_calibrate != NULL) {
            lv_label_set_text(s_label_calibrate, LV_SYMBOL_REFRESH " Калибр.");
            ESP_LOGI(TAG, "Calibration button text restored");
        }
        if (s_audio_panel != NULL && s_label_aecm_delay != NULL) {
            lv_label_set_text_fmt(s_label_aecm_delay, "Задержка: %d мс", s_aecm_delay_ms);
        }
        if (s_audio_panel != NULL && s_slider_aecm_delay != NULL) {
            lv_slider_set_value(s_slider_aecm_delay, s_aecm_delay_ms, LV_ANIM_OFF);
        }
        bsp_display_unlock();
    } else {
        ESP_LOGW(TAG, "Failed to lock display for UI update after calibration");
    }
    
    s_calibration_task_handle = NULL;
    vTaskDelete(NULL);
}

static void btn_calibrate_cb(lv_event_t *e)
{
    (void)e;
    
    // Если калибровка уже идёт - можно отменить повторным нажатием
    if (s_calibration_running || s_calibration_task_handle != NULL) {
        ESP_LOGW(TAG, "Cancelling calibration...");
        s_calibration_running = false;  // Сигнал на отмену
        s_calibration_active = false;   // Останавливаем режим калибровки в TX/RX
        
        // Обновляем текст кнопки
        if (s_label_calibrate) {
            lv_label_set_text(s_label_calibrate, "Отмена...");
        }
        return;
    }
    
    // Меняем текст кнопки
    if (s_label_calibrate) {
        lv_label_set_text(s_label_calibrate, "...");
    }
    
    // Запускаем калибровку в отдельной задаче чтобы не блокировать UI
    BaseType_t ret = xTaskCreate(calibration_task, "calibrate", 4096, NULL, 5, &s_calibration_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create calibration task");
        if (s_label_calibrate) {
            lv_label_set_text(s_label_calibrate, "Ошибка!");
        }
        s_calibration_running = false;
    }
}

// ===== Общая панель настроек =====
static void settings_panel_close(void)
{
    if (s_settings_panel) {
        lv_obj_delete(s_settings_panel);
        s_settings_panel = NULL;
    }
}

static void settings_close_cb(lv_event_t *e)
{
    (void)e;
    settings_panel_close();
}

// Forward declarations для settings_panel
static void btn_wifi_cb(lv_event_t *e);
static void btn_ip_cb(lv_event_t *e);
static void btn_audio_cb(lv_event_t *e);

static void settings_wifi_cb(lv_event_t *e)
{
    (void)e;
    settings_panel_close();
    // Вызываем открытие WiFi панели напрямую
    btn_wifi_cb(NULL);
}

static void settings_ip_cb(lv_event_t *e)
{
    (void)e;
    settings_panel_close();
    // Вызываем открытие IP панели напрямую
    btn_ip_cb(NULL);
}

static void settings_audio_cb(lv_event_t *e)
{
    (void)e;
    settings_panel_close();
    // Вызываем открытие Audio панели напрямую
    btn_audio_cb(NULL);
}

static void btn_settings_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Settings button pressed");

    if (s_settings_panel) {
        settings_panel_close();
        return;
    }

    // Создаем главную панель настроек
    s_settings_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_settings_panel, 280, 200);
    lv_obj_align(s_settings_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_settings_panel, 10, 0);

    // Заголовок
    lv_obj_t *title = lv_label_create(s_settings_panel);
    lv_label_set_text(title, "Настройки");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(title, &font_montserrat_14_cyrillic, 0);

    // Кнопка закрытия
    lv_obj_t *btn_close = lv_btn_create(s_settings_panel);
    lv_obj_set_size(btn_close, 45, 40);
    lv_obj_align(btn_close, LV_ALIGN_TOP_RIGHT, 0, -5);
    lv_obj_add_event_cb(btn_close, settings_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_close = lv_label_create(btn_close);
    lv_label_set_text(label_close, LV_SYMBOL_CLOSE);
    lv_obj_center(label_close);

    // Контейнер для кнопок (горизонтальный ряд)
    lv_obj_t *btn_row = lv_obj_create(s_settings_panel);
    lv_obj_set_size(btn_row, 260, 65);
    lv_obj_align(btn_row, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_pad_all(btn_row, 5, 0);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);  // Отключаем скролл

    // Кнопка WiFi
    lv_obj_t *btn_wifi = lv_btn_create(btn_row);
    lv_obj_set_size(btn_wifi, 75, 50);
    lv_obj_add_event_cb(btn_wifi, settings_wifi_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_wifi, lv_color_make(50, 100, 200), 0);
    lv_obj_t *wifi_icon = lv_label_create(btn_wifi);
    lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_align(wifi_icon, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_t *wifi_txt = lv_label_create(btn_wifi);
    lv_label_set_text(wifi_txt, "WiFi");
    lv_obj_align(wifi_txt, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_text_font(wifi_txt, &font_montserrat_14_cyrillic, 0);

    // Кнопка IP/Устройство
    lv_obj_t *btn_ip = lv_btn_create(btn_row);
    lv_obj_set_size(btn_ip, 75, 50);
    lv_obj_add_event_cb(btn_ip, settings_ip_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_ip, lv_color_make(100, 150, 50), 0);
    lv_obj_t *ip_icon = lv_label_create(btn_ip);
    lv_label_set_text(ip_icon, LV_SYMBOL_HOME);
    lv_obj_align(ip_icon, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_t *ip_txt = lv_label_create(btn_ip);
    lv_label_set_text(ip_txt, "Адрес");
    lv_obj_align(ip_txt, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_text_font(ip_txt, &font_montserrat_14_cyrillic, 0);

    // Кнопка Аудио - временно отключена
    // lv_obj_t *btn_audio = lv_btn_create(btn_row);
    // lv_obj_set_size(btn_audio, 75, 50);
    // lv_obj_add_event_cb(btn_audio, settings_audio_cb, LV_EVENT_CLICKED, NULL);
    // lv_obj_set_style_bg_color(btn_audio, lv_color_make(150, 100, 200), 0);
    // lv_obj_t *audio_icon = lv_label_create(btn_audio);
    // lv_label_set_text(audio_icon, LV_SYMBOL_AUDIO);
    // lv_obj_align(audio_icon, LV_ALIGN_TOP_MID, 0, 2);
    // lv_obj_t *audio_txt = lv_label_create(btn_audio);
    // lv_label_set_text(audio_txt, "Аудио");
    // lv_obj_align(audio_txt, LV_ALIGN_BOTTOM_MID, 0, -2);
    // lv_obj_set_style_text_font(audio_txt, &font_montserrat_14_cyrillic, 0);

    // Разделительная линия
    lv_obj_t *line = lv_obj_create(s_settings_panel);
    lv_obj_set_size(line, 240, 2);
    lv_obj_align(line, LV_ALIGN_TOP_MID, 0, 105);
    lv_obj_set_style_bg_color(line, lv_color_make(100, 100, 100), 0);
    lv_obj_set_style_border_width(line, 0, 0);

    // Информация о сборке
    lv_obj_t *build_label = lv_label_create(s_settings_panel);
    char build_info[64];
    snprintf(build_info, sizeof(build_info), "Сборка: %s %s", BUILD_DATE, BUILD_TIME);
    lv_label_set_text(build_label, build_info);
    lv_obj_align(build_label, LV_ALIGN_TOP_MID, 0, 115);
    lv_obj_set_style_text_font(build_label, &font_montserrat_14_cyrillic, 0);
    lv_obj_set_style_text_color(build_label, lv_color_make(128, 128, 128), 0);

    // MAC адрес устройства
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    lv_obj_t *mac_label = lv_label_create(s_settings_panel);
    char mac_str[32];
    snprintf(mac_str, sizeof(mac_str), "MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    lv_label_set_text(mac_label, mac_str);
    lv_obj_align(mac_label, LV_ALIGN_TOP_MID, 0, 135);
    lv_obj_set_style_text_color(mac_label, lv_color_make(128, 128, 128), 0);

    // Версия прошивки (можно добавить в будущем)
    lv_obj_t *ver_label = lv_label_create(s_settings_panel);
    lv_label_set_text(ver_label, "ESP32-S3 Intercom v1.0");
    lv_obj_align(ver_label, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_text_font(ver_label, &font_montserrat_14_cyrillic, 0);
    lv_obj_set_style_text_color(ver_label, lv_color_make(100, 100, 100), 0);
}

static void btn_audio_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Audio settings button pressed");

    if (s_audio_panel) {
        audio_panel_close();
        return;
    }

    // Создаем панель настроек аудио на весь экран
    s_audio_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_audio_panel, 320, 240);
    lv_obj_align(s_audio_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_audio_panel, 5, 0);

    // Заголовок
    lv_obj_t *title = lv_label_create(s_audio_panel);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " Настройки аудио");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(title, &font_montserrat_14_cyrillic, 0);

    // Кнопка закрытия
    lv_obj_t *btn_close = lv_btn_create(s_audio_panel);
    lv_obj_set_size(btn_close, 45, 40);
    lv_obj_align(btn_close, LV_ALIGN_TOP_RIGHT, 0, -5);
    lv_obj_add_event_cb(btn_close, btn_audio_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_label = lv_label_create(btn_close);
    lv_label_set_text(close_label, LV_SYMBOL_CLOSE);
    lv_obj_center(close_label);

    int y_pos = 22;
    
    // === AECM секция ===
    // AECM переключатель
    lv_obj_t *aecm_label = lv_label_create(s_audio_panel);
    lv_label_set_text(aecm_label, "Эхоподавление:");
    lv_obj_align(aecm_label, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(aecm_label, &font_montserrat_14_cyrillic, 0);
    
    s_sw_aecm = lv_switch_create(s_audio_panel);
    lv_obj_align(s_sw_aecm, LV_ALIGN_TOP_RIGHT, -5, y_pos - 3);
    if (s_aecm_enabled) lv_obj_add_state(s_sw_aecm, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_sw_aecm, sw_aecm_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    y_pos += 28;
    
    // AECM Delay slider
    s_label_aecm_delay = lv_label_create(s_audio_panel);
    lv_label_set_text_fmt(s_label_aecm_delay, "Задержка: %d мс", s_aecm_delay_ms);
    lv_obj_align(s_label_aecm_delay, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(s_label_aecm_delay, &font_montserrat_14_cyrillic, 0);
    
    s_slider_aecm_delay = lv_slider_create(s_audio_panel);
    lv_obj_set_size(s_slider_aecm_delay, 180, 10);
    lv_obj_align(s_slider_aecm_delay, LV_ALIGN_TOP_RIGHT, -10, y_pos + 3);
    lv_slider_set_range(s_slider_aecm_delay, 10, 200);
    lv_slider_set_value(s_slider_aecm_delay, s_aecm_delay_ms, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_slider_aecm_delay, slider_aecm_delay_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    y_pos += 30;
    
    // === AGC секция ===
    // AGC переключатель
    lv_obj_t *agc_label = lv_label_create(s_audio_panel);
    lv_label_set_text(agc_label, "Усиление (AGC):");
    lv_obj_align(agc_label, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(agc_label, &font_montserrat_14_cyrillic, 0);
    
    s_sw_agc = lv_switch_create(s_audio_panel);
    lv_obj_align(s_sw_agc, LV_ALIGN_TOP_RIGHT, -5, y_pos - 3);
    if (s_agc_enabled) lv_obj_add_state(s_sw_agc, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_sw_agc, sw_agc_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    y_pos += 28;
    
    // AGC Gain slider
    s_label_agc_gain = lv_label_create(s_audio_panel);
    lv_label_set_text_fmt(s_label_agc_gain, "Усиление: %d дБ", s_agc_gain_db);
    lv_obj_align(s_label_agc_gain, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(s_label_agc_gain, &font_montserrat_14_cyrillic, 0);
    
    s_slider_agc_gain = lv_slider_create(s_audio_panel);
    lv_obj_set_size(s_slider_agc_gain, 180, 10);
    lv_obj_align(s_slider_agc_gain, LV_ALIGN_TOP_RIGHT, -10, y_pos + 3);
    lv_slider_set_range(s_slider_agc_gain, 0, 60);
    lv_slider_set_value(s_slider_agc_gain, s_agc_gain_db, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_slider_agc_gain, slider_agc_gain_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    y_pos += 30;
    
    // === Noise Gate секция ===
    s_label_noise_gate = lv_label_create(s_audio_panel);
    lv_label_set_text_fmt(s_label_noise_gate, "Шумоподавл: %d", s_noise_gate_threshold);
    lv_obj_align(s_label_noise_gate, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(s_label_noise_gate, &font_montserrat_14_cyrillic, 0);
    
    s_slider_noise_gate = lv_slider_create(s_audio_panel);
    lv_obj_set_size(s_slider_noise_gate, 180, 10);
    lv_obj_align(s_slider_noise_gate, LV_ALIGN_TOP_RIGHT, -10, y_pos + 3);
    lv_slider_set_range(s_slider_noise_gate, 0, 500);
    lv_slider_set_value(s_slider_noise_gate, s_noise_gate_threshold, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_slider_noise_gate, slider_noise_gate_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    y_pos += 30;
    
    // === PCM/ADPCM секция ===
    lv_obj_t *pcm_label = lv_label_create(s_audio_panel);
    lv_label_set_text(pcm_label, "Кодек:");
    lv_obj_align(pcm_label, LV_ALIGN_TOP_LEFT, 5, y_pos);
    lv_obj_set_style_text_font(pcm_label, &font_montserrat_14_cyrillic, 0);
    
    // Подпись ADPCM слева от переключателя
    lv_obj_t *adpcm_lbl = lv_label_create(s_audio_panel);
    lv_label_set_text(adpcm_lbl, "ADPCM");
    lv_obj_align(adpcm_lbl, LV_ALIGN_TOP_RIGHT, -75, y_pos);
    lv_obj_set_style_text_font(adpcm_lbl, &font_montserrat_14_cyrillic, 0);
    lv_obj_set_style_text_color(adpcm_lbl, lv_color_hex(0x888888), 0);
    
    s_sw_pcm = lv_switch_create(s_audio_panel);
    lv_obj_align(s_sw_pcm, LV_ALIGN_TOP_RIGHT, -40, y_pos - 3);
    if (s_use_pcm) lv_obj_add_state(s_sw_pcm, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_sw_pcm, sw_pcm_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Подпись PCM справа от переключателя
    lv_obj_t *pcm_lbl = lv_label_create(s_audio_panel);
    lv_label_set_text(pcm_lbl, "PCM");
    lv_obj_align(pcm_lbl, LV_ALIGN_TOP_RIGHT, -5, y_pos);
    lv_obj_set_style_text_font(pcm_lbl, &font_montserrat_14_cyrillic, 0);
    lv_obj_set_style_text_color(pcm_lbl, lv_color_hex(0x888888), 0);
    
    y_pos += 28;
    
    // === Кнопка автокалибровки ===
    s_btn_calibrate = lv_btn_create(s_audio_panel);
    lv_obj_set_size(s_btn_calibrate, 140, 35);
    lv_obj_align(s_btn_calibrate, LV_ALIGN_TOP_MID, 0, y_pos);
    lv_obj_add_event_cb(s_btn_calibrate, btn_calibrate_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(s_btn_calibrate, lv_color_make(50, 150, 200), 0);
    
    s_label_calibrate = lv_label_create(s_btn_calibrate);
    lv_label_set_text(s_label_calibrate, LV_SYMBOL_REFRESH " Калибр.");
    lv_obj_center(s_label_calibrate);
    lv_obj_set_style_text_font(s_label_calibrate, &font_montserrat_14_cyrillic, 0);
    
    // Инфо строка
    lv_obj_t *info = lv_label_create(s_audio_panel);
    lv_label_set_text(info, "Калибровка измерит задержку");
    lv_obj_align(info, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_color(info, lv_color_make(128, 128, 128), 0);
    lv_obj_set_style_text_font(info, &font_montserrat_14_cyrillic, 0);
}

// Временное хранение выбранного hostname до ввода имени комнаты
static char s_selected_hostname[64] = {0};
static lv_obj_t *s_room_ta = NULL;  // Поле ввода имени комнаты

// Обработчик нажатия на кнопку переключения языка
static void kb_lang_btn_cb(lv_event_t *e)
{
    lv_obj_t *kb = lv_event_get_user_data(e);
    lv_obj_t *ta = lv_keyboard_get_textarea(kb);
    const char *txt = lv_buttonmatrix_get_button_text(kb, lv_buttonmatrix_get_selected_button(kb));
    
    if (txt && (strcmp(txt, "en") == 0 || strcmp(txt, "EN") == 0)) {
        // Переключаем на английский (используем наши английские карты с кнопкой ru)
        s_kb_russian = false;
        lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_LOWER, kb_map_en_lc, kb_ctrl_en_map);
        lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_UPPER, kb_map_en_uc, kb_ctrl_en_map);
        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_TEXT_LOWER);
        
        // Удаляем введённый текст "en"
        if (ta) {
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
        }
    } else if (txt && (strcmp(txt, "ru") == 0 || strcmp(txt, "RU") == 0)) {
        // Переключаем на русский
        s_kb_russian = true;
        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_1);
        
        // Удаляем введённый текст "ru"
        if (ta) {
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
        }
    } else if (txt && strcmp(txt, "АБВ") == 0) {
        // Для русской раскладки: переключаем на верхний регистр (USER_2)
        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_2);
        // Удаляем введённый текст "АБВ" (3 кириллических символа)
        if (ta) {
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
        }
    } else if (txt && strcmp(txt, "абв") == 0) {
        // Для русской раскладки: переключаем на нижний регистр (USER_1)
        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_1);
        // Удаляем введённый текст "абв" (3 кириллических символа)
        if (ta) {
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
            lv_textarea_delete_char(ta);
        }
    }
}

// Настройка клавиатуры с поддержкой русского языка
static void setup_keyboard_russian(lv_obj_t *kb)
{
    // Устанавливаем русские карты для USER режимов
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_USER_1, kb_map_ru_lc, kb_ctrl_ru_map);
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_USER_2, kb_map_ru_uc, kb_ctrl_ru_map);
    
    // Устанавливаем английские карты с кнопкой переключения на русский
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_LOWER, kb_map_en_lc, kb_ctrl_en_map);
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_UPPER, kb_map_en_uc, kb_ctrl_en_map);
    
    // Включаем русский режим по умолчанию (для ввода русских имён)
    s_kb_russian = true;
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_1);
    
    // Добавляем обработчик для кнопок переключения языка
    lv_obj_add_event_cb(kb, kb_lang_btn_cb, LV_EVENT_VALUE_CHANGED, kb);
    
    // Устанавливаем шрифт для клавиатуры
    lv_obj_set_style_text_font(kb, &font_montserrat_14_cyrillic, 0);
}

// Callback для клавиатуры ввода имени комнаты
static void room_kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        const char *room = lv_textarea_get_text(s_room_ta);
        
        // Сохраняем hostname и имя комнаты
        strncpy(s_target_host, s_selected_hostname, sizeof(s_target_host) - 1);
        if (room && room[0] != '\0') {
            strncpy(s_room_name, room, sizeof(s_room_name) - 1);
        } else {
            strncpy(s_room_name, s_selected_hostname, sizeof(s_room_name) - 1);
        }
        save_target_host(s_target_host, s_room_name);
        
        // Ищем IP адрес устройства в списке найденных
        s_target_ip[0] = '\0';
        for (int i = 0; i < s_discovered_count; i++) {
            if (s_discovered_devices[i].valid && 
                strcmp(s_discovered_devices[i].hostname, s_selected_hostname) == 0 &&
                s_discovered_devices[i].ip[0] != '\0') {
                strncpy(s_target_ip, s_discovered_devices[i].ip, sizeof(s_target_ip) - 1);
                break;
            }
        }
        
        ESP_LOGI(TAG, "Selected device: %s, Room: %s (IP: %s)", s_target_host, s_room_name,
                 s_target_ip[0] ? s_target_ip : "will resolve");
        
        // Обновляем текст на кнопке TX
        if (s_label_tx) {
            lv_label_set_text(s_label_tx, s_room_name);
        }
        s_room_ta = NULL;
        ip_panel_close();
    } else if (code == LV_EVENT_CANCEL) {
        s_room_ta = NULL;
        ip_panel_close();
    }
}

// Callback для выбора устройства из списка
static void device_list_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);
    
    // Проверяем валидность индекса
    if (idx >= 0 && idx < s_discovered_count && s_discovered_devices[idx].valid) {
        const char *hostname = s_discovered_devices[idx].hostname;
        strncpy(s_selected_hostname, hostname, sizeof(s_selected_hostname) - 1);
        
        // Убираем список устройств
        if (s_device_list) {
            lv_obj_delete(s_device_list);
            s_device_list = NULL;
        }
        
        // Показываем поле ввода имени комнаты (выше клавиатуры)
        lv_obj_t *room_label = lv_label_create(s_ip_panel);
        lv_label_set_text_fmt(room_label, "Имя для %s:", hostname);
        lv_obj_align(room_label, LV_ALIGN_TOP_LEFT, 5, 20);
        lv_obj_set_style_text_font(room_label, &font_montserrat_14_cyrillic, 0);
        lv_obj_set_style_bg_color(room_label, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(room_label, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(room_label, 3, 0);
        
        s_room_ta = lv_textarea_create(s_ip_panel);
        lv_textarea_set_one_line(s_room_ta, true);
        lv_textarea_set_placeholder_text(s_room_ta, "Введи имя");
        // Если это текущее устройство и имя уже задано - предзаполняем
        if (strcmp(hostname, s_target_host) == 0 && s_room_name[0] != '\0') {
            lv_textarea_set_text(s_room_ta, s_room_name);
        } else {
            lv_textarea_set_text(s_room_ta, "");
        }
        lv_obj_set_size(s_room_ta, 300, 35);
        lv_obj_align(s_room_ta, LV_ALIGN_TOP_MID, 0, 40);
        lv_obj_set_style_text_font(s_room_ta, &font_montserrat_14_cyrillic, 0);
        
        // Создаем клавиатуру (занимает нижнюю часть)
        s_kb = lv_keyboard_create(s_ip_panel);
        lv_obj_set_size(s_kb, 320, 150);
        lv_obj_align(s_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_keyboard_set_textarea(s_kb, s_room_ta);
        lv_obj_set_style_text_font(s_kb, &font_montserrat_14_cyrillic, 0);
        setup_keyboard_russian(s_kb);  // Настраиваем русскую клавиатуру
        lv_obj_add_event_cb(s_kb, room_kb_event_cb, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(s_kb, room_kb_event_cb, LV_EVENT_CANCEL, NULL);
    }
}

static void btn_ip_close_cb(lv_event_t *e)
{
    (void)e;
    ip_panel_close();
}

static void btn_ip_clear_cb(lv_event_t *e)
{
    (void)e;
    s_target_host[0] = '\0';
    s_target_ip[0] = '\0';
    s_room_name[0] = '\0';
    save_target_host("", "");
    ESP_LOGI(TAG, "Target host cleared, using broadcast");
    
    // Обновляем текст на кнопке TX
    if (s_label_tx) {
        lv_label_set_text(s_label_tx, "Вызов");
    }
    ip_panel_close();
}

static void btn_ip_scan_cb(lv_event_t *e)
{
    (void)e;
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected, cannot scan");
        return;
    }
    if (s_device_list) {
        lv_obj_clean(s_device_list);
        lv_list_add_text(s_device_list, "Поиск...");
    }
    mdns_scan_start();
}

static void btn_ip_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Device button pressed");

    if (s_ip_panel) {
        ip_panel_close();
        return;
    }

    // Создаем панель выбора устройства
    s_ip_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_ip_panel, 320, 240);
    lv_obj_align(s_ip_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_ip_panel, 5, 0);

    // Заголовок
    lv_obj_t *title = lv_label_create(s_ip_panel);
    lv_label_set_text(title, "Устройства");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(title, &font_montserrat_14_cyrillic, 0);

    // Кнопка закрытия
    lv_obj_t *btn_close = lv_btn_create(s_ip_panel);
    lv_obj_set_size(btn_close, 45, 40);
    lv_obj_align(btn_close, LV_ALIGN_TOP_RIGHT, 0, -5);
    lv_obj_add_event_cb(btn_close, btn_ip_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_label = lv_label_create(btn_close);
    lv_label_set_text(close_label, LV_SYMBOL_CLOSE);
    lv_obj_center(close_label);

    // Свой hostname (для идентификации)
    lv_obj_t *my_host_label = lv_label_create(s_ip_panel);
    lv_label_set_text_fmt(my_host_label, "Я: %s", s_my_hostname);
    lv_obj_align(my_host_label, LV_ALIGN_TOP_LEFT, 5, 20);
    lv_obj_set_style_text_color(my_host_label, lv_color_make(50, 150, 50), 0);
    lv_obj_set_style_text_font(my_host_label, &font_montserrat_14_cyrillic, 0);

    // Текущий target
    lv_obj_t *current_label = lv_label_create(s_ip_panel);
    if (s_target_host[0] != '\0') {
        lv_label_set_text_fmt(current_label, "Цель: %s", s_target_host);
    } else {
        lv_label_set_text(current_label, "Цель: все");
    }
    lv_obj_align(current_label, LV_ALIGN_TOP_LEFT, 5, 38);
    lv_obj_set_style_text_font(current_label, &font_montserrat_14_cyrillic, 0);

    // Кнопки: Scan и Broadcast
    lv_obj_t *btn_scan = lv_btn_create(s_ip_panel);
    lv_obj_set_size(btn_scan, 90, 30);
    lv_obj_align(btn_scan, LV_ALIGN_TOP_LEFT, 5, 55);
    lv_obj_add_event_cb(btn_scan, btn_ip_scan_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_scan, lv_color_make(50, 150, 200), 0);
    lv_obj_t *scan_label = lv_label_create(btn_scan);
    lv_label_set_text(scan_label, LV_SYMBOL_REFRESH " Поиск");
    lv_obj_center(scan_label);
    lv_obj_set_style_text_font(scan_label, &font_montserrat_14_cyrillic, 0);

    lv_obj_t *btn_clear = lv_btn_create(s_ip_panel);
    lv_obj_set_size(btn_clear, 90, 30);
    lv_obj_align(btn_clear, LV_ALIGN_TOP_LEFT, 100, 55);
    lv_obj_add_event_cb(btn_clear, btn_ip_clear_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_clear, lv_color_make(200, 100, 50), 0);
    lv_obj_t *clear_label = lv_label_create(btn_clear);
    lv_label_set_text(clear_label, "Все");
    lv_obj_center(clear_label);
    lv_obj_set_style_text_font(clear_label, &font_montserrat_14_cyrillic, 0);

    // Список найденных устройств
    s_device_list = lv_list_create(s_ip_panel);
    lv_obj_set_size(s_device_list, 300, 140);
    lv_obj_align(s_device_list, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_text_font(s_device_list, &font_montserrat_14_cyrillic, 0);

    // Если WiFi подключен - сразу начинаем сканировать
    if (s_wifi_connected) {
        lv_list_add_text(s_device_list, "Поиск...");
        mdns_scan_start();
    } else {
        lv_list_add_text(s_device_list, "Сначала подключись к WiFi");
    }
}

static char s_last_password[65] = {0};  // Для сохранения в NVS после подключения

static void kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        const char *pwd = lv_textarea_get_text(s_pwd_ta);
        strncpy(s_last_password, pwd, sizeof(s_last_password) - 1);  // Сохраняем пароль
        ESP_LOGI(TAG, "Connecting to %s", s_selected_ssid);
        wifi_connect(s_selected_ssid, pwd);
        wifi_panel_close();
    } else if (code == LV_EVENT_CANCEL) {
        wifi_panel_close();
    }
}

static void wifi_list_cb(lv_event_t *e)
{
    // Получаем текст выбранного элемента
    // lv_list_add_button создает: child(0) = иконка, child(1) = текст
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, 1);  // Текст во втором child
    if (label) {
        const char *text = lv_label_get_text(label);
        // Текст формата "SSID (xx%)" - нужно извлечь только SSID
        strncpy(s_selected_ssid, text, sizeof(s_selected_ssid) - 1);
        // Убираем "(xx%)" суффикс
        char *paren = strrchr(s_selected_ssid, '(');
        if (paren && paren > s_selected_ssid) {
            *(paren - 1) = '\0';  // Убираем пробел перед скобкой тоже
        }
        ESP_LOGI(TAG, "Selected: %s", s_selected_ssid);
        
        // Убираем список
        if (s_wifi_list) {
            lv_obj_delete(s_wifi_list);
            s_wifi_list = NULL;
        }

        // Создаем поле ввода пароля
        lv_obj_t *pwd_label = lv_label_create(s_wifi_panel);
        lv_label_set_text_fmt(pwd_label, LV_SYMBOL_WIFI " Пароль %s", s_selected_ssid);
        lv_obj_align(pwd_label, LV_ALIGN_TOP_LEFT, 5, 25);
        lv_obj_set_style_text_font(pwd_label, &font_montserrat_14_cyrillic, 0);

        s_pwd_ta = lv_textarea_create(s_wifi_panel);
        lv_textarea_set_one_line(s_pwd_ta, true);
        lv_textarea_set_password_mode(s_pwd_ta, true);
        lv_textarea_set_placeholder_text(s_pwd_ta, "Введи пароль");
        lv_obj_set_size(s_pwd_ta, 300, 35);
        lv_obj_align(s_pwd_ta, LV_ALIGN_TOP_MID, 0, 45);
        lv_obj_set_style_text_font(s_pwd_ta, &font_montserrat_14_cyrillic, 0);

        // Создаем клавиатуру на всю ширину экрана
        s_kb = lv_keyboard_create(s_wifi_panel);
        lv_obj_set_size(s_kb, 320, 140);
        lv_obj_align(s_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_keyboard_set_textarea(s_kb, s_pwd_ta);
        setup_keyboard_russian(s_kb);  // Настраиваем русскую клавиатуру
        lv_obj_add_event_cb(s_kb, kb_event_cb, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(s_kb, kb_event_cb, LV_EVENT_CANCEL, NULL);
    }
}

static void btn_wifi_close_cb(lv_event_t *e)
{
    (void)e;
    wifi_panel_close();
}

static void btn_wifi_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "WiFi button pressed");

    if (s_wifi_panel) {
        wifi_panel_close();
        return;
    }

    // Создаем панель Wi-Fi на весь экран (landscape: 320x240)
    s_wifi_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_wifi_panel, 320, 240);
    lv_obj_align(s_wifi_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_wifi_panel, 5, 0);

    // Заголовок
    lv_obj_t *title = lv_label_create(s_wifi_panel);
    lv_label_set_text(title, "Сети WiFi");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(title, &font_montserrat_14_cyrillic, 0);

    // Кнопка закрытия
    lv_obj_t *btn_close = lv_btn_create(s_wifi_panel);
    lv_obj_set_size(btn_close, 45, 40);
    lv_obj_align(btn_close, LV_ALIGN_TOP_RIGHT, 0, -5);
    lv_obj_add_event_cb(btn_close, btn_wifi_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, "X");
    lv_obj_center(lbl_close);

    // Список сетей (шире для landscape)
    s_wifi_list = lv_list_create(s_wifi_panel);
    lv_obj_set_size(s_wifi_list, 300, 180);
    lv_obj_align(s_wifi_list, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_style_text_font(s_wifi_list, &font_montserrat_14_cyrillic, 0);

    // Запускаем асинхронное сканирование
    lv_list_add_text(s_wifi_list, "Поиск...");
    wifi_scan_start();
}

// ===== Touch calibration debug =====
static void touch_debug_event_cb(lv_event_t *e)
{
    if (!s_touch_debug_enabled) return;
    
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    
    // Показываем точку в месте касания
    if (s_touch_point) {
        lv_obj_set_pos(s_touch_point, point.x - 10, point.y - 10);
        lv_obj_remove_flag(s_touch_point, LV_OBJ_FLAG_HIDDEN);
    }
    
    // Обновляем координаты
    if (s_touch_label) {
        lv_label_set_text_fmt(s_touch_label, "X:%ld Y:%ld", (long)point.x, (long)point.y);
    }
}

static void touch_debug_release_cb(lv_event_t *e)
{
    // Скрываем точку при отпускании (опционально)
    // if (s_touch_point) {
    //     lv_obj_add_flag(s_touch_point, LV_OBJ_FLAG_HIDDEN);
    // }
}

static void btn_touch_debug_cb(lv_event_t *e)
{
    (void)e;
    s_touch_debug_enabled = !s_touch_debug_enabled;
    
    if (s_touch_debug_enabled) {
        ESP_LOGI(TAG, "Touch debug ENABLED");
        // Показываем метку координат
        if (s_touch_label) {
            lv_obj_remove_flag(s_touch_label, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(s_touch_label, "Touch...");
        }
    } else {
        ESP_LOGI(TAG, "Touch debug DISABLED");
        // Скрываем точку и метку
        if (s_touch_point) {
            lv_obj_add_flag(s_touch_point, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_touch_label) {
            lv_obj_add_flag(s_touch_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void ui_init(void)
{
    s_indev = bsp_display_get_input_dev();

    bsp_display_lock(0);

    // Устанавливаем наш шрифт как шрифт по умолчанию для всех новых объектов
    lv_obj_set_style_text_font(lv_scr_act(), &font_montserrat_14_cyrillic, 0);

    // Статусная метка вверху СЛЕВА
    s_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_label, "Загрузка...");
    lv_obj_align(s_label, LV_ALIGN_TOP_LEFT, 10, 15);
    lv_obj_set_style_text_font(s_label, &font_montserrat_14_cyrillic, 0);

    // Единая кнопка настроек (шестерёнка) - справа вверху
    s_btn_settings = lv_btn_create(lv_scr_act());
    lv_obj_set_size(s_btn_settings, 50, 40);
    lv_obj_align(s_btn_settings, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_add_event_cb(s_btn_settings, btn_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(s_btn_settings, lv_color_make(80, 80, 80), 0);  // Серый цвет

    lv_obj_t *label_settings = lv_label_create(s_btn_settings);
    lv_label_set_text(label_settings, LV_SYMBOL_SETTINGS);
    lv_obj_center(label_settings);

    // Скрытые ссылки на старые кнопки для совместимости (не отображаются)
    s_btn_wifi = NULL;
    s_btn_ip = NULL;
    s_btn_audio = NULL;

    // Кнопка Speaker (beep) - верхний ряд слева
    s_btn_speaker = lv_btn_create(lv_scr_act());
    lv_obj_set_size(s_btn_speaker, 70, 35);
    lv_obj_align(s_btn_speaker, LV_ALIGN_CENTER, -70, -10);
    lv_obj_add_event_cb(s_btn_speaker, btn_speaker_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label_spk = lv_label_create(s_btn_speaker);
    lv_label_set_text(label_spk, "Тест");
    lv_obj_center(label_spk);
    lv_obj_set_style_text_font(label_spk, &font_montserrat_14_cyrillic, 0);

    // Кнопка Record - верхний ряд справа
    s_btn_record = lv_btn_create(lv_scr_act());
    lv_obj_set_size(s_btn_record, 70, 35);
    lv_obj_align(s_btn_record, LV_ALIGN_CENTER, 70, -10);
    lv_obj_add_event_cb(s_btn_record, btn_record_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(s_btn_record, lv_color_make(200, 50, 50), 0);  // Красноватый цвет

    lv_obj_t *label_rec = lv_label_create(s_btn_record);
    lv_label_set_text(label_rec, "Зап");
    lv_obj_center(label_rec);
    lv_obj_set_style_text_font(label_rec, &font_montserrat_14_cyrillic, 0);

    // БОЛЬШАЯ кнопка TX с названием комнаты - внизу по центру
    s_btn_tx = lv_btn_create(lv_scr_act());
    lv_obj_set_size(s_btn_tx, 200, 60);
    lv_obj_align(s_btn_tx, LV_ALIGN_BOTTOM_MID, 0, -25);
    // PTT: реагируем на нажатие и отпускание
    lv_obj_add_event_cb(s_btn_tx, btn_tx_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_btn_tx, btn_tx_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_set_style_bg_color(s_btn_tx, lv_color_make(50, 150, 50), 0);  // Зелёный

    // Метка на кнопке TX - название комнаты или "Вызов"
    s_label_tx = lv_label_create(s_btn_tx);
    if (s_room_name[0] != '\0') {
        lv_label_set_text(s_label_tx, s_room_name);
    } else {
        lv_label_set_text(s_label_tx, "Вызов");
    }
    lv_obj_align(s_label_tx, LV_ALIGN_CENTER, 0, -8);  // Выше центра (чтобы было место для индикатора RX)
    lv_obj_set_style_text_font(s_label_tx, &font_montserrat_14_cyrillic, 0);  // Кириллический шрифт

    // Индикатор входящего вызова - метка "ГОВОРИТ" под названием на кнопке TX
    s_label_rx = lv_label_create(s_btn_tx);
    lv_label_set_text(s_label_rx, LV_SYMBOL_CALL " ГОВОРИТ");
    lv_obj_align(s_label_rx, LV_ALIGN_CENTER, 0, 12);  // Ниже центра (под основной меткой)
    lv_obj_set_style_text_color(s_label_rx, lv_color_make(255, 255, 255), 0);  // Белый цвет на кнопке
    lv_obj_set_style_text_font(s_label_rx, &font_montserrat_14_cyrillic, 0);
    lv_obj_add_flag(s_label_rx, LV_OBJ_FLAG_HIDDEN);  // Скрыта по умолчанию

    // Подсказка над кнопкой TX
    lv_obj_t *hint = lv_label_create(lv_scr_act());
    lv_label_set_text(hint, "Удерживай для вызова");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -90);
    lv_obj_set_style_text_color(hint, lv_color_make(128, 128, 128), 0);
    lv_obj_set_style_text_font(hint, &font_montserrat_14_cyrillic, 0);
    
    // ===== Touch calibration debug =====
    // Кнопка включения режима отладки тача (маленькая, в левом нижнем углу)
    lv_obj_t *btn_touch = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_touch, 35, 25);
    lv_obj_align(btn_touch, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_add_event_cb(btn_touch, btn_touch_debug_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_touch, lv_color_make(80, 80, 80), 0);
    lv_obj_set_style_bg_opa(btn_touch, LV_OPA_70, 0);
    
    lv_obj_t *label_touch = lv_label_create(btn_touch);
    lv_label_set_text(label_touch, LV_SYMBOL_EYE_OPEN);
    lv_obj_center(label_touch);
    
    // Точка-индикатор касания (красный круг 20x20)
    s_touch_point = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_touch_point, 20, 20);
    lv_obj_set_style_radius(s_touch_point, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_touch_point, lv_color_make(255, 0, 0), 0);
    lv_obj_set_style_border_width(s_touch_point, 2, 0);
    lv_obj_set_style_border_color(s_touch_point, lv_color_make(255, 255, 255), 0);
    lv_obj_add_flag(s_touch_point, LV_OBJ_FLAG_HIDDEN);  // Скрыта по умолчанию
    lv_obj_remove_flag(s_touch_point, LV_OBJ_FLAG_CLICKABLE);  // Не ловит клики
    
    // Метка с координатами (вверху по центру)
    s_touch_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_touch_label, "X:0 Y:0");
    lv_obj_align(s_touch_label, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_set_style_bg_color(s_touch_label, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(s_touch_label, LV_OPA_70, 0);
    lv_obj_set_style_text_color(s_touch_label, lv_color_make(255, 255, 0), 0);
    lv_obj_add_flag(s_touch_label, LV_OBJ_FLAG_HIDDEN);  // Скрыта по умолчанию
    
    // Обработчик касания для всего экрана
    lv_obj_add_event_cb(lv_scr_act(), touch_debug_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(lv_scr_act(), touch_debug_release_cb, LV_EVENT_RELEASED, NULL);
    
    // RX будет запущен автоматически при подключении Wi-Fi

    lv_timer_create(ui_timer_cb, 100, NULL);

    bsp_display_unlock();
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ES3C28P test app");
    
    // Подавляем спам логов от LVGL touch
    esp_log_level_set("LVGL", ESP_LOG_ERROR);

    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Загружаем настройки аудио из NVS
    load_audio_settings();

    // Initialize Wi-Fi
    wifi_init();

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
    lv_display_t *disp = bsp_display_start();
    if (!disp) {
        ESP_LOGE(TAG, "Display init failed");
    } else {
        bsp_display_brightness_set(80);
        ui_init();
        
        // Touch debug task disabled to avoid interfering with LVGL input
    }
#else
    bsp_display_brightness_init();
    bsp_display_backlight_on();
#endif

    // Предварительная инициализация AEC и NS для быстрого старта TX
    // (после LVGL, чтобы дисплей успел выделить буферы)
#if USE_ESP_AEC
    esp_aec_lightweight_init();
#endif
#if USE_ESP_NS
    esp_ns_init_custom();
#endif
#if USE_AGC
    esp_agc_init_custom();
#endif
#if USE_VAD
    vad_init_custom();
#endif

    // Предварительно выделяем TX буферы для мгновенного старта
    s_tx_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_tx_mono_buf = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    s_tx_adpcm_buf = (uint8_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE / 2, MALLOC_CAP_DEFAULT);
    ESP_LOGI(TAG, "TX buffers pre-allocated: tx=%p mono=%p adpcm=%p", s_tx_buf, s_tx_mono_buf, s_tx_adpcm_buf);

    // audio_task на CPU0 с низким приоритетом (мониторинг уровня, не критично)
    // CPU1 оставляем свободным для audio_rx/tx с высоким приоритетом
    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 3, &s_audio_task_handle, 0);

    // Инициализация физической кнопки PTT (Cherry MX)
    ptt_button_init();
}
