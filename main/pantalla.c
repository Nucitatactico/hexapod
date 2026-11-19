#include "pantalla.h"
#include "hardware.h"
#include "motion.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <stdio.h>
#include <string.h>

#include "u8g2.h"
#include "env_sensor.h"

static const char *TAG = "PANTALLA";

#define PANTALLA_PERIODO_MS  500

static u8g2_t u8g2;

// ─────────────────────────────────────────────────────────────
// Callback GPIO + delay para u8g2 (SW SPI)
// ─────────────────────────────────────────────────────────────
static uint8_t gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << PIN_LCD_CLK) | (1ULL << PIN_LCD_DATA) |
                                (1ULL << PIN_LCD_CS)  | (1ULL << PIN_LCD_RST),
                .mode         = GPIO_MODE_OUTPUT,
                .pull_up_en   = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            gpio_config(&io_conf);
            gpio_set_level(PIN_LCD_CS, 1);
            gpio_set_level(PIN_LCD_RST, 1);
            break;
        }
        case U8X8_MSG_DELAY_NANO:      esp_rom_delay_us(1); break;
        case U8X8_MSG_DELAY_100NANO:   esp_rom_delay_us(1); break;
        case U8X8_MSG_DELAY_10MICRO:   esp_rom_delay_us(10); break;
        case U8X8_MSG_DELAY_MILLI:     vTaskDelay(pdMS_TO_TICKS(arg_int)); break;
        case U8X8_MSG_GPIO_CS:         gpio_set_level(PIN_LCD_CS, arg_int); break;
        case U8X8_MSG_GPIO_RESET:      gpio_set_level(PIN_LCD_RST, arg_int); break;
        case U8X8_MSG_GPIO_SPI_CLOCK:  gpio_set_level(PIN_LCD_CLK, arg_int); break;
        case U8X8_MSG_GPIO_SPI_DATA:   gpio_set_level(PIN_LCD_DATA, arg_int); break;
        default: return 0;
    }
    return 1;
}

static const char *modo_texto(void)
{
    if (modo_calibracion) return "SERVOS";
    return modo_automatico ? "AUTOMATICO" : "MANUAL";
}

// ─────────────────────────────────────────────────────────────
// Tarea de pantalla
// ─────────────────────────────────────────────────────────────
static void tarea_pantalla(void *pvParameters)
{
    // 1) Reutilizar el bus I2C creado por hardware_init()
    i2c_master_bus_handle_t bus = hardware_get_i2c_bus();
    if (!bus) {
        ESP_LOGE(TAG, "Bus I2C no disponible");
    }

    // 2) Inicializar sensores SIN abortar si fallan
    env_sensor_t sensor;
    bool sensor_ok = false;
    if (bus) {
        esp_err_t init_ret = env_sensor_init(bus, &sensor);
        if (init_ret == ESP_OK) {
            sensor_ok = true;
            ESP_LOGI(TAG, "Sensores OK (AHT20 + BMP280)");
        } else {
            ESP_LOGW(TAG, "Sensores no disponibles: %s", esp_err_to_name(init_ret));
        }
    }

    char ant[5][24] = {0};

    while (1) {
        // 3) Leer sensores SOLO si la init fue OK
        float temp = 0.0f, hum = 0.0f, pres = 0.0f;
        bool lectura_ok = false;

        if (sensor_ok) {
            env_sensor_reading_t reading;
            if (env_sensor_read(&sensor, &reading) == ESP_OK) {
                temp = reading.temperature;
                hum  = reading.humidity;
                pres = reading.pressure;
                lectura_ok = true;
            }
        }

        // 4) Formatear 5 líneas
        char l[5][24];

        if (lectura_ok) {
            snprintf(l[0], sizeof(l[0]), "T:%.1fC H:%.0f%%", temp, hum);
            snprintf(l[1], sizeof(l[1]), "P:%.0fhPa", pres);
        } else {
            snprintf(l[0], sizeof(l[0]), "T:--  H:--");
            snprintf(l[1], sizeof(l[1]), "P:--");
        }

        snprintf(l[2], sizeof(l[2]), "Modo: %s", modo_texto());
        snprintf(l[3], sizeof(l[3]), "%.20s", get_accion());
        snprintf(l[4], sizeof(l[4]), "%s",
                 robot_encendido ? "ESTADO: DE PIE" : "ESTADO: APAGADO");

        // 5) Redibujar solo si algo cambió
        bool cambio = false;
        for (int i = 0; i < 5; i++) {
            if (strcmp(l[i], ant[i]) != 0) { cambio = true; break; }
        }

        if (cambio) {
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
            u8g2_DrawStr(&u8g2, 0, 10, l[0]);
            u8g2_DrawStr(&u8g2, 0, 22, l[1]);
            u8g2_DrawStr(&u8g2, 0, 34, l[2]);
            u8g2_DrawStr(&u8g2, 0, 46, l[3]);
            u8g2_DrawLine(&u8g2, 0, 51, 128, 51);
            u8g2_DrawStr(&u8g2, 0, 62, l[4]);
            u8g2_SendBuffer(&u8g2);
            memcpy(ant, l, sizeof(ant));
        }

        vTaskDelay(pdMS_TO_TICKS(PANTALLA_PERIODO_MS));
    }
}

// ─────────────────────────────────────────────────────────────
// Inicialización
// ─────────────────────────────────────────────────────────────
void pantalla_iniciar(void)
{
    u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R0,
                                 u8x8_byte_3wire_sw_spi,
                                 gpio_and_delay_esp32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "LCD ST7920 iniciado");
    xTaskCreatePinnedToCore(tarea_pantalla, "Pantalla", 4096, NULL, 4, NULL, 0);
}
