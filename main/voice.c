#include "voice.h"
#include "motion.h"
#include "hardware.h"

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"

#include <stdlib.h>

static const char *TAG = "VOICE";

#define PDM_CLK_PIN   12
#define PDM_DATA_PIN  9

static const esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t        *afe_data   = NULL;
static esp_mn_iface_t           *multinet   = NULL;
static model_iface_data_t       *mn_data    = NULL;
static bool voice_ready = false;

static i2s_chan_handle_t rx_handle = NULL;

static int ultimo_id = -1;
static TickType_t tiempo_ultimo_id = 0;
#define DEBOUNCE_MS 800

static void aplicar_comando_voz(int id) {
    TickType_t ahora = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (id == ultimo_id && (ahora - tiempo_ultimo_id) < DEBOUNCE_MS) return;
    ultimo_id = id;
    tiempo_ultimo_id = ahora;

    ESP_LOGI(TAG, "Comando de voz detectado: id=%d", id);

    switch (id) {
        case 0: // go
            if (!modo_automatico && robot_encendido && !modo_calibracion) {
                direccion_actual = 2;
            }
            break;
        case 1: // back
            if (!modo_automatico && robot_encendido && !modo_calibracion) {
                direccion_actual = 1;
            }
            break;
        case 2: // left
            if (!modo_automatico && robot_encendido && !modo_calibracion) {
                direccion_actual = 4;
            }
            break;
        case 3: // right
            if (!modo_automatico && robot_encendido && !modo_calibracion) {
                direccion_actual = 5;
            }
            break;
        case 4: // jump
            if (robot_encendido && !modo_automatico && !modo_calibracion)
                flexiones();
            break;
        case 5: // power
            transicion_postura(!robot_encendido);
            if (!robot_encendido) {
                direccion_actual = 0;
                modo_calibracion = false;
            }
            break;
    }
}

esp_err_t voice_init(void) {
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "esp_srmodel_init fallo");
        return ESP_FAIL;
    }

    afe_config_t *afe_config = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!afe_config) {
        ESP_LOGE(TAG, "afe_config_init fallo");
        return ESP_FAIL;
    }

    afe_handle = esp_afe_handle_from_config(afe_config);
    if (!afe_handle) {
        ESP_LOGE(TAG, "afe_handle fallo");
        return ESP_FAIL;
    }

    afe_data = afe_handle->create_from_config(afe_config);
    if (!afe_data) {
        ESP_LOGE(TAG, "afe create fallo");
        return ESP_FAIL;
    }

    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!mn_name) {
        ESP_LOGE(TAG, "no MN ingles");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Modelo MultiNet: %s", mn_name);

    multinet = esp_mn_handle_from_name(mn_name);
    if (!multinet) {
        ESP_LOGE(TAG, "mn_handle fallo");
        return ESP_FAIL;
    }

    mn_data = multinet->create(mn_name, 5760);
    if (!mn_data) {
        ESP_LOGE(TAG, "mn create fallo");
        return ESP_FAIL;
    }

    esp_mn_commands_clear();
    esp_mn_commands_add(0, "go");
    esp_mn_commands_add(1, "back");
    esp_mn_commands_add(2, "left");
    esp_mn_commands_add(3, "right");
    esp_mn_commands_add(4, "jump");
    esp_mn_commands_add(5, "power");
    esp_mn_commands_update();

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_PIN,
            .din = PDM_DATA_PIN,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    voice_ready = true;
    ESP_LOGI(TAG, "ESP-SR inicializado (PDM GPIO12/9)");
    return ESP_OK;
}

void mic_feed_task(void *pvParameters) {
    if (!voice_ready || !rx_handle) {
        vTaskDelete(NULL);
        return;
    }

    int chunksize = afe_handle->get_feed_chunksize(afe_data);
    size_t size   = (size_t)chunksize * sizeof(int16_t);

    int16_t *buffer = malloc(size);
    if (!buffer) {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "mic_feed_task (chunk=%d)", chunksize);

    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(rx_handle, buffer, size, &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK || bytes_read == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        afe_handle->feed(afe_data, buffer);

        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) continue;

        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, ">>> WAKE WORD <<<");
        }

        esp_mn_state_t state = multinet->detect(mn_data, res->data);
        if (state == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *r = multinet->get_results(mn_data);
            if (r && r->num > 0) {
                aplicar_comando_voz(r->phrase_id[0]);
            }
        }
    }
}
