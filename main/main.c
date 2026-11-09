#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware.h"
#include "motion.h"
#include "voice.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Iniciando Hexapodo (FireBeetle 2 ESP32-P4)...");

    ESP_ERROR_CHECK(hardware_init());

    if (voice_init() == ESP_OK) {
        xTaskCreatePinnedToCore(mic_feed_task, "mic_feed", 8192, NULL, 3, NULL, 0);
    } else {
        ESP_LOGE(TAG, "Voz no disponible.");
    }

    xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Sistema listo.");
}
