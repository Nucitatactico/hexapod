#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "env_sensor.h"

static const char *TAG = "env_sensor_example";

void app_main(void)
{
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(env_sensor_create_bus(&bus));

    env_sensor_t sensor;
    ESP_ERROR_CHECK(env_sensor_init(bus, &sensor));

    while (1) {
        env_sensor_reading_t reading;
        esp_err_t ret = env_sensor_read(&sensor, &reading);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "T=%.2f°C  RH=%.2f%%  P=%.2fhPa",
                     reading.temperature, reading.humidity, reading.pressure);
        } else {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
