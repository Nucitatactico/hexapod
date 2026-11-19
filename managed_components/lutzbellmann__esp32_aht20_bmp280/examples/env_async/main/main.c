#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "env_sensor.h"

#if !CONFIG_POWER_IS_GPIO
#error "This example requires CONFIG_POWER_IS_GPIO to be enabled"
#endif

#define EXAMPLE_DURATION_MS 120000 // Run for 2 minutes
#define EXAMPLE_READ_INTERVAL_MS 15000 // Read every 15 seconds
static const char *TAG = "env_async_example";

void values_task(void *arg)
{
    while (1) {
        ESP_LOGI(TAG, "Last reading: Temperature: %.2f °C, Humidity: %.2f %%RH, Pressure: %.2f hPa",
                 env_sensor_last_reading.temperature,
                 env_sensor_last_reading.humidity,
                 env_sensor_last_reading.pressure);    
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_READ_INTERVAL_MS));

    }        

}

void app_main(void)
{
    i2c_master_bus_handle_t bus;
    esp_err_t ret;
    ret = env_sensor_create_bus(&bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create bus: %s", esp_err_to_name(ret));
        return;
    }
    ret = env_sensor_init_auto_read(bus, EXAMPLE_READ_INTERVAL_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize auto read: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Auto read started. Reading every %d ms for %d ms.", EXAMPLE_READ_INTERVAL_MS, EXAMPLE_DURATION_MS);

    xTaskCreate(values_task, "values_task", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_DURATION_MS)); // Run for the specified duration

    ret = env_sensor_stop_auto_read(bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop auto read: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Auto read stopped.");
}
