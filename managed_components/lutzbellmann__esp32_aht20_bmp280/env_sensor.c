#include "env_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>


#if CONFIG_BMP280_I2C_ADDRESS_0x76
#define BMP280_ADDR 0x76
#else
#define BMP280_ADDR 0x77
#endif

#define SCL_PIN CONFIG_I2C_SCL_PIN
#define SDA_PIN CONFIG_I2C_SDA_PIN
#define I2C_BUS CONFIG_I2C_BUS

#if CONFIG_POWER_IS_GPIO
#define POWER_PIN CONFIG_POWER_GPIO_PIN
static TaskHandle_t s_read_task_handle = NULL;
static volatile bool s_stop_requested = false;
int read_freq;
env_sensor_reading_t env_sensor_last_reading;
#endif

static const char *TAG = "env_sensor";

i2c_master_bus_handle_t bus_local;


/* ──────────────────────────── AHT20 ──────────────────────────── */

#define AHT20_ADDR          0x38
#define AHT20_CMD_SOFTRESET 0xBA
#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_BIT_BUSY      (1 << 7)
#define AHT20_BIT_CALIB     (1 << 3)

static esp_err_t aht20_init(i2c_master_dev_handle_t dev)
{
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t cmd = AHT20_CMD_SOFTRESET;
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, 100);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(40)); /* datasheet min 20ms; 40ms safe margin after reset */

    uint8_t status;
    ret = i2c_master_receive(dev, &status, 1, 100);
    if (ret != ESP_OK) return ret;

    if (!(status & AHT20_BIT_CALIB)) {
        uint8_t init[] = {AHT20_CMD_INIT, 0x08, 0x00};
        ret = i2c_master_transmit(dev, init, sizeof(init), 100);
        if (ret != ESP_OK) return ret;
        vTaskDelay(pdMS_TO_TICKS(10));

        /* verify calibration bit is now set */
        ret = i2c_master_receive(dev, &status, 1, 100);
        if (ret != ESP_OK) return ret;
        if (!(status & AHT20_BIT_CALIB)) {
            ESP_LOGE(TAG, "AHT20 calibration failed (status=0x%02X)", status);
            return ESP_ERR_INVALID_STATE;
        }
    }
    return ESP_OK;
}

static esp_err_t aht20_read(i2c_master_dev_handle_t dev, float *temp, float *hum)
{
    uint8_t trigger[] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    esp_err_t ret = i2c_master_transmit(dev, trigger, sizeof(trigger), 100);
    if (ret != ESP_OK) return ret;

    /* poll busy bit; sensor typically ready in 80ms, allow up to 120ms */
    uint8_t d[6];
    for (int attempt = 0; attempt < 5; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(attempt == 0 ? 80 : 10));
        ret = i2c_master_receive(dev, d, sizeof(d), 100);
        if (ret != ESP_OK) return ret;
        if (!(d[0] & AHT20_BIT_BUSY)) break;
        if (attempt == 4) return ESP_ERR_TIMEOUT;
    }

    uint32_t raw_hum  = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t raw_temp = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];

    *hum  = (float)raw_hum  / (1 << 20) * 100.0f;
    *temp = (float)raw_temp / (1 << 20) * 200.0f - 50.0f;
    return ESP_OK;
}

/* ──────────────────────────── BMP280 ──────────────────────────── */

#define BMP280_REG_CHIP_ID   0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CALIB     0x88

static esp_err_t bmp280_read_regs(i2c_master_dev_handle_t dev, uint8_t reg,
                                   uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static esp_err_t bmp280_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 100);
}

static esp_err_t bmp280_init(i2c_master_dev_handle_t dev, env_sensor_t *h)
{
    uint8_t id;
    esp_err_t ret = bmp280_read_regs(dev, BMP280_REG_CHIP_ID, &id, 1);
    if (ret != ESP_OK) return ret;
    /* 0x58 = BMP280, 0x60 = BME280 (pressure-compatible) */
    if (id != 0x58 && id != 0x60) {
        ESP_LOGE(TAG, "BMP280 unexpected chip ID 0x%02X", id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ret = bmp280_write_reg(dev, BMP280_REG_RESET, 0xB6);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t cal[24];
    ret = bmp280_read_regs(dev, BMP280_REG_CALIB, cal, sizeof(cal));
    if (ret != ESP_OK) return ret;

#define LE16U(i) ((uint16_t)((cal[(i)] | ((uint16_t)cal[(i)+1] << 8))))
#define LE16S(i) ((int16_t) LE16U(i))
    h->dig_T1 = LE16U(0);  h->dig_T2 = LE16S(2);  h->dig_T3 = LE16S(4);
    h->dig_P1 = LE16U(6);  h->dig_P2 = LE16S(8);  h->dig_P3 = LE16S(10);
    h->dig_P4 = LE16S(12); h->dig_P5 = LE16S(14); h->dig_P6 = LE16S(16);
    h->dig_P7 = LE16S(18); h->dig_P8 = LE16S(20); h->dig_P9 = LE16S(22);
#undef LE16U
#undef LE16S

    /* osrs_t=x2, osrs_p=x16, normal mode */
    ret = bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS, 0x57);
    if (ret != ESP_OK) return ret;
    /* IIR filter coeff=16, standby=0.5 ms */
    ret = bmp280_write_reg(dev, BMP280_REG_CONFIG, 0x10);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(50));   /* first measurement ready */
    return ESP_OK;
}

static esp_err_t bmp280_read(i2c_master_dev_handle_t dev, env_sensor_t *h, float *pressure)
{
    uint8_t raw[6];
    esp_err_t ret = bmp280_read_regs(dev, BMP280_REG_PRESS_MSB, raw, sizeof(raw));
    if (ret != ESP_OK) return ret;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    /* Temperature compensation (Bosch datasheet section 4.2.3) */
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)h->dig_T1 << 1))) * h->dig_T2) >> 11;
    int32_t v2 = (((((adc_T >> 4) - (int32_t)h->dig_T1) *
                    ((adc_T >> 4) - (int32_t)h->dig_T1)) >> 12) * h->dig_T3) >> 14;
    int32_t t_fine = v1 + v2;

    /* Pressure compensation — 64-bit integer path */
    int64_t p1 = (int64_t)t_fine - 128000;
    int64_t p2 = p1 * p1 * (int64_t)h->dig_P6;
    p2 = p2 + ((p1 * (int64_t)h->dig_P5) << 17);
    p2 = p2 + (((int64_t)h->dig_P4) << 35);
    p1 = ((p1 * p1 * (int64_t)h->dig_P3) >> 8) + ((p1 * (int64_t)h->dig_P2) << 12);
    p1 = ((((int64_t)1) << 47) + p1) * ((int64_t)h->dig_P1) >> 33;
    if (p1 == 0) return ESP_ERR_INVALID_STATE;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p2) * 3125) / p1;
    p1 = (((int64_t)h->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p2 = (((int64_t)h->dig_P8) * p) >> 19;
    p = ((p + p1 + p2) >> 8) + (((int64_t)h->dig_P7) << 4);

    /* Result is Pa*256 (Q24.8); convert to hPa */
    *pressure = (float)(uint32_t)p / 256.0f / 100.0f;
    return ESP_OK;
}

#if CONFIG_POWER_IS_GPIO

void read_task(void *arg)
{
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)arg;
    env_sensor_t sensor;
    esp_err_t ret;
    while (1) {
        if (s_stop_requested) {
            s_stop_requested = false;
            s_read_task_handle = NULL;
            vTaskDelete(NULL);
        }
        ret = env_sensor_poweron();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to power on sensor: %s", esp_err_to_name(ret));
        }
        ret = env_sensor_init(bus, &sensor);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize sensor: %s", esp_err_to_name(ret));
        }
        ret = env_sensor_read(&sensor, &env_sensor_last_reading);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%RH, Pressure: %.2f hPa",
                     env_sensor_last_reading.temperature, env_sensor_last_reading.humidity, env_sensor_last_reading.pressure);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        }
        ret = env_sensor_poweroff();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to power off sensor: %s", esp_err_to_name(ret));
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(read_freq));  // sleep, but wake immediately if a stop is requested
    }
}
#endif
/* ──────────────────────────── Public API ──────────────────────── */

esp_err_t env_sensor_create_bus(i2c_master_bus_handle_t *bus_out)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_BUS,
        .sda_io_num        = SDA_PIN,
        .scl_io_num        = SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, bus_out);
}



esp_err_t env_sensor_init(i2c_master_bus_handle_t bus, env_sensor_t *handle)
{
    esp_err_t ret;
    i2c_device_config_t aht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT20_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(bus, &aht20_cfg, &handle->aht20);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t bmp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMP280_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(bus, &bmp_cfg, &handle->bmp280);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(handle->aht20);
        return ret;
    }

    ret = aht20_init(handle->aht20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AHT20 init: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = bmp280_init(handle->bmp280, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 init: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    return ESP_OK;

cleanup:
    i2c_master_bus_rm_device(handle->aht20);
    i2c_master_bus_rm_device(handle->bmp280);
    return ret;
}
#if CONFIG_POWER_IS_GPIO

esp_err_t env_sensor_init_auto_read(i2c_master_bus_handle_t bus, int read_wait)
{
    if (s_read_task_handle != NULL) return ESP_ERR_INVALID_STATE;

    read_freq = read_wait;
    s_stop_requested = false;
    BaseType_t ret = xTaskCreate(read_task, "env_sensor_read_task", 4096, bus, 5, &s_read_task_handle);
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}

esp_err_t env_sensor_stop_auto_read(i2c_master_bus_handle_t bus)
{
    if (s_read_task_handle == NULL) return ESP_ERR_INVALID_STATE;

    s_stop_requested = true;
    xTaskNotifyGive(s_read_task_handle);

    /* read_task clears the handle right before it deletes itself */
    while (s_read_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

esp_err_t env_sensor_poweron()
{
    static bool configured = false;
    if (!configured) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << POWER_PIN,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) return ret;
        configured = true;
    }

    gpio_set_level(POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(40)); /* allow sensors to power up */
    return ESP_OK;
}

esp_err_t env_sensor_poweroff() 
{
    gpio_set_level(POWER_PIN, 0);
    return ESP_OK;
}
#endif

esp_err_t env_sensor_read(env_sensor_t *handle, env_sensor_reading_t *out)
{
    esp_err_t ret = aht20_read(handle->aht20, &out->temperature, &out->humidity);
    if (ret != ESP_OK) return ret;

    return bmp280_read(handle->bmp280, handle, &out->pressure);
}

