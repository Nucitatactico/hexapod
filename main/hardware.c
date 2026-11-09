#include "hardware.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <math.h>
#include <stdbool.h>

static const char *TAG = "HW";

#define PCA9685_ADDR     0x40
#define PCA9685_MODE1    0x00
#define PCA9685_PRESCALE 0xFE

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t pca_dev = NULL;

float ang_actual[12];
int   ang_objetivo[12];

#define INTERVALO_SUAVIZADO_MS 15
#define VEL_SUAVIZADO_COXA     1.5f
#define VEL_SUAVIZADO_FEMUR    4.0f

static esp_err_t pca_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(pca_dev, buf, 2, 1000);
}

static void pca9685_set_pwm_freq(float freq_hz) {
    uint8_t prescale = (uint8_t)((25000000.0f / (4096.0f * freq_hz)) - 1.0f + 0.5f);
    pca_write_reg(PCA9685_MODE1, 0x10);
    pca_write_reg(PCA9685_PRESCALE, prescale);
    vTaskDelay(pdMS_TO_TICKS(1));
    pca_write_reg(PCA9685_MODE1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void pca9685_set_channel(uint8_t canal, uint16_t valor) {
    uint8_t base = 0x06 + 4 * canal;
    uint8_t off_l = valor & 0xFF;
    uint8_t off_h = (valor >> 8) & 0x0F;
    pca_write_reg(base + 0, 0);
    pca_write_reg(base + 1, 0);
    pca_write_reg(base + 2, off_l);
    pca_write_reg(base + 3, off_h);
}

int angulo_a_pulsos(int angulo) {
    if (angulo < 0)   angulo = 0;
    if (angulo > 180) angulo = 180;
    return 130 + (angulo * (480 - 130) / 180);
}

void mover_servo_inmediato(uint8_t pin, int angulo) {
    if (pin > 11) return;
    ang_actual[pin]   = (float)angulo;
    ang_objetivo[pin] = angulo;
    pca9685_set_channel(pin, angulo_a_pulsos(angulo));
}

void mover_servo(uint8_t pin, int angulo) {
    if (pin > 11) return;
    ang_objetivo[pin] = angulo;
}

void procesar_movimiento_suave(void) {
    for (int i = 0; i < 12; i++) {
        float diff = (float)ang_objetivo[i] - ang_actual[i];
        if (fabsf(diff) > 0.5f) {
            float vel = (i % 2 == 1) ? VEL_SUAVIZADO_FEMUR : VEL_SUAVIZADO_COXA;
            if (diff > 0.0f) ang_actual[i] += vel;
            else             ang_actual[i] -= vel;
            if (fabsf((float)ang_objetivo[i] - ang_actual[i]) <= vel) {
                ang_actual[i] = (float)ang_objetivo[i];
            }
            pca9685_set_channel(i, angulo_a_pulsos((int)ang_actual[i]));
        }
    }
}

void mover_todas_caderas_centro(void) {
    mover_servo_inmediato(D_IZQ_C, COXA_CENTRO);
    mover_servo_inmediato(M_IZQ_C, COXA_CENTRO);
    mover_servo_inmediato(P_IZQ_C, COXA_CENTRO);
    mover_servo_inmediato(D_DER_C, COXA_CENTRO);
    mover_servo_inmediato(M_DER_C, COXA_CENTRO);
    mover_servo_inmediato(P_DER_C, COXA_CENTRO);
}

void mover_tripode_a(int femur, int coxa_izq, int coxa_der) {
    mover_servo(D_IZQ_F, femur); mover_servo(D_IZQ_C, coxa_izq);
    mover_servo(P_IZQ_F, femur); mover_servo(P_IZQ_C, coxa_izq);
    mover_servo(M_DER_F, femur); mover_servo(M_DER_C, coxa_der);
}

void mover_tripode_b(int femur, int coxa_izq, int coxa_der) {
    mover_servo(M_IZQ_F, femur); mover_servo(M_IZQ_C, coxa_izq);
    mover_servo(D_DER_F, femur); mover_servo(D_DER_C, coxa_der);
    mover_servo(P_DER_F, femur); mover_servo(P_DER_C, coxa_der);
}

esp_err_t hardware_init(void) {
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PIN_RADAR) | (1ULL << PIN_CDS),
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    gpio_set_level(PIN_CDS, 1);

    i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .scl_io_num        = PIN_SCL,
        .sda_io_num        = PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PCA9685_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &pca_dev));

    pca_write_reg(PCA9685_MODE1, 0x00);
    pca9685_set_pwm_freq(50.0f);

    for (int i = 0; i < 12; i++) {
        int a = (i % 2 == 1) ? FEMUR_ACOSTADO : COXA_CENTRO;
        mover_servo_inmediato(i, a);
    }

    ESP_LOGI(TAG, "Hardware listo (PCA9685 en 0x40, I2C 7/8)");
    return ESP_OK;
}
