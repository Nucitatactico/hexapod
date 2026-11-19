#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"   // ← necesario para el tipo i2c_master_bus_handle_t

// ─── Pines ───
#define PIN_SDA    7
#define PIN_SCL    8
#define PIN_TRIG   4   // HC-SR04 Trigger
#define PIN_ECHO   5   // HC-SR04 Echo
#define PIN_CDS    4   // (legado, no se usa)
#define PIN_SCK    28
#define PIN_MOSI   29
#define PIN_CS     32
#define PIN_RST    33

// ─── LCD ST7920 (SPI por software) ───
#define PIN_LCD_CLK   PIN_SCK    // 28
#define PIN_LCD_DATA  PIN_MOSI   // 29
#define PIN_LCD_CS    PIN_CS     // 32
#define PIN_LCD_RST   PIN_RST    // 33

// ─── Índices de servos en el PCA9685 ───
#define D_IZQ_C  0
#define D_IZQ_F  1
#define M_IZQ_C  2
#define M_IZQ_F  3
#define P_IZQ_C  4
#define P_IZQ_F  5
#define D_DER_C  6
#define D_DER_F  7
#define M_DER_C  8
#define M_DER_F  9
#define P_DER_C 10
#define P_DER_F 11

// ─── Calibración ───
#define COXA_CENTRO             90
#define COXA_IZQ_ADELANTE      115
#define COXA_IZQ_ATRAS          65
#define COXA_DER_ADELANTE       65
#define COXA_DER_ATRAS         115

#define COXA_IZQ_GIRO_ADELANTE 102
#define COXA_IZQ_GIRO_ATRAS     78
#define COXA_DER_GIRO_ADELANTE  78
#define COXA_DER_GIRO_ATRAS    102

#define FEMUR_ACOSTADO         150
#define FEMUR_LEVANTADO        130
#define FEMUR_APOYO             45
i2c_master_bus_handle_t hardware_get_i2c_bus(void);
// ─── Funciones ───
esp_err_t hardware_init(void);
int  angulo_a_pulsos(int angulo);
void mover_servo_inmediato(uint8_t pin, int angulo);
void mover_servo(uint8_t pin, int angulo);
void procesar_movimiento_suave(void);
void mover_todas_caderas_centro(void);
void mover_tripode_a(int femur, int coxa_izq, int coxa_der);
void mover_tripode_b(int femur, int coxa_izq, int coxa_der);

// ─── NUEVO: acceso al bus I2C compartido ───
i2c_master_bus_handle_t hardware_get_i2c_bus(void);
