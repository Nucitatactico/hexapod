#include "motion.h"
#include "hardware.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <string.h>
#include <stdbool.h>

static const char *TAG = "MOTION";

bool robot_encendido              = false;
bool modo_automatico              = false;
bool modo_calibracion             = false;
bool radar_fisicamente_encendido  = true;
int  direccion_actual             = 0;
int  velocidad_caminata           = 600;
bool en_transicion                = false;

static char accion_actual[32] = "ESTATICO";

void set_accion(const char *texto) {
    strncpy(accion_actual, texto, sizeof(accion_actual) - 1);
    accion_actual[sizeof(accion_actual) - 1] = '\0';
}
const char *get_accion(void) { return accion_actual; }

// ─── Radar con histéresis (disponible, pero NO usado todavía) ───
// Cuando aísles el sensor y decidas reactivar la evasión, esta función
// ya está lista. Por ahora se queda sin uso.
#define RADAR_ON_MS   400
#define RADAR_OFF_MS  600

static bool     radar_estado = false;
static TickType_t radar_cambio_ts = 0;

bool radar_obstaculo(void) {
    if (!radar_fisicamente_encendido) {
        radar_estado = false;
        return false;
    }

    bool raw = gpio_get_level(PIN_RADAR);
    TickType_t ahora = xTaskGetTickCount();
    uint32_t delta_ms = (ahora - radar_cambio_ts) * portTICK_PERIOD_MS;

    if (!radar_estado && raw) {
        if (delta_ms >= RADAR_ON_MS) {
            radar_estado = true;
            radar_cambio_ts = ahora;
            ESP_LOGI(TAG, "Radar: obstáculo CONFIRMADO");
        }
    } else if (radar_estado && !raw) {
        if (delta_ms >= RADAR_OFF_MS) {
            radar_estado = false;
            radar_cambio_ts = ahora;
            ESP_LOGI(TAG, "Radar: obstáculo LIBERADO");
        }
    } else {
        radar_cambio_ts = ahora;
    }

    return radar_estado;
}

// ─── Estado de marcha ───
static int fase_marcha = 0;
static unsigned long tiempo_paso = 0;
static int direccion_anterior = 0;

// ─── Marcha de 6 fases (versión original que ya funcionaba) ───
static void caminar(int direccion) {
    unsigned long ahora = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (ahora - tiempo_paso < (unsigned long)velocidad_caminata) return;
    tiempo_paso = ahora;

    if (direccion == 1) {
        set_accion("MOV: ATRAS (real)");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE);
    }
    else if (direccion == 2) {
        set_accion("MOV: ADELANTE (real)");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS);
    }
    else if (direccion == 4) {
        set_accion("GIRO: IZQUIERDA (real)");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS);
    }
    else if (direccion == 5) {
        set_accion("GIRO: DERECHA (real)");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE);
    }

    fase_marcha = (fase_marcha + 1) % 6;
}

// ─── Transición postura lenta ───
void transicion_postura(bool levantar) {
    en_transicion = true;
    set_accion(levantar ? "LEVANTANDO..." : "APAGANDO...");

    int inicial = levantar ? FEMUR_ACOSTADO : FEMUR_APOYO;
    int final_  = levantar ? FEMUR_APOYO    : FEMUR_ACOSTADO;
    int inc     = (final_ > inicial) ? 5 : -5;
    int actual  = inicial;

    mover_todas_caderas_centro();
    vTaskDelay(pdMS_TO_TICKS(200));

    while (actual != final_) {
        actual += inc;
        if ((inc > 0 && actual > final_) || (inc < 0 && actual < final_)) actual = final_;

        mover_servo_inmediato(D_IZQ_F, actual);
        mover_servo_inmediato(M_IZQ_F, actual);
        mover_servo_inmediato(P_IZQ_F, actual);
        mover_servo_inmediato(D_DER_F, actual);
        mover_servo_inmediato(M_DER_F, actual);
        mover_servo_inmediato(P_DER_F, actual);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    robot_encendido = levantar;
    set_accion("ESTATICO");
    en_transicion = false;
}

// ─── Flexiones ───
void flexiones(void) {
    en_transicion = true;
    set_accion("FLEXIONES");

    for (int r = 0; r < 3; r++) {
        for (int i = 1; i < 12; i += 2) mover_servo_inmediato(i, FEMUR_APOYO);
        mover_todas_caderas_centro();
        vTaskDelay(pdMS_TO_TICKS(400));

        for (int i = 1; i < 12; i += 2) mover_servo_inmediato(i, FEMUR_ACOSTADO);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    mover_tripode_a(FEMUR_APOYO, COXA_CENTRO, COXA_CENTRO);
    mover_tripode_b(FEMUR_APOYO, COXA_CENTRO, COXA_CENTRO);
    set_accion("ESTATICO");
    en_transicion = false;
}

// ─── Tarea principal SIN evasión ───
void motion_task(void *pvParameters) {
    while (1) {
        if (!en_transicion) {
            procesar_movimiento_suave();
        }

        if (robot_encendido && !en_transicion) {
            if (!modo_calibracion) {
                if (direccion_actual != direccion_anterior) {
                    fase_marcha = 0;
                    direccion_anterior = direccion_actual;
                }

                if (direccion_actual != 0) {
                    caminar(direccion_actual);
                } else {
                    mover_tripode_a(FEMUR_APOYO, COXA_CENTRO, COXA_CENTRO);
                    mover_tripode_b(FEMUR_APOYO, COXA_CENTRO, COXA_CENTRO);
                    fase_marcha = 0;
                    set_accion("ESTATICO");
                }
            } else {
                set_accion("MODO SERVOS");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
