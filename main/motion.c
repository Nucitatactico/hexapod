#include "motion.h"
#include "hardware.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <string.h>
#include <stdbool.h>

//static const char *TAG = "MOTION";
__attribute__((unused)) static const char *TAG = "MOTION";

bool robot_encendido     = false;
bool modo_automatico     = false;
bool modo_calibracion    = false;
int  direccion_actual    = 0;
int  velocidad_caminata  = 600;
bool en_transicion       = false;

static char accion_actual[32] = "ESTATICO";

void set_accion(const char *texto) {
    strncpy(accion_actual, texto, sizeof(accion_actual) - 1);
    accion_actual[sizeof(accion_actual) - 1] = '\0';
}
const char *get_accion(void) { return accion_actual; }

// ─── HC-SR04 en GPIO 4 (Trig) / GPIO 5 (Echo) ───
#define TIMEOUT_US         30000
#define UMBRAL_OBSTACULO   40.0f   // cm

static void hcsr04_init(void) {
    gpio_config_t trig = {
        .pin_bit_mask = (1ULL << PIN_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_set_level(PIN_TRIG, 0);

    gpio_config_t echo = {
        .pin_bit_mask = (1ULL << PIN_ECHO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo);
}

static float hcsr04_medir_cm(void) {
    gpio_set_level(PIN_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_TRIG, 0);

    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_ECHO) == 0) {
        if (esp_timer_get_time() - t0 > TIMEOUT_US) return -1.0f;
    }

    int64_t inicio = esp_timer_get_time();
    while (gpio_get_level(PIN_ECHO) == 1) {
        if (esp_timer_get_time() - inicio > TIMEOUT_US) return -1.0f;
    }
    int64_t fin = esp_timer_get_time();

    return (float)(fin - inicio) * 0.0343f / 2.0f;
}

static bool obstaculo_detectado(void) {
    float d = hcsr04_medir_cm();
    return (d > 0.0f && d < UMBRAL_OBSTACULO);
}

// ─── Marcha de 6 fases ───
static int fase_marcha = 0;
static unsigned long tiempo_paso = 0;
static int direccion_anterior = 0;

// Ejecuta UNA fase de marcha en la dirección indicada (respeta velocidad_caminata).
// Devuelve true si se ejecutó una fase, false si aún no toca por tiempo.
static bool caminar_paso(int direccion) {
    unsigned long ahora = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (ahora - tiempo_paso < (unsigned long)velocidad_caminata) return false;
    tiempo_paso = ahora;

    if (direccion == 1) {
        set_accion("MOV: ATRAS");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE);
    }
    else if (direccion == 2) {
        set_accion("MOV: ADELANTE");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_ATRAS, COXA_DER_ATRAS); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_ADELANTE, COXA_DER_ADELANTE); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_ATRAS, COXA_DER_ATRAS);
    }
    else if (direccion == 4) {
        set_accion("GIRO: IZQUIERDA");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS);
    }
    else if (direccion == 5) {
        set_accion("GIRO: DERECHA");
        if (fase_marcha == 0) mover_tripode_a(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 1) { mover_tripode_a(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); }
        else if (fase_marcha == 2) mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE);
        else if (fase_marcha == 3) mover_tripode_b(FEMUR_LEVANTADO, COXA_CENTRO, COXA_CENTRO);
        else if (fase_marcha == 4) { mover_tripode_b(FEMUR_LEVANTADO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE); mover_tripode_a(FEMUR_APOYO, COXA_IZQ_GIRO_ADELANTE, COXA_DER_GIRO_ATRAS); }
        else if (fase_marcha == 5) mover_tripode_b(FEMUR_APOYO, COXA_IZQ_GIRO_ATRAS, COXA_DER_GIRO_ADELANTE);
    }

    fase_marcha = (fase_marcha + 1) % 6;
    return true;
}

// ─── Transición postura lenta (power) ───
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

// ─── Jump: bajar y subir rápido ───
void flexiones(void) {
    en_transicion = true;
    set_accion("SALTO");

    mover_todas_caderas_centro();
    mover_servo_inmediato(D_IZQ_F, FEMUR_APOYO);
    mover_servo_inmediato(M_IZQ_F, FEMUR_APOYO);
    mover_servo_inmediato(P_IZQ_F, FEMUR_APOYO);
    mover_servo_inmediato(D_DER_F, FEMUR_APOYO);
    mover_servo_inmediato(M_DER_F, FEMUR_APOYO);
    mover_servo_inmediato(P_DER_F, FEMUR_APOYO);
    vTaskDelay(pdMS_TO_TICKS(100));

    mover_servo_inmediato(D_IZQ_F, FEMUR_ACOSTADO);
    mover_servo_inmediato(M_IZQ_F, FEMUR_ACOSTADO);
    mover_servo_inmediato(P_IZQ_F, FEMUR_ACOSTADO);
    mover_servo_inmediato(D_DER_F, FEMUR_ACOSTADO);
    mover_servo_inmediato(M_DER_F, FEMUR_ACOSTADO);
    mover_servo_inmediato(P_DER_F, FEMUR_ACOSTADO);
    vTaskDelay(pdMS_TO_TICKS(100));

    set_accion("ESTATICO");
    en_transicion = false;
}

// ─── Tarea principal ───
// Regla de go (direccion 2):
//   1. Si el sensor NO detecta obstáculo  → avanzar con el patrón de go.
//   2. Si el sensor SÍ detecta obstáculo → girar con el MISMO patrón de right
//      hasta que el frente quede libre (sin quedarse quieto).
//   3. En cuanto deje de detectar       → volver a avanzar con go.
void motion_task(void *pvParameters) {
    hcsr04_init();

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

                if (direccion_actual == 2) {
                    // GO: ¿hay obstáculo delante?
                    if (obstaculo_detectado()) {
                        // Girar con el patrón de right (dirección 5) hasta que el frente esté libre
                        caminar_paso(5);
                    } else {
                        // Frente libre → seguir avanzando con go
                        caminar_paso(2);
                    }
                } else if (direccion_actual != 0) {
                    // Otras direcciones (atrás, giros) sin evasión
                    caminar_paso(direccion_actual);
                } else {
                    // Estático
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
