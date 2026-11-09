#pragma once
#include <stdbool.h>

// Estado global del robot
extern bool robot_encendido;
extern bool modo_automatico;
extern bool modo_calibracion;
extern bool radar_fisicamente_encendido;
extern int  direccion_actual;
extern int  velocidad_caminata;
extern bool en_transicion;   // bloquea motion_task durante transicion_postura

// Acción legible para pantalla / web (sustituye al String de Arduino)
void set_accion(const char *texto);
const char *get_accion(void);

// Secuencias
void transicion_postura(bool levantar);
void flexiones(void);

// Tarea
void motion_task(void *pvParameters);
