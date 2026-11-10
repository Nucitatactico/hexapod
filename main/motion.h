#pragma once
#include <stdbool.h>

extern bool robot_encendido;
extern bool modo_automatico;
extern bool modo_calibracion;
extern bool radar_fisicamente_encendido;
extern int  direccion_actual;
extern int  velocidad_caminata;
extern bool en_transicion;

void set_accion(const char *texto);
const char *get_accion(void);

// Radar con histéresis
bool radar_obstaculo(void);

// Secuencias
void transicion_postura(bool levantar);
void flexiones(void);

// Tarea
void motion_task(void *pvParameters);
