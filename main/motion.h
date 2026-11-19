#pragma once
#include <stdbool.h>

extern bool robot_encendido;
extern bool modo_automatico;
extern bool modo_calibracion;
extern int  direccion_actual;
extern int  velocidad_caminata;
extern bool en_transicion;

void set_accion(const char *texto);
const char *get_accion(void);

void transicion_postura(bool levantar);
void flexiones(void);

void motion_task(void *pvParameters);
