#pragma once
#include "esp_err.h"

esp_err_t voice_init(void);
void      mic_feed_task(void *pvParameters);
