idf_component_register(
    SRCS
        "main.c"
        "hardware.c"
        "motion.c"
        "voice.c"
        "pantalla.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        nvs_flash
        esp-sr
        u8g2
        esp_timer   # <-- Añadir esto
)
