idf_component_register(
    SRCS
        "main.c"
        "hardware.c"
        "motion.c"
        "voice.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        nvs_flash
        esp-sr
)
