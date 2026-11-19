file(GLOB u8g2_srcs "csrc/*.c")
idf_component_register(SRCS ${u8g2_srcs} INCLUDE_DIRS "csrc")
