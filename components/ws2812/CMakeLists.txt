idf_component_register(
    SRCS "ws2812.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES "driver" "hal" "esp_timer"
)

set_source_files_properties(ws2812.c
    PROPERTIES COMPILE_FLAGS
    -O2
)
