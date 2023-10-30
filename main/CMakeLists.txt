idf_component_register(
    SRCS "taskmon.c" "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "esp_event" "led_strip" "storage" "tasks" "udp_client" "wifi_conn" "ws2812"
)
