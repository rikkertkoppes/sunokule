set(component_srcs "src/wifi_conn.c")

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES "driver"
    REQUIRES "storage")
