set(component_srcs "src/webserver.c")

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES "driver"
    REQUIRES "esp_https_server storage wifi_conn storage"
    EMBED_TXTFILES
    "certs/cacert.pem"
    "certs/prvtkey.pem")
