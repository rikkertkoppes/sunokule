set(component_srcs "webserver.c")

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES "driver"
    REQUIRES "esp_https_server"
    EMBED_TXTFILES
    "certs/cacert.pem"
    "certs/prvtkey.pem")
