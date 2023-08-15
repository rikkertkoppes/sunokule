set(component_srcs "src/webserver.c")

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES "driver" "esp_https_server" "storage" "wifi_conn" "storage" "esp_netif"
    EMBED_TXTFILES "certs/servercert.pem" "certs/prvtkey.pem"
)
