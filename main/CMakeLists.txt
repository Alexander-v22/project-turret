idf_component_register(
    SRCS "turret-control.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_timer
        esp_http_server
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_adc
        driver
)
