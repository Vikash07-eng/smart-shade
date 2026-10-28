idf_component_register(
    SRCS "main.c" "stepper.c" "sun.c"
    INCLUDE_DIRS "."
    EMBED_FILES "index.html"
    REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_server json driver esp_timer esp_hw_support esp_rom lwip
)
