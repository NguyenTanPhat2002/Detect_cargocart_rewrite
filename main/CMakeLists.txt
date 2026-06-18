idf_component_register(SRCS "my_protocol.c" "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES
                        esp_driver_gpio
                        esp_wifi
                        esp_event
                        esp_netif
                        nvs_flash
                        )
