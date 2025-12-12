set(req esp_driver_gpio freertos)

idf_component_register(
        SRCS "src/1602_driver.c"
        INCLUDE_DIRS "include"
        PRIV_REQUIRES ${req})
