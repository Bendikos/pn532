idf_component_register(SRCS "src/pn532.c" "src/pn532_uart.c" "src/pn532_spi.c" "src/pn532_i2c.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
