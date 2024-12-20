#include "pn532.h"

#if CONFIG_LOG_DEFAULT_LEVEL >= 4 // 4 = LOG_LEVEL_DEBUG
#define PN532_DEBUG
#endif

#define PN532_DELAY_DEFAULT (pdMS_TO_TICKS(500))
#define PN532_DELAY(ms) (pdMS_TO_TICKS(ms))

typedef struct
{
    i2c_port_t i2c_port;
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;

} pn532_i2c_handle_t;

typedef struct
{
    spi_host_device_t spi_port;
    spi_device_handle_t spi_dev;
    // TODO
} pn532_spi_handle_t;

typedef struct pn532_t
{
    pn532_protocol_t protocol;
    union
    {
        uart_port_t uart_port;
        pn532_spi_handle_t spi;
        pn532_i2c_handle_t i2c;
    };
    SemaphoreHandle_t mutex;
    esp_err_t (*write_command)(struct pn532_t *pn532, uint8_t *command, uint8_t command_len);
    esp_err_t (*read_response)(struct pn532_t *pn532, uint8_t *response, uint8_t response_len);
    esp_err_t (*free)(struct pn532_t *pn532);
} pn532_t;