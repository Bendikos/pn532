#include "pn532.h"
#include "pn532_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_log.h>


#define PN532_REG_SPI_CONTROL   0xa9    // SPI control bits         R/W
#define PN532_REG_SPI_STATUS    0xaa    // SPI Status/Error bits    R

#define PN532_MAX_TRANSFER_SIZE 256


static const char* TAG = "pn532_spi";

static esp_err_t _spi_write_data(pn532_t *pn532, uint8_t addr, uint8_t data)
{
    esp_err_t ret;

    /*If MSB of addr is set the host will read data from slave.
     *If MSB of addr is clear the host will write data on slave.
     */
    spi_transaction_t trans_desc = {
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {addr, data},
        .length = 16,
    };

    // transaction CS is drivven acordingly?
    ret = spi_device_polling_transmit(pn532->spi.dev, &trans_desc);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return ret;
}

static esp_err_t _spi_read_data(pn532_t *pn532, uint8_t addr, uint8_t *data)
{
    esp_err_t ret;

    /*If MSB of addr is set the host will read data from slave.
     *If MSB of addr is clear the host will write data on slave.
     */
    uint8_t instruction_to_read_data[2] = {0x80 | addr, 0xFF};
    spi_transaction_t trans_desc = {
        .flags = SPI_TRANS_USE_RXDATA,
        .tx_buffer = instruction_to_read_data,
        .rxlength = 16,
        .length = 16
    };
    ret = spi_device_polling_transmit(pn532->spi.dev, &trans_desc);

    *data = trans_desc.rx_data[1];
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return ret;
}

/*
static esp_err_t pn532_spi_write_command(pn532_t* pn532, uint8_t* command, uint8_t command_len) {
    (void) uart_flush(pn532->uart_port);

    size_t data_len = command_len + 1;
    uint8_t cmd[data_len + 6];

    cmd[0] = PN532_PREAMBLE;
    cmd[1] = PN532_STARTCODE1;
    cmd[2] = PN532_STARTCODE2;
    cmd[3] = data_len;
    cmd[4] = ~data_len + 1;
    cmd[5] = PN532_HOSTTOPN532;

    for(size_t i = 0; i < command_len; i++) {
        cmd[6 + i] = command[i];
    }

    uint8_t checksum = PN532_HOSTTOPN532;
    for(size_t i = 0; i < command_len; i++) {
        checksum += command[i];
    }
    checksum = ~checksum + 1;
    
    cmd[6 + command_len] = checksum;
    cmd[7 + command_len] = PN532_POSTAMBLE;

    #ifdef PN532_DEBUG
        ESP_LOGD(TAG, "writing command:");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, sizeof(cmd), ESP_LOG_DEBUG);
    #endif

    if(xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "failed to take mutex");
        return ESP_FAIL;
    }

    int written = uart_write_bytes(pn532->uart_port, (const char*) cmd, sizeof(cmd));
    if(written != sizeof(cmd)) {
        ESP_LOGE(TAG, "failed to write command");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

static esp_err_t pn532_spi_response(pn532_t* pn532, uint8_t* response, size_t response_len) {
    if(xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "failed to take mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);
    int len = uart_read_bytes(pn532->uart_port, response, response_len, (PN532_DELAY_DEFAULT));
    if(len < 0) {
        ESP_LOGE(TAG, "failed to read response");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

    #ifdef PN532_DEBUG
        ESP_LOGD(TAG, "reading response:");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, response, len, ESP_LOG_DEBUG);
    #endif

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}
*/


static esp_err_t pn532_spi_free(struct pn532_t* pn532) {
    esp_err_t err; 
    
    err = spi_bus_remove_device(pn532->spi.dev);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "spi device removal failed: %d", err);
        return err;
    }

    err = spi_bus_free(pn532->spi.bus);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "spi bus removal failed: %d", err);
        return err;
    }

    // vSemaphoreDelete(pn532->mutex);
    return ESP_OK;
}


/*  From the PN532_C1.pdf datasheet:

    The SPI has the following features:
    • Compliant with Motorola de-facto Serial Peripheral Interface (SPI) standard
    • Synchronous, Serial, Half-Duplex communication, 5 MHz max
    • Slave configuration
    • 8 bits bus interface
*/
esp_err_t pn532_spi_init(pn532_t* pn532, const pn532_spi_config_t* config) {

    spi_bus_config_t buscfg = {
        .mosi_io_num     = config->mosi,
        .miso_io_num     = config->miso,
        .sclk_io_num     = config->clk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = PN532_MAX_TRANSFER_SIZE,
        .flags           = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK
    };
    ESP_ERROR_CHECK(spi_bus_initialize(config->bus, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .spics_io_num   = config->cs,
        .clock_speed_hz = (config->clock_speed_hz > 5000 * 1000 ?
                            5000 * 1000 : 
                            config->clock_speed_hz),
        .mode           = 0,
        .queue_size     = 7,
//      .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
//      .flags  = 0
    };
    ESP_ERROR_CHECK(spi_bus_add_device(config->bus, &devcfg, &pn532->spi.dev));

    pn532->protocol = PN532_SPI_PROTOCOL;
    pn532->spi.bus = config->bus;
    pn532->free = pn532_spi_free;

    ESP_LOGI(TAG, "pn532 SPI initialized");
    return ESP_OK;
}
