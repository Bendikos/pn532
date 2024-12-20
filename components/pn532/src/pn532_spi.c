#include "pn532_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPI_CS_PIN 5
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SPI_CLK_PIN 18

static const char *TAG = "pn532_spi";

// SPI sends data to a specific address
esp_err_t _spi_write_data(pn532_t *pn532, uint8_t addr, uint8_t *cmd, uint8_t cmd_length)
{
    gpio_set_level(SPI_CS_PIN, 0); // Pull CS low to select the device
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay

    gpio_set_level(SPI_CLK_PIN, 1); // Ensure the clock starts high

    // Transmit the address byte bit by bit
    for (size_t i = 0; i < 8; i++)
    {
        gpio_set_level(SPI_MOSI_PIN, (addr >> i) & 1); // Set MOSI according to the bit value
        gpio_set_level(SPI_CLK_PIN, 0);                // Toggle clock low
        gpio_set_level(SPI_CLK_PIN, 1);                // Toggle clock high
    }

    // Transmit the command bytes
    for (size_t i = 0; i < cmd_length; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            gpio_set_level(SPI_MOSI_PIN, (cmd[i] >> j) & 1); // Set MOSI
            gpio_set_level(SPI_CLK_PIN, 0);                  // Toggle clock low
            gpio_set_level(SPI_CLK_PIN, 1);                  // Toggle clock high
        }
    }

    gpio_set_level(SPI_CS_PIN, 1); // Pull CS high to deselect the device
    return ESP_OK;
}

// SPI reads data from a specific address
esp_err_t _spi_read_data(pn532_t *pn532, uint8_t addr, uint8_t *response, uint8_t response_length)
{
    gpio_set_level(SPI_CS_PIN, 0); // Pull CS low to select the device
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay

    uint8_t x = 0;
    gpio_set_level(SPI_CLK_PIN, 1); // Ensure the clock starts high

    // Transmit the address byte bit by bit
    for (size_t i = 0; i < 8; i++)
    {
        gpio_set_level(SPI_MOSI_PIN, (addr >> i) & 1); // Set MOSI
        gpio_set_level(SPI_CLK_PIN, 0);                // Toggle clock low
        gpio_set_level(SPI_CLK_PIN, 1);                // Toggle clock high
    }

    // Receive response bytes
    for (size_t i = 0; i < response_length; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            if (gpio_get_level(SPI_MISO_PIN))
            {
                x |= (1 << j); // Update response bit by bit
            }
            gpio_set_level(SPI_CLK_PIN, 0); // Toggle clock low
            gpio_set_level(SPI_CLK_PIN, 1); // Toggle clock high
        }
        response[i] = x; // Store received byte
        x = 0x00;        // Reset for next byte
    }

    gpio_set_level(SPI_CS_PIN, 1); // Pull CS high to deselect the device
    return ESP_OK;
}

// Write a command to the PN532 via SPI
static esp_err_t pn532_spi_write_command(pn532_t *pn532, uint8_t *command, uint8_t command_len)
{
    size_t data_len = command_len + 1;
    uint8_t cmd[data_len + 7];

    // Construct command frame
    cmd[0] = PN532_PREAMBLE;
    cmd[1] = PN532_STARTCODE1;
    cmd[2] = PN532_STARTCODE2;
    cmd[3] = data_len;
    cmd[4] = ~data_len + 1;
    cmd[5] = PN532_HOSTTOPN532;

    for (size_t i = 0; i < command_len; i++)
    {
        cmd[6 + i] = command[i];
    }

    uint8_t checksum = PN532_HOSTTOPN532;
    for (size_t i = 0; i < command_len; i++)
    {
        checksum += command[i];
    }
    checksum = ~checksum + 1;
    cmd[6 + command_len] = checksum;
    cmd[7 + command_len] = PN532_POSTAMBLE;

#ifdef PN532_DEBUG
    ESP_LOGE(TAG, "writing command:");
    ESP_LOG_BUFFER_HEX(TAG, cmd, sizeof(cmd));
#endif

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "failed to take mutex");
        return ESP_FAIL;
    }

    esp_err_t err = _spi_write_data(pn532, 0x01, cmd, sizeof(cmd));
    xSemaphoreGive(pn532->mutex);

    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

// Read a response from the PN532 via SPI
static esp_err_t pn532_spi_read_response(pn532_t *pn532, uint8_t *response, uint8_t response_len)
{
    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "failed to take mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);

    uint8_t is_ready = 0x00;
    esp_err_t err = _spi_read_data(pn532, 0x02, &is_ready, 1);

    // look up PN532_datasheet.pdf page 45
    if (is_ready == 0x00)
    {
        err = ESP_FAIL;
    }

    if (err != ESP_OK)
    {
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

    err = _spi_read_data(pn532, 0x03, response, response_len);
    if (err != ESP_OK)
    {
        xSemaphoreGive(pn532->mutex);
        ESP_LOGE(TAG, "Failed to read response");
        return ESP_FAIL;
    }

#ifdef PN532_DEBUG
    ESP_LOGE(TAG, "reading response:");
    ESP_LOG_BUFFER_HEX(TAG, response, response_len);
#endif

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

// Free SPI resources
static esp_err_t pn532_spi_free(struct pn532_t *pn532)
{
    // esp_err_t err = spi_bus_remove_device(pn532->spi.spi_dev);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "spi device removal failed: %d", err);
    //     return err;
    // }

    // err = spi_bus_free(pn532->spi.spi_port);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "spi bus removal failed: %d", err);
    //     return err;
    // }
    vSemaphoreDelete(pn532->mutex);
    return ESP_OK;
}

// Initialize PN532 with SPI protocol
esp_err_t pn532_spi_init(pn532_t *pn532, const pn532_spi_config_t *config)
{
    pn532->protocol = PN532_SPI_PROTOCOL;
    pn532->spi.spi_port = config->port;
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&reset_gpio_config);

    esp_rom_gpio_pad_select_gpio(config->clk);
    esp_rom_gpio_pad_select_gpio(config->miso);
    esp_rom_gpio_pad_select_gpio(config->mosi);
    esp_rom_gpio_pad_select_gpio(config->cs);

    gpio_set_direction(config->cs, GPIO_MODE_OUTPUT);
    gpio_set_level(config->cs, 1);
    gpio_set_direction(config->clk, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->mosi, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->miso, GPIO_MODE_INPUT);

    gpio_set_level(config->rst_pin, 1);
    gpio_set_level(config->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    gpio_set_level(config->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    pn532->mutex = xSemaphoreCreateMutex();
    if (!pn532->mutex)
    {
        ESP_LOGE(TAG, "failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    pn532->write_command = pn532_spi_write_command;
    pn532->read_response = pn532_spi_read_response;
    pn532->free = pn532_spi_free;

    ESP_LOGI(TAG, "pn532 SPI initialized");
    return ESP_OK;
}
