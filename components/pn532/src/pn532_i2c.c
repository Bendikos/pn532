#include "pn532_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Logger tag for debugging
static const char *TAG = "pn532_i2c";

// Write a command to the PN532 via I2C
static esp_err_t pn532_i2c_write_command(pn532_t *pn532, uint8_t *command, uint8_t command_len)
{
    size_t data_len = command_len + 1; // Include TFI in the length
    uint8_t cmd[data_len + 7];         // Allocate memory for the full frame

    // Build the command frame
    cmd[0] = PN532_PREAMBLE;
    cmd[1] = PN532_STARTCODE1;
    cmd[2] = PN532_STARTCODE2;
    cmd[3] = data_len;
    cmd[4] = ~data_len + 1; // Checksum of the length
    cmd[5] = PN532_HOSTTOPN532;

    memcpy(&cmd[6], command, command_len); // Copy command data

    // Calculate checksum
    uint8_t checksum = PN532_HOSTTOPN532;
    for (size_t i = 0; i < command_len; i++)
    {
        checksum += command[i];
    }
    checksum = ~checksum + 1;
    cmd[6 + command_len] = checksum;
    cmd[7 + command_len] = PN532_POSTAMBLE;

#ifdef PN532_DEBUG
    ESP_LOGE(TAG, "Writing command:");
    ESP_LOG_BUFFER_HEX(TAG, cmd, sizeof(cmd));
#endif

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    // Send the command via I2C
    esp_err_t err = i2c_master_transmit(pn532->i2c.i2c_dev, cmd, sizeof(cmd), PN532_DELAY_DEFAULT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write command");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

// Read a response from the PN532 via I2C
static esp_err_t pn532_i2c_read_response(pn532_t *pn532, uint8_t *response, uint8_t response_len)
{
    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);

    // look up PN532_datasheet.pdf page 42
    uint8_t data[response_len + 1]; // Include header byte
    esp_err_t err = i2c_master_receive(pn532->i2c.i2c_dev, data, response_len + 1, PN532_DELAY_DEFAULT);

    if (data[0] != 0x01)
    { // Ensure the first byte indicates success
        err = ESP_FAIL;
    }

    memcpy(response, &data[1], response_len); // Copy the actual response

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read response");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

#ifdef PN532_DEBUG
    ESP_LOGE(TAG, "Reading response:");
    ESP_LOG_BUFFER_HEX(TAG, response, response_len);
#endif

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

// Free resources associated with the PN532 I2C interface
static esp_err_t pn532_i2c_free(struct pn532_t *pn532)
{
    esp_err_t err;

    err = i2c_master_bus_rm_device(pn532->i2c.i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C device removal failed: %d", err);
        return err;
    }

    err = i2c_del_master_bus(pn532->i2c.i2c_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus removal failed: %d", err);
        return err;
    }

    // vSemaphoreDelete(pn532->mutex);  // Uncomment if required
    return ESP_OK;
}

// Initialize the PN532 I2C interface
esp_err_t pn532_i2c_init(pn532_t *pn532, const pn532_i2c_config_t *config)
{
    pn532->protocol = PN532_I2C_PROTOCOL;
    pn532->i2c.i2c_port = config->i2c_port;

    // Configure the reset pin
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&reset_gpio_config);

    // Configure the I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config->i2c_port,
        .scl_io_num = config->i2c_scl,
        .sda_io_num = config->i2c_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &pn532->i2c.i2c_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus");
        return ESP_FAIL;
    }

    // Add the PN532 device
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = config->i2c_speed};

    err = i2c_master_bus_add_device(pn532->i2c.i2c_bus, &i2c_device_config, &pn532->i2c.i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ESP_FAIL;
    }

    // Perform reset
    gpio_set_level(config->rst_pin, 1);
    gpio_set_level(config->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    gpio_set_level(config->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create mutex for thread safety
    pn532->mutex = xSemaphoreCreateMutex();
    if (!pn532->mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        pn532_i2c_free(pn532);
        return ESP_ERR_NO_MEM;
    }

    // Assign function pointers
    pn532->write_command = pn532_i2c_write_command;
    pn532->read_response = pn532_i2c_read_response;
    pn532->free = pn532_i2c_free;

    ESP_LOGI(TAG, "PN532 I2C initialized");
    return ESP_OK;
}
