#include "pn532_types.h"

#define PN532_UART_RX_BUF_SIZE 256
#define PN532_UART_TX_BUF_SIZE 256

static const char *TAG = "pn532";

/**
 * @brief Write a command to the PN532 over UART
 *
 * This function constructs a valid PN532 command frame and sends it via UART.
 *
 * @param[in] pn532 Pointer to the PN532 structure
 * @param[in] command Command data buffer
 * @param[in] command_len Length of the command data
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t pn532_uart_write_command(pn532_t *pn532, uint8_t *command, uint8_t command_len)
{
    // Clear the UART RX buffer
    uart_flush(pn532->uart_port);

    size_t data_len = command_len + 1;
    uint8_t cmd[data_len + 7];

    // Construct the command frame
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
    ESP_LOGD(TAG, "Writing command:");
    ESP_LOG_BUFFER_HEX(TAG, cmd, sizeof(cmd));
#endif

    // Acquire mutex for thread-safe UART operation
    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }

    // Send the command frame via UART
    int written = uart_write_bytes(pn532->uart_port, (const char *)cmd, sizeof(cmd));
    if (written != sizeof(cmd))
    {
        ESP_LOGE(TAG, "Failed to write command");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

/**
 * @brief Read a response from the PN532 over UART
 *
 * This function reads the response frame from the PN532.
 *
 * @param[in] pn532 Pointer to the PN532 structure
 * @param[out] response Buffer to store the response data
 * @param[in] response_len Length of the response buffer
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t pn532_uart_read_response(pn532_t *pn532, uint8_t *response, uint8_t response_len)
{
    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);

    // Read response from UART
    int len = uart_read_bytes(pn532->uart_port, response, response_len, PN532_DELAY_DEFAULT);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Failed to read response");
        xSemaphoreGive(pn532->mutex);
        return ESP_FAIL;
    }

#ifdef PN532_DEBUG
    ESP_LOGD(TAG, "Reading response:");
    ESP_LOG_BUFFER_HEX(TAG, response, len);
#endif

    xSemaphoreGive(pn532->mutex);
    return ESP_OK;
}

/**
 * @brief Free resources allocated for the PN532 UART interface
 *
 * @param[in] pn532 Pointer to the PN532 structure
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t pn532_uart_free(pn532_t *pn532)
{
    esp_err_t err = uart_driver_delete(pn532->uart_port);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete UART driver: %d", err);
        return err;
    }

    vSemaphoreDelete(pn532->mutex);
    return ESP_OK;
}

/**
 * @brief Initialize the PN532 UART interface
 *
 * @param[in,out] pn532 Pointer to the PN532 structure
 * @param[in] config UART configuration parameters
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t pn532_uart_init(pn532_t *pn532, const pn532_uart_config_t *config)
{
    pn532->protocol = PN532_UART_PROTOCOL;
    pn532->uart_port = config->uart_port;

    // Configure the reset pin as output
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&reset_gpio_config);

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};

    esp_err_t err = uart_param_config(pn532->uart_port, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %d", err);
        goto ERR;
    }

    // Set UART pins
    err = uart_set_pin(pn532->uart_port, config->tx, config->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %d", err);
        goto ERR;
    }

    // Install UART driver
    err = uart_driver_install(pn532->uart_port, PN532_UART_RX_BUF_SIZE, PN532_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %d", err);
        goto ERR;
    }

    uart_flush(pn532->uart_port);

    // Perform hardware reset
    gpio_set_level(config->rst_pin, 1);
    gpio_set_level(config->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    gpio_set_level(config->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create mutex for thread-safe operations
    pn532->mutex = xSemaphoreCreateMutex();
    if (!pn532->mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        uart_driver_delete(pn532->uart_port);
        return ESP_ERR_NO_MEM;
    }

    // Assign function pointers
    pn532->write_command = pn532_uart_write_command;
    pn532->read_response = pn532_uart_read_response;
    pn532->free = pn532_uart_free;

    ESP_LOGI(TAG, "PN532 UART initialized");
    return ESP_OK;

ERR:
    return err;
}
