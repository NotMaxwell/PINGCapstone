#include "idf_config.h"

esp_err_t ping_uart_config_8n1(uart_port_t port, int baud)
{
    const uart_config_t config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    return uart_param_config(port, &config);
}

esp_err_t ping_i2c_new_master_bus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl,
                                  i2c_master_bus_handle_t *bus)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // The modules have their own pull-ups; these only help.
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, bus);
}

esp_err_t ping_i2c_add_device(i2c_master_bus_handle_t bus, uint16_t address, uint32_t speed_hz,
                              i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = speed_hz,
    };
    return i2c_master_bus_add_device(bus, &config, device);
}

esp_err_t ping_spi_bus_initialize(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso,
                                  gpio_num_t sclk)
{
    const spi_bus_config_t config = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    return spi_bus_initialize(host, &config, SPI_DMA_CH_AUTO);
}

esp_err_t ping_spi_add_device(spi_host_device_t host, int speed_hz, uint8_t mode,
                              spi_device_handle_t *device)
{
    const spi_device_interface_config_t config = {
        .mode = mode,
        .clock_speed_hz = speed_hz,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    return spi_bus_add_device(host, &config, device);
}

esp_err_t ping_spi_transfer(spi_device_handle_t device, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(device, &transaction);
}
