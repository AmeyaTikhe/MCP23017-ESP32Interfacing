#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO   23
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_FREQ_HZ  100000

#define MCP23017_ADDR       0x20
#define IODIRA              0x00

static const char *TAG = "I2C_TEST";

// global bus and device handles
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void i2c_master_init()
{
    // I2C master bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    
    // adding MCP23017 to the bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP23017_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    
    ESP_LOGI(TAG, "I2C initialized successfully");
}

void i2c_scan()
{
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t addr = 1; addr < 127; addr++) 
    {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, 100);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X", addr);
        }
    }

    ESP_LOGI(TAG, "Scan complete");
}

esp_err_t mcp23017_read(uint8_t reg, uint8_t *data)
{
    // Write register address, then read the data
    return i2c_master_transmit_receive(
        dev_handle,
        &reg,           // Register to read
        1,              // 1 byte to write
        data,           // Buffer for read data
        1,              // 1 byte to read
        1000            // Timeout in ms
    );
}

void app_main()
{
    uint8_t iodira = 0;

    i2c_master_init();

    i2c_scan();

    if (mcp23017_read(IODIRA, &iodira) == ESP_OK) 
    {
        ESP_LOGI(TAG, "MCP23017 detected!");
        ESP_LOGI(TAG, "IODIRA = 0x%02X (expected 0xFF)", iodira);
    } 
    else 
    {
        ESP_LOGE(TAG, "Failed to read MCP23017");
    }
}