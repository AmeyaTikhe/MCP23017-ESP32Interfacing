#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SDA_IO   23
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_FREQ_HZ  100000

#define MCP23017_ADDR       0x20

// MCP23017 Register addresses
#define IODIRA              0x00  // I/O Direction Register A
#define GPIOA               0x12  // GPIO Register A

#define LED_PIN             0     // GPA0

static const char *TAG = "I2C_LED";

// Global bus and device handles
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

esp_err_t mcp23017_write_register(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: reg=0x%02X, val=0x%02X, err=%s", 
                 reg, value, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t mcp23017_read_register(uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: reg=0x%02X, err=%s", reg, esp_err_to_name(ret));
    }
    return ret;
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

void mcp23017_init()
{
    ESP_LOGI(TAG, "Initializing MCP23017...");
    
    // Set GPA0 as output (0 = output, 1 = input)
    uint8_t iodira;
    if (mcp23017_read_register(IODIRA, &iodira) == ESP_OK) {
        ESP_LOGI(TAG, "Current IODIRA: 0x%02X", iodira);
        
        // Clear bit 0 to make GPA0 an output
        iodira &= ~(1 << LED_PIN);
        
        if (mcp23017_write_register(IODIRA, iodira) == ESP_OK) {
            ESP_LOGI(TAG, "GPA0 configured as output");
            
            // Verify
            uint8_t verify;
            if (mcp23017_read_register(IODIRA, &verify) == ESP_OK) {
                ESP_LOGI(TAG, "Verified IODIRA: 0x%02X", verify);
            }
        }
    }
}

void app_main()
{
    bool led_state = false;
    
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    mcp23017_init();
    
    ESP_LOGI(TAG, "Starting LED blink...");
    
    while (1) 
    {
        led_state = !led_state;
        
        // Write to GPIOA (0 = HIGH = LED ON, 1 = LOW = LED OFF)
        uint8_t gpio_value = led_state ? 0 : (1 << LED_PIN);
        
        if (mcp23017_write_register(GPIOA, gpio_value) == ESP_OK) {
            ESP_LOGI(TAG, "LED %s", led_state ? "ON" : "OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}