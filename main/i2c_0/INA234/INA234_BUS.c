#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "INA234"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      17
#define I2C_MASTER_SCL_IO      18
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TIMEOUT_MS  1000

#define INA234_ADDR            0x42

// Registers
#define REG_SHUNT_VOLTAGE      0x01
#define REG_BUS_VOLTAGE        0x02
#define REG_POWER              0x03
#define REG_CURRENT            0x04
#define REG_CALIBRATION        0x05

// Your shunt resistor
#define R_SHUNT                0.005f    // 5 mΩ

// Calculated for 6A full-scale
#define CURRENT_LSB            0.000183f         // A/bit
#define POWER_LSB              (25 * CURRENT_LSB) // W/bit
#define CALIBRATION_VALUE      0x15D6            // 5590

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t ina234_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t data[3] = { reg, val >> 8, val & 0xFF };
    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        INA234_ADDR,
        data, 3,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

static esp_err_t ina234_read_reg(uint8_t reg, uint16_t *out_val)
{
    uint8_t data[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        INA234_ADDR,
        &reg, 1,
        data, 2,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    if (err != ESP_OK) return err;

    *out_val = (data[0] << 8) | data[1];
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized.");

    // Write calibration
    ESP_ERROR_CHECK(ina234_write_reg(REG_CALIBRATION, CALIBRATION_VALUE));
    ESP_LOGI(TAG, "Calibration register written: 0x%04X", CALIBRATION_VALUE);

    while (1)
    {
        uint16_t bus_raw, current_raw, power_raw;

        ina234_read_reg(REG_BUS_VOLTAGE, &bus_raw);
        uint16_t bus_swapped = (bus_raw << 8) | (bus_raw >> 8);
        uint16_t bus_val = bus_swapped >> 3;
        float bus_voltage = bus_val * 0.00125f;

        ina234_read_reg(REG_CURRENT, &current_raw);
        float current = current_raw * CURRENT_LSB;

        ina234_read_reg(REG_POWER, &power_raw);
        float power = power_raw * POWER_LSB;

        ESP_LOGI(TAG, "Bus Voltage : %.4f V", bus_voltage);
        ESP_LOGI(TAG, "Current : %.6f A", current);
        ESP_LOGI(TAG, "Power   : %.6f W", power);
        ESP_LOGI(TAG, "---------------------------");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
