#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_1/lvgl_ui/lvgl_init.h"
#include "init_func/init_func.h"

#define TAG "INA234"

#define INA234_ADDR            0x42

#define REG_SHUNT_VOLTAGE      0x01
#define REG_BUS_VOLTAGE        0x02
#define REG_POWER              0x03
#define REG_CURRENT            0x04
#define REG_CALIBRATION        0x05

#define R_SHUNT                0.005f

#define CURRENT_LSB            0.000183f
#define POWER_LSB              (25 * CURRENT_LSB)
#define CALIBRATION_VALUE      0x15D6

// 写寄存器
static esp_err_t ina234_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

// 读寄存器
static esp_err_t ina234_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out_val) {
    uint8_t buf[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, 2, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return err;

    *out_val = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

void ina234_bus_thread(void *arg) {

    char buf[32];
    ESP_ERROR_CHECK(ina234_write_reg(ina234_bus, REG_CALIBRATION, CALIBRATION_VALUE));
    ESP_LOGI(TAG, "Calibration register written: 0x%04X", CALIBRATION_VALUE);

    while (1) {
        uint16_t bus_raw, current_raw, power_raw;

        ina234_read_reg(ina234_bus, REG_BUS_VOLTAGE, &bus_raw);
        uint16_t bus_swapped = (bus_raw << 8) | (bus_raw >> 8);
        uint16_t bus_val = bus_swapped >> 3;
        float bus_voltage = bus_val * 0.00125f;

        ina234_read_reg(ina234_bus, REG_CURRENT, &current_raw);
        float current = current_raw * CURRENT_LSB;

        ina234_read_reg(ina234_bus, REG_POWER, &power_raw);
        float power = power_raw * POWER_LSB;

        snprintf(buf, sizeof(buf), "Bus Power STAT:");
        update_label_text(10, buf);
        snprintf(buf, sizeof(buf), "U: %.2fV", bus_voltage);
        update_label_text(11, buf);
        snprintf(buf, sizeof(buf), "I: %.2fA", current);
        update_label_text(12, buf);
        snprintf(buf, sizeof(buf), "P: %.2fW", power);
        update_label_text(13, buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
