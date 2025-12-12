#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_0/TMP117/TMP117.h"
#include "i2c_0/MP4245/MP4245.h"
#include "i2c_0/INA234/INA234_BUS.h"

#include "i2c_1/lvgl_ui/lvgl_init.h"

#include "init_func/init_func.h"

#include "ws2812/ws2812.h"

#include "fan_ctl/pwm_ctl.h"

#include "driver/gpio.h"

static const char *TAG = "Controller Main Thread";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing R-SODIUM Ultra Enclosure 2.0 Controller");
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "Initialize WS2812 (Process LED)");
    xTaskCreate(ws2812_thread, "WS2812_THREAD", 4*1024, 0, 2, NULL);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_7) | (1ULL << GPIO_NUM_10) | (1ULL << GPIO_NUM_9) | (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_14),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NUM_3, 0x00);
    gpio_set_level(GPIO_NUM_7, 0x01);
    gpio_set_level(GPIO_NUM_10, 0x00);

    gpio_set_level(GPIO_NUM_9, 0x01);
    gpio_set_level(GPIO_NUM_11, 0x01);
    gpio_set_level(GPIO_NUM_14, 0x01);

    ESP_LOGI(TAG, "Initialize TMP117 (Temperature Sensor)");
    xTaskCreate(tmp117_thread, "TMP117_THREAD", 4*1024, 0, 2, NULL);

    ESP_LOGI(TAG, "Initialize Fan Control (PWM + RPM)");

    fan_pwm_init();
    fan_rpm_init();

    xTaskCreate(fan_rpm_task, "fam_rpm_task", 4096, NULL, 5, NULL);
    xTaskCreate(fan_temp_task, "fam_temp_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialize MP4245");
    xTaskCreate(mp4245_thread, "MP4245_THREAD", 4*1024, 0, 2, NULL);

    ESP_LOGI(TAG, "Initialize INA234");
    xTaskCreate(ina234_bus_thread, "INA234_BUS_THREAD", 4*1024, 0, 2, NULL);
    
    lv_init();
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);
    xTaskCreate(show_ui, "LVGL_UI", 16*1024, display, 2, NULL);

}