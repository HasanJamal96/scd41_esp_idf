
#include <stdio.h>
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/i2c.h"
#include "driver_scd41_co.h"


#define I2C_SDA     5
#define I2C_SCL     4
#define I2C_FREQ    100000 // 100k
#define SCD_41_ADDR 0x62
i2c_port_t I2C_PORT = I2C_NUM_0;        

static const char *TAG = "Main";

esp_err_t initialize_i2c() {
    ESP_LOGI(TAG, "Initializing I2C");
    i2c_config_t i2c_config;
    i2c_config.sda_io_num = I2C_SDA;
    i2c_config.scl_io_num = I2C_SCL;
    i2c_config.master.clk_speed = I2C_FREQ;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.mode = I2C_MODE_MASTER;

    i2c_param_config(I2C_PORT, &i2c_config);
    esp_err_t ret = i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);
    if(ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C initialized successfully");
    }
    else {
        ESP_LOGE(TAG, "I2C initialized failed, Error: %s", esp_err_to_name(ret));
    }
    return ret;
}



void run_scan_indefinate() {
    esp_err_t res;
    bool dataReady;
    while (1) {
        dataReady = false;
        vTaskDelay(pdMS_TO_TICKS(5000));
        while (!dataReady) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_err_t error = scd41_is_data_ready(&dataReady);
            if (error == ESP_OK) {
                dataReady = true;
            }
            else {
                ESP_LOGI(TAG, "SCD41 Data is not ready");
            }
        }
        uint16_t co;
        float humi, temp = 0;
        res = scd41_read_measurement(&co, &temp, &humi);
        if(res != ESP_OK) {
            ESP_LOGI(TAG, "SCD41 Error reading data");
        }
        else {
            printf("CO2: %dppm, Temperature: %.2fC, Humidity: %.2fRH \n", co, temp, humi);
        }
    }
}


void app_main(void) {
    if(initialize_i2c() == ESP_OK) {
        scd41_begin(I2C_PORT, SCD_41_ADDR); 
    }
    xTaskCreatePinnedToCore(run_scan_indefinate, "", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

}
