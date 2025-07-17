# ESP-IDF SCD41 CO₂ Sensor Driver

A simple ESP-IDF driver for the **Sensirion SCD41** CO₂ sensor, organized as a reusable component.

---

## 📌 Overview

This project demonstrates how to use the **SCD41** sensor with an **ESP32** using ESP-IDF.  
The driver is placed in `components/scd41_co/` and can be reused in other projects as a drop-in component.

---

## ⚙️ Project Structure

```
scd41_esp_idf/
 ├── components/
 │   └── scd41_co/
 │       ├── include/
 │       │   └── driver_scd41_co.h
 │       ├── driver_scd41_co.c
 │       └── CMakeLists.txt
 ├── main/
 │   ├── main.c
 │   └── CMakeLists.txt
 ├── ...
```

- **components/scd41_co/**: Contains the SCD41 driver.
- **main/**: Contains your application’s entry point.

---

## 🚀 Getting Started

1. **Clone the repo**  
   ```bash
   git clone https://github.com/HasanJamal96/scd41_esp_idf.git
   cd scd41_esp_idf
   ```

2. **Configure I²C**  
   Make sure your `main.c` initializes the I²C bus before using the driver.

3. **Build & Flash**  
   ```bash
   idf.py build
   idf.py flash
   idf.py monitor
   ```

---

## 📌 Example Usage

Your `main.c` might look like:

```c

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

```

---

## 🗂️ Component `CMakeLists.txt`

In `components/scd41_co/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "driver_scd41_co.c"
                       INCLUDE_DIRS "include")
```

In `main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES scd41_co)
```

Root `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(scd41_esp_idf)
```

---

## 📄 API

| Function | Description |
| -------- | ----------- |
| `esp_err_t scd41_init(void);` | Initialize sensor |
| `esp_err_t scd41_start_periodic_measurement(void);` | Start periodic measurement |
| `esp_err_t scd41_stop_measurement(void);` | Stop measurement |
| `esp_err_t scd41_read_measurement(float *co2, float *temperature, float *humidity);` | Read measurement data |

---

## 🧑‍💻 Author

**Hasan Jamal**  
[Your GitHub Profile](https://github.com/HasanJamal96)
