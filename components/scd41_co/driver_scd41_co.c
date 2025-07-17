#include "driver_scd41_co.h"


i2c_port_t scd_i2c_port    = I2C_NUM_0;
uint8_t    scd_i2c_address = 0;
static const char *TAG = "SCD 41";
static uint8_t communication_buffer[9] = {0};




uint8_t generate_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}


void scd41_create_command(uint16_t command) {
    memset(communication_buffer, 0, sizeof(communication_buffer));
    communication_buffer[0] = ((command & 0xFF00) >> 8);
    communication_buffer[1] = ((command & 0x00FF) >> 0);
}


void scd41_create_write_command(uint16_t command, uint8_t *data, uint8_t len) {
    memset(communication_buffer, 0, sizeof(communication_buffer));
    communication_buffer[0] = ((command & 0xFF00) >> 8);
    communication_buffer[1] = ((command & 0x00FF) >> 0);
    for(uint8_t i=0; i<len; i++) {
        communication_buffer[i+2] = data[i];
    }

}


esp_err_t scd41_send_frame(uint8_t *buffer, uint8_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( scd_i2c_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, buffer, length, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(scd_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("[I2C] [send_frame]", "Failed to read i2c response: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ret;
}


esp_err_t scd41_receive_frame(uint8_t *rx_frame) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (scd_i2c_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, rx_frame, 9, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(scd_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE("[I2C] [receive_frame]", "Failed to read i2c response: %s", esp_err_to_name(ret));
        return ret;
    }
    return ret;
}


esp_err_t scd41_stop_periodic_measurement() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x3f86);
    ret = scd41_send_frame(communication_buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return ret;
}


esp_err_t scd41_start_periodic_measurement() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x21b1);
    ret = scd41_send_frame(communication_buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}


esp_err_t scd_41_enable_measure_single_shot() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x219d);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(1350));
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}


esp_err_t scd41_enable_measure_single_shot_rht_only() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x2196);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}


esp_err_t scd41_start_low_power_periodic_measurement() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x21ac);
    ret = scd41_send_frame(communication_buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}


esp_err_t scd41_set_temperature_offset(float offset) {
    esp_err_t ret = ESP_OK;
    uint8_t _offset[3] = {0};
    uint16_t value = (offset * 65535)/175;
    _offset[0] = (value >> 8) & 0xFF;
    _offset[1] = value & 0xFF;
    _offset[2] = generate_crc(_offset, 2);
    scd41_create_write_command(0x241D, _offset, 3);
    ret = scd41_send_frame(communication_buffer, 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}


esp_err_t scd41_get_temperature_offset(float *temp) {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x2318);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(1));
    if(ret != ESP_OK)
        return ret;
    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t crc = generate_crc(receive_buffer, 2);
    
    if(crc != receive_buffer[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t value = (receive_buffer[0] << 8) | receive_buffer[1];
    *temp = 175.0 * value / 65535.0;
    return ret;
}


esp_err_t scd41_set_sensor_altitude(uint16_t altitude) {
    esp_err_t ret = ESP_OK;
    uint8_t _offset[3] = {0};
    _offset[0] = (altitude >> 8) & 0xFF;
    _offset[1] = altitude & 0xFF;
    _offset[2] = generate_crc(_offset, 2);
    scd41_create_write_command(0x2427, _offset, 3);
    ret = scd41_send_frame(communication_buffer, 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}


esp_err_t scd41_get_sensor_altitude(uint16_t *alt) {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x2322);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(1));
    if(ret != ESP_OK)
        return ret;
    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t crc = generate_crc(receive_buffer, 2);
    
    if(crc != receive_buffer[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    *alt = (receive_buffer[0] << 8) | receive_buffer[1];
    return ret;
}


esp_err_t scd41_set_ambient_pressure(uint16_t pressure) {
    esp_err_t ret = ESP_OK;
    uint8_t _offset[3] = {0};
    pressure = pressure/100;
    _offset[0] = (pressure >> 8) & 0xFF;
    _offset[1] = pressure & 0xFF;
    _offset[2] = generate_crc(_offset, 2);
    scd41_create_write_command(0x2427, _offset, 3);
    ret = scd41_send_frame(communication_buffer, 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}


esp_err_t scd41_perform_self_test(bool *status) {
    esp_err_t ret = ESP_OK;
    *status = false;
    scd41_create_command(0x3639);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdTICKS_TO_MS(5500));
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t crc = generate_crc(receive_buffer, 2);
    
    if(crc != receive_buffer[2]) {
        return ESP_ERR_INVALID_CRC;
    }
    uint16_t dummy = 0;

    dummy = receive_buffer[0];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[1];
    if(dummy == 0) *status = true;
    return ret;
}


esp_err_t scd41_perfom_factory_reset() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x3632);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdTICKS_TO_MS(1200));
    return ret;
}


esp_err_t scd41_perform_forced_recalibration(uint16_t reference_co2, int *frc) {
    esp_err_t ret = ESP_OK;
    uint8_t _data[3] = {0};
    _data[0] = (reference_co2 >> 8) & 0xFF;
    _data[1] = reference_co2 & 0xFF;
    _data[2] = generate_crc(_data, 2);
    scd41_create_write_command(0x362f, _data, 3);
    ret = scd41_send_frame(communication_buffer, 3);
    vTaskDelay(pdMS_TO_TICKS(400));
    if(ret != ESP_OK)
        return ret;

    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t crc = generate_crc(receive_buffer, 2);
    
    if(crc != receive_buffer[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    
    *frc = (receive_buffer[0] << 8) | receive_buffer[1];
    if(*frc == 0x8000)
        return ESP_ERR_INVALID_RESPONSE;
    *frc = *frc - 0x80000;

    return ret;
}

esp_err_t scd41_set_automatic_self_calibration_enabled(bool enable){
    esp_err_t ret = ESP_OK;
    uint8_t _data[3] = {0};
    _data[0] = 0;
    _data[1] = enable ? 0x01 : 0x00;
    _data[2] = generate_crc(_data, 2);
    scd41_create_write_command(0x2416, _data, 3);
    ret = scd41_send_frame(communication_buffer, 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}

esp_err_t scd41_get_automatic_self_calibration_enabled(bool *value) {
    esp_err_t ret = ESP_OK;
    *value = false;
    scd41_create_command(0x2313);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdTICKS_TO_MS(1));
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t crc = generate_crc(receive_buffer, 2);
    
    if(crc != receive_buffer[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t dummy = 0;

    dummy = receive_buffer[0];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[1];
    if(dummy == 0x0001) *value = true;
    return ret;
}


esp_err_t scd41_persist_settings() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x3615);
    ret = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdTICKS_TO_MS(800));
    return ret;
}


esp_err_t scd41_reinit() {
    esp_err_t ret = ESP_OK;
    scd41_create_command(0x3646);
    ret = scd41_send_frame(communication_buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    return ret;
}


esp_err_t scd41_is_data_ready(bool *arg0) {
    esp_err_t ret = 0;
    scd41_create_command(0xe4b8);
    ret = scd41_send_frame(communication_buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    uint8_t receive_buffer[9] = {0};
    ret = scd41_receive_frame(receive_buffer);
    if (ret != ESP_OK) {
        return ret;
    }
    uint16_t dummy = 0;

    dummy = receive_buffer[0];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[1];
    
    if (ret != ESP_OK) {
        return ret;
    }
    *arg0 = (dummy & 2047) != 0;
    return ret;
}


esp_err_t scd41_read_measurement_raw(uint16_t *co2Concentration, uint16_t *temperature, uint16_t *relativeHumidity) {
    esp_err_t err = ESP_OK;
    scd41_create_command(0xec05);
    err = scd41_send_frame(communication_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (err != ESP_OK) {
        return err;
    }
    uint8_t receive_buffer[9] = {0};
    err = scd41_receive_frame(receive_buffer);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t dummy = 0;
    uint8_t buffer_for_crc_check[2] = {};

    for(uint8_t i=0; i<9; i+=3) {
        buffer_for_crc_check[0] = receive_buffer[i];
        buffer_for_crc_check[1] = receive_buffer[i+1];
        
        uint8_t crc = generate_crc(buffer_for_crc_check, 2);
        
        if(crc != receive_buffer[i+2]) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    dummy = receive_buffer[0];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[1];
    *co2Concentration = dummy;

    dummy = receive_buffer[3];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[4];
    *temperature = dummy;

    dummy = receive_buffer[6];
    dummy = (dummy << 8);
    dummy = dummy | receive_buffer[7];
    *relativeHumidity = dummy;
    return err;
}


esp_err_t scd41_read_measurement(uint16_t *aCo2Concentration, float *aTemperature, float *aRelativeHumidity) {
    uint16_t rawTemperature      = 0;
    uint16_t rawRelativeHumidity = 0;
    uint16_t rawCo2Concentration = 0;
    esp_err_t err = scd41_read_measurement_raw(&rawCo2Concentration, &rawTemperature, &rawRelativeHumidity);
    if (err != ESP_OK) {
        return err;
    }
    *aCo2Concentration = rawCo2Concentration;
    *aTemperature      = -45.0 + ((175.0 * rawTemperature) / 65535.0);
    *aRelativeHumidity = (100.0 * rawRelativeHumidity) / 65535.0;
    return err;
}


esp_err_t scd41_begin(i2c_port_t port, uint8_t addr) {
    scd_i2c_port = port;
    scd_i2c_address = addr;
    esp_err_t ret;
    ret = scd41_stop_periodic_measurement();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Unable to stop periodic measurements");
        return ret;
    }
    ret = scd41_reinit();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Reinit failed");
        return ret;
    }
    ret = scd41_start_low_power_periodic_measurement();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Unable to start periodic measurements");
        return ret;
    }
    return ret;
}