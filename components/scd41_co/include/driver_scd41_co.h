#ifndef _SCD_41_H_
#define _SCD_41_H_

#include "driver/i2c.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <stdio.h>


#define WRITE_BIT    I2C_MASTER_WRITE
#define READ_BIT     I2C_MASTER_READ
#define ACK_CHECK_EN 0x01 

extern i2c_port_t scd_i2c_port;
extern uint8_t scd_i2c_address;



/**
 * @brief Initialize the SCD41 sensor.
 *
 * @param port I2C port number.
 * @param addr I2C address of the sensor.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_begin(i2c_port_t port, uint8_t addr);

/**
 * @brief Reinitialize the SCD41 sensor. wait time is 20ms
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_reinit();

/**
 * @brief Check if new measurement data is ready.
 *
 * @param arg0 Pointer to a bool that will be true if data is ready.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_is_data_ready(bool *arg0);

/**
 * @brief Read CO2, temperature, and humidity measurements.
 *
 * @param aCo2Concentration Pointer to CO2 concentration in ppm.
 * @param aTemperature Pointer to temperature in °C * 100.
 * @param aRelativeHumidity Pointer to relative humidity in %RH * 100.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_read_measurement(uint16_t *aCo2Concentration, float *aTemperature, float *aRelativeHumidity);

/**
 * @brief Read raw CO2, temperature, and humidity measurement values.
 *
 * @param co2Concentration Pointer to raw CO2 value.
 * @param temperature Pointer to raw temperature value.
 * @param relativeHumidity Pointer to raw humidity value.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_read_measurement_raw(uint16_t *co2Concentration, uint16_t *temperature, uint16_t *relativeHumidity);

/**
 * @brief Stop periodic measurement.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_stop_periodic_measurement();

/**
 * @brief Start periodic measurement data update after every 5 seconds.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_start_periodic_measurement();

/**
 * @brief Start low power periodic measurement data update after every 30 seconds.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_start_low_power_periodic_measurement();

/**
 * @brief Start a single-shot measurement on the SCD41 sensor.
 *
 * This function enables a single-shot measurement mode for CO₂, temperature, and humidity.
 * The sensor performs one measurement and returns to idle mode. Value update after 5 minutes
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd_41_enable_measure_single_shot();

/**
 * @brief Start a single-shot measurement of temperature and humidity only.
 *
 * This function enables a single-shot measurement mode that measures only
 * temperature and relative humidity (RHT) without CO₂. Useful for faster,
 * lower-power measurements. CO2 will always 0.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_enable_measure_single_shot_rht_only();

/**
 * @brief Perform a forced recalibration (FRC) of the CO₂ sensor.
 *
 * This function calibrates the CO₂ sensor to a given reference CO₂ concentration.
 * The result of the calibration (FRC correction value) is returned via the output pointer.
 *
 * @param reference_co2  The reference CO₂ concentration in ppm (400–2000 ppm recommended).
 * @param frc            Pointer to store the resulting FRC correction value.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_perform_forced_recalibration(uint16_t reference_co2, int *frc);

/**
 * @brief Enable or disable the automatic self-calibration (ASC) feature.
 *
 * ASC automatically adjusts the sensor baseline over time for better accuracy.
 * It should typically be enabled if the sensor operates in an environment
 * with regular fresh air exchange.
 *
 * @param enable  true to enable ASC, false to disable it.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_set_automatic_self_calibration_enabled(bool enable);

/**
 * @brief Get the current status of the automatic self-calibration (ASC) feature.
 *
 * This function reads whether ASC is currently enabled or disabled.
 *
 * @param value  Pointer to a bool to store the ASC status (true = enabled, false = disabled).
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_get_automatic_self_calibration_enabled(bool *value);

/**
 * @brief The perform_self_test feature can be used as an end-of-line test to confirm sensor functionality and the customer
 *        power supply to the sensor. wait time is 5500ms
 * @param status Pointer to a bool that will be true if test was done successfully.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_perform_self_test(bool *status);

/**
 * @brief The perform_factory_reset command resets all configuration settings stored in the EEPROM and erases the
 *        FRC and ASC algorithm history. wait time is 1200ms
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_perfom_factory_reset();

/**
 * @brief Configuration settings such as the temperature offset, sensor altitude and the ASC enabled/disabled parameter
 *        are by default stored in the volatile memory (RAM) only and will be lost after a power-cycle or when using the power_down
 *        command. The persist_settings command stores the current configuration in the EEPROM of the SCD4x, making them persistent
 *        across power-cycling. To avoid unnecessary wear of the EEPROM, the persist_settings command should only be sent when
 *        persistence is required and if actual changes to the configuration have been made (the EEPROM is guaranteed to endure at
 *        least 2000 write cycles before failure). Note that field calibration history is automatically
 *        stored in a separate EEPROM dimensioned for the specified sensor lifetime. 
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_persist_settings();

/**
 * @brief Set the temperature offset for the SCD41 sensor.
 *
 * This function sets a temperature offset to correct the measured temperature.
 * This can be used to compensate for self-heating or installation conditions.
 *
 * @param offset  Temperature offset in degrees Celsius.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_set_temperature_offset(float offset);

/**
 * @brief Set the sensor installation altitude.
 *
 * This function configures the sensor with its installation altitude above sea level,
 * in meters. This helps the sensor adjust its CO₂ measurement for air pressure.
 *
 * @param altitude  Altitude in meters.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_set_sensor_altitude(uint16_t altitude);

/**
 * @brief Set the current ambient air pressure.
 *
 * This function updates the ambient air pressure in millibars (hPa),
 * which helps the sensor improve CO₂ measurement accuracy.
 *
 * @param pressure  Ambient pressure in millibars (hPa).
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_set_ambient_pressure(uint16_t pressure);

/**
 * @brief Get the configured temperature offset.
 *
 * This function reads the currently set temperature offset from the sensor.
 *
 * @param temp  Pointer to a float to store the temperature offset in degrees Celsius.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL or other error code on failure
 */
esp_err_t scd41_get_temperature_offset(float *temp);

/**
 * @brief Create a command to send to the sensor.
 *
 * @param command 16-bit command code.
 */
void scd41_create_command(uint16_t command);

/**
 * @brief Create a command to write to the sensor.
 *
 * @param command 16-bit command code.
 */
void scd41_create_write_command(uint16_t command, uint8_t *data, uint8_t len);

/**
 * @brief Send a frame over I2C.
 *
 * @param buffer Pointer to frame data.
 * @param length Length of the frame.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_send_frame(uint8_t *buffer, uint8_t length);

/**
 * @brief Receive a frame over I2C.
 *
 * @param rx_frame Pointer to receive buffer.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t scd41_receive_frame(uint8_t *rx_frame);

/**
 * @brief Generate CRC for given data.
 *
 * @param data Pointer to data array.
 * @param len Length of data.
 * @return 8-bit CRC value.
 */
uint8_t generate_crc(uint8_t *data, uint8_t len);



#endif