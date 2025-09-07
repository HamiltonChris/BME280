/*
 * @file bme280.c
 *
 * @author Chris Hamilton
 * 
 * @brief implementation for bme280 hummidity, pressure, and temperature driver
 */

#include "bme280.h"

static void burst_read(bme280_t* config, int32_t* raw_pressure, int32_t* raw_temperature, int16_t* raw_humidity);
static void read_calibration_data(bme280_t* config);
static void write_configuration(const bme280_t* config);
static int32_t compensate_temperature(bme280_t* config, int32_t adc_T);
static uint32_t compensate_presure(const bme280_t* config, int32_t adc_P);
static uint32_t compensate_humidity(const bme280_t* config, int32_t adc_H);

int8_t bme280_init(bme280_t* config)
{
    uint8_t chip_id = 0;
    uint8_t reg_address = BME280_ID_REG;

    // read chip ID
    config->send(BME280_I2C_ADDRESS, &reg_address, 1);
    config->receive(BME280_I2C_ADDRESS, &chip_id, 1);

    // check chip ID equals what is expected
    if (chip_id != BME280_CHIP_ID)
    {
        // Error with BME280
        return -1;
    }

    // get compensation values
    read_calibration_data(config);

    // make configurations
    write_configuration(config);

    return 0;
}

int8_t bme280_read_all(bme280_t* config, uint32_t* pressure, uint32_t* humidity, int32_t* temperature)
{
    int32_t raw_pressure = 0;
    int32_t raw_temperature = 0;
    int16_t raw_humidity = 0;

    if (!config || !config->send || !config->receive)
    {
        // Error with configuration struct
        return -1;
    }

    burst_read(config, &raw_pressure, &raw_temperature, &raw_humidity);
    *temperature = compensate_temperature(config, raw_temperature);
    *pressure = compensate_presure(config, raw_pressure);
    *humidity = compensate_humidity(config, raw_humidity);

    return 0;
}

static void read_calibration_data(bme280_t* config)
{
    uint8_t data_buffer[BME280_CALIBRATION_LENGTH] = { 0 };
    uint8_t reg_address = BME280_CALIB00_REG;
    uint8_t data_length = BME280_CALIB23_REG - BME280_CALIB00_REG + 1;
    uint8_t data_offset = data_length;

    config->send(BME280_I2C_ADDRESS, &reg_address, 1);
    config->receive(BME280_I2C_ADDRESS, data_buffer, data_length);

    reg_address = BME280_CALIB25_REG;
    config->send(BME280_I2C_ADDRESS, &reg_address, 1);
    config->receive(BME280_I2C_ADDRESS, data_buffer + data_offset, 1);

    data_offset++;

    reg_address = BME280_CALIB26_REG;
    data_length = BME280_CALIB32_REG - BME280_CALIB26_REG + 1;
    config->send(BME280_I2C_ADDRESS, &reg_address, 1);
    config->receive(BME280_I2C_ADDRESS, data_buffer + data_offset, data_length);

    config->dig_T1 = data_buffer[0] | (uint16_t)data_buffer[1] << 8;
    config->dig_T2 = data_buffer[2] | (int16_t)data_buffer[3] << 8;
    config->dig_T3 = data_buffer[4] | (int16_t)data_buffer[5] << 8;

    config->dig_P1 = data_buffer[6] | (uint16_t)data_buffer[7] << 8;
    config->dig_P2 = data_buffer[8] | (int16_t)data_buffer[9] << 8;
    config->dig_P3 = data_buffer[10] | (int16_t)data_buffer[11] << 8;
    config->dig_P4 = data_buffer[12] | (int16_t)data_buffer[13] << 8;
    config->dig_P5 = data_buffer[14] | (int16_t)data_buffer[15] << 8;
    config->dig_P6 = data_buffer[16] | (int16_t)data_buffer[17] << 8;
    config->dig_P7 = data_buffer[18] | (int16_t)data_buffer[19] << 8;
    config->dig_P8 = data_buffer[20] | (int16_t)data_buffer[21] << 8;
    config->dig_P9 = data_buffer[22] | (int16_t)data_buffer[23] << 8;

    config->dig_H1 = data_buffer[24];
    config->dig_H2 = data_buffer[25] | (int16_t)data_buffer[26] << 8;
    config->dig_H3 = data_buffer[27];
    config->dig_H4 = ((int16_t)data_buffer[28] << 4) | (data_buffer[29] & 0x0F);
    config->dig_H5 = (data_buffer[29] & 0xF0) | ((int16_t)data_buffer[30] << 4);
    config->dig_H6 = data_buffer[31];
}

static void write_configuration(const bme280_t* config)
{
    uint8_t data_buffer[2];
    
    // write to ctrl_hum register
    data_buffer[0] = BME280_CTRL_HUM_REG;
    data_buffer[1] = config->humidity_oversampling & 0x7;
    config->send(BME280_I2C_ADDRESS, data_buffer, 2);
    
    // write to ctrl_meas register
    data_buffer[0] = BME280_CTRL_MEAS_REG;
    data_buffer[1] = config->temperature_oversampling << 5 | config->pressure_oversampling << 2 | config->mode;
    config->send(BME280_I2C_ADDRESS, data_buffer, 2);

    // write to config register
    data_buffer[0] = BME280_CONFIG_REG;
    data_buffer[1] = config->standby << 5 | config->filter << 2;
    config->send(BME280_I2C_ADDRESS, data_buffer, 2);
}

static void burst_read(bme280_t* config, int32_t* raw_pressure, int32_t* raw_temperature, int16_t* raw_humidity)
{
    uint8_t data_buffer[BME280_DATA_LENGTH];
    uint8_t reg_address = BME280_PRESS_MSB_REG;

    config->send(BME280_I2C_ADDRESS, &reg_address, 1);
    config->receive(BME280_I2C_ADDRESS, data_buffer, BME280_DATA_LENGTH);

    *raw_pressure = (uint32_t)data_buffer[0] << 12 | (uint16_t)data_buffer[1] << 4 | data_buffer[2] >> 4;
    *raw_temperature = (uint32_t)data_buffer[3] << 12 | (uint16_t)data_buffer[4] << 4 | data_buffer[5] >> 4;
    *raw_humidity = (uint16_t)data_buffer[6] << 8 | data_buffer[7];
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
// This must be run first before compensate_pressure and compensate_humidity
static int32_t compensate_temperature(bme280_t* config, int32_t adc_T)
{
    int32_t var1 = (((adc_T >> 3) - ((int32_t)config->dig_T1 << 1)) * ((int32_t)config->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)config->dig_T1)) * ((adc_T >> 4) - ((int32_t)config->dig_T1))) >> 12) *
            ((int32_t)config->dig_T3)) >> 14;
    config->fine_temperature = var1 + var2;
    int32_t temperature = (config->fine_temperature * 5 + 128) >> 8;
    return temperature;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 inteteger bits and 8 fractional bits)
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa
static uint32_t compensate_presure(const bme280_t* config, int32_t adc_P)
{
    int64_t var1 = ((int64_t)config->fine_temperature) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)config->dig_P6;
    var2 += ((int64_t)config->dig_P5) << 17;
    var2 += ((int64_t)config->dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)config->dig_P3) >> 8) + ((var1 * (int64_t)config->dig_P2) << 12);
    var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)config->dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; //avoid exception caused by division by zero
    }
    int64_t pressure = 1048576 - adc_P;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)config->dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (((int64_t)config->dig_P8) * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)config->dig_P7) << 4);
    
    return (uint32_t)pressure;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits)
static uint32_t compensate_humidity(const bme280_t* config, int32_t adc_H)
{
    int32_t humidity;
    humidity = config->fine_temperature - ((int32_t)76800);
    humidity = ((((adc_H << 14) - (((int32_t)config->dig_H4) << 20) - (((int32_t)config->dig_H5) * humidity)) + ((int32_t)16384)) >> 15) *
                (((((((humidity * ((int32_t)config->dig_H6)) >> 10) * (((humidity * ((int32_t)config->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                ((int32_t)config->dig_H2) + 8192) >> 14);

    humidity = (humidity - (((((humidity >> 15) * (humidity >> 15)) >> 7) * ((int32_t)config->dig_H1)) >> 4));
    humidity = (humidity < 0 ? 0 : humidity);
    humidity = (humidity > 419430400 ? 419430400 : humidity);
    return (uint32_t)(humidity >> 12);
}
