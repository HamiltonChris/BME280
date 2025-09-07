/*
 * @file bme280.h
 *
 * @author Chris Hamilton
 * 
 * @brief driver for bme280 hummidity, pressure, and temperature unit
 */

#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#define BME280_I2C_ADDRESS  0x76
#define BME280_CHIP_ID      0x60

#define BME280_CALIB00_REG      0x88
#define BME280_CALIB23_REG		0x9F
#define BME280_CALIB25_REG      0xA1
#define BME280_ID_REG           0xD0
#define BME280_RESET_REG        0xE0
#define BME280_CALIB26_REG      0xE1
#define BME280_CALIB32_REG      0xE7
#define BME280_CALIB41_REG      0xF0
#define BME280_CTRL_HUM_REG     0xF2
#define BME280_STATUS_REG       0xF3
#define BME280_CTRL_MEAS_REG    0xF4
#define BME280_CONFIG_REG       0xF5
#define BME280_PRESS_MSB_REG    0xF7
#define BME280_PRESS_LSB_REG    0xF8
#define BME280_PRESS_XLSB_REG   0xF9
#define BME280_TEMP_MSB_REG     0xFA
#define BME280_TEMP_LSB_REG     0xFB
#define BME280_HUM_XLSB_REG     0xFC
#define BME280_HUM_MSB_REG      0xFD
#define BME280_HUM_LSB_REG      0xFE

#define BME280_RESET_VALUE      0xB6

#define BME280_DATA_LENGTH          8
#define BME280_CALIBRATION_LENGTH   32

typedef enum bme280_oversampling_e {
    SKIPPED = 0,
    OVERSAMPLING_x1 = 1,
    OVERSAMPLING_x2 = 2,
    OVERSAMPLING_x4 = 3,
    OVERSAMPLING_x8 = 4,
    OVERSAMPLING_x16 = 5
} bme280_oversampling_t;

typedef enum bme280_mode_e {
    SLEEP = 0,
    FORCED = 1,
    NORMAL = 3
} bme280_mode_t;

typedef enum bme280_standby_e {
    SB_0_5MS = 0,
    SB_62_5MS = 1,
    SB_125MS = 2,
    SB_250MS = 3,
    SB_500MS = 4,
    SB_1000MS = 5,
    SB_10MS = 6,
    SB_20MS = 7,
} bme280_standby_t;

typedef enum bme280_filter_e {
    FILTER_OFF = 0,
    FILTER_COEFF_2 = 1,
    FILTER_COEFF_4 = 2,
    FILTER_COEFF_8 = 3,
    FILTER_COEFF_16 = 4
} bme280_filter_t;

typedef struct bme280_s {
    void (*send)(uint8_t, uint8_t*, uint8_t);
    void (*receive)(uint8_t, uint8_t*, uint8_t);

    bme280_oversampling_t   temperature_oversampling;
    bme280_oversampling_t   pressure_oversampling;
    bme280_oversampling_t   humidity_oversampling;
    bme280_mode_t           mode;
    bme280_standby_t        standby;
    bme280_filter_t         filter;

    // Compensation values
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    // fine_temperature carries fine tempereature as global value
    int32_t fine_temperature;
} bme280_t;

int8_t bme280_init(bme280_t* config);
int8_t bme280_read_all(bme280_t* config, uint32_t* pressure, uint32_t* humidity, int32_t* temperature);


#endif // BME280_H
