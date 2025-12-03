// aht10.h
/**
 * @file
 * @brief Header file for the AHT10 sensor interface.
 *
 * This file defines the functions and constants used to interact with the AHT10 humidity and temperature sensor.
 * It includes function declarations for reading temperature, humidity, and dew point values, as well as necessary
 * constants and types for sensor communication.
 *
 * @author Juliano Oliveira
 * @date 2025-08-10
 */

#ifndef AHT10_H
#define AHT10_H

#include "I2C.hpp"
#include <stdbool.h>

// device has default bus address of 0x38
#define ADDR 0x38
#define WATER_VAPOR 17.62f
#define BAROMETRIC_PRESSURE 243.5f

typedef enum {
    eAHT10Address_default = 0x38,
    eAHT10Address_Low     = 0x38,
    eAHT10Address_High    = 0x39,
} HUM_SENSOR_T;

typedef unsigned char Sensor_CMD;

#ifdef __cplusplus
extern "C" {
#endif

extern Sensor_CMD eSensorCalibrateCmd[3];
extern Sensor_CMD eSensorNormalCmd[3];
extern Sensor_CMD eSensorMeasureCmd[3];
extern Sensor_CMD eSensorResetCmd ;
extern bool    GetRHumidityCmd;
extern bool    GetTempCmd;

unsigned long readSensor(bool GetDataCmd);
float GetTemperature(I2C *i2c);
float GetHumidity(I2C *i2c);
float GetDewPoint(I2C *i2c);

#ifdef __cplusplus
}
#endif

#endif
