#include "aht10.h"
#include "log_vt100.h"
#include "math.h"
#include "I2C.hpp"
#include "FreeRTOS.h"
#include "task.h"

Sensor_CMD eSensorCalibrateCmd[3] = {0xE1, 0x08, 0x00};
Sensor_CMD eSensorNormalCmd[3]    = {0xA8, 0x00, 0x00};
Sensor_CMD eSensorMeasureCmd[3]   = {0xAC, 0x33, 0x00};
Sensor_CMD eSensorResetCmd        = 0xBA;
bool    GetRHumidityCmd        = true;
bool    GetTempCmd             = false;

unsigned long readSensor(I2C *i2c, bool GetDataCmd)
{
    unsigned long result;
    uint8_t temp[6];

    i2c->beginTransmission(ADDR, true);
    i2c->write(eSensorMeasureCmd, sizeof(eSensorMeasureCmd));
    #ifdef I2C_USE_FREERTOS
        vTaskDelay(pdMS_TO_TICKS(50));
    #else
        sleep_ms(50);
    #endif

    result = i2c->requestFrom(ADDR, sizeof(temp), false);
    if (result != sizeof(temp)) {
        LOG_INFO("AHT10: requestFrom returned %lu, expected %zu\n", result, sizeof(temp));

        result = i2c->endTransmission();
        if (result != 0) {
            LOG_INFO("AHT10: endTransmission failed after requestFrom with code %lu\n", result);
        }
        return 0;
    }

    result = i2c->endTransmission();
    if (result != 0) {
        LOG_INFO("AHT10: endTransmission failed with code %lu\n", result);
        return 0;
    }


    i2c->read((uint8_t*)temp, sizeof(temp));

    if(GetDataCmd)
    {
        result = ((((unsigned long)temp[1] << 16)) | (((unsigned long)temp[2]) << 8) | ((unsigned long)temp[3])) >> 4;
    }
    else
    {
        result = ((((unsigned long)temp[3]) & 0x0F) << 16) | (((unsigned long)temp[4]) << 8) | ((unsigned long)temp[5]);
    }

    return result;
}

float GetTemperature(I2C *i2c)
{
    #ifdef I2C_USE_FREERTOS
    taskENTER_CRITICAL();
    #endif
    float value = readSensor(i2c, GetTempCmd);
    float temperature = ((200 * value) / 1048576) - 50;
    #ifdef I2C_USE_FREERTOS
    taskEXIT_CRITICAL();
    #endif
    return temperature;
}

float GetHumidity(I2C *i2c)
{
    #ifdef I2C_USE_FREERTOS
    taskENTER_CRITICAL();
    #endif
    float value = readSensor(i2c, GetRHumidityCmd);
    if (value == 0) {
        #ifdef I2C_USE_FREERTOS
        taskEXIT_CRITICAL();
        #endif
        return 0;
    }
    float humidity = value * 100 / 1048576;
    #ifdef I2C_USE_FREERTOS
    taskEXIT_CRITICAL();
    #endif
    return humidity;
}

float GetDewPoint(I2C *i2c)
{
  float humidity = GetHumidity(i2c);
  float temperature = GetTemperature(i2c);

    #ifdef I2C_USE_FREERTOS
    taskENTER_CRITICAL();
    #endif
  float gamma = log(humidity / 100) + WATER_VAPOR * temperature / (BAROMETRIC_PRESSURE + temperature);
  float dewPoint = BAROMETRIC_PRESSURE * gamma / (WATER_VAPOR - gamma);
  
    #ifdef I2C_USE_FREERTOS
    taskEXIT_CRITICAL();
    #endif
  return dewPoint;
}
