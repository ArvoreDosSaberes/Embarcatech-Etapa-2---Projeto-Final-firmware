#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include <math.h>

#include "I2C.hpp"

#include "log_vt100.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "aht10.h"
#include "temperature_humidity_task.hpp"
#include "rack_event_groups.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

#define TEMPERATURE_UNITS 'C'

extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

static I2C *pAht10I2c = nullptr;

extern "C" {

static inline float read_rack_temperature(const char unit);
static inline float read_rack_humidity(void);

void vTemperatureHumidityTask(void *pvParameters) {
    LOG_INFO("[Temperature & humidity] Iniciando...");

    pAht10I2c = static_cast<I2C*>(pvParameters);

    float last_rack_temperature = -1.0f;
    float last_rack_humidity = -1.0f;

    for (;;) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t loopStartUs = wcetProbeNowUs();
#endif
        LOG_INFO("[Temperature & humidity] Lendo temperatura e umidade...");

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t tempReadStartUs = wcetProbeNowUs();
#endif
        float rack_temperature = read_rack_temperature(TEMPERATURE_UNITS);
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("temp_hum.read_temperature", tempReadStartUs, wcetProbeNowUs());
#endif
        float rack_temperature_rounded = roundf(rack_temperature * 10.0f) / 10.0f;

        LOG_INFO("[Temperature & humidity] Temperature reading: %.1f %c", rack_temperature_rounded, TEMPERATURE_UNITS);
        if (rack_temperature_rounded != last_rack_temperature) {
            LOG_INFO("[Temperature & humidity] Rack temperature: %.1f %c", rack_temperature_rounded, TEMPERATURE_UNITS);
            environment.temperature = rack_temperature_rounded;
            xEventGroupSetBits(xEventGroup, xTemperatureBitsToWaitFor);
            last_rack_temperature = rack_temperature_rounded;
        }

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t humReadStartUs = wcetProbeNowUs();
#endif
        float rack_humidity = read_rack_humidity();
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("temp_hum.read_humidity", humReadStartUs, wcetProbeNowUs());
#endif
        float rack_humidity_rounded = roundf(rack_humidity * 10.0f) / 10.0f;

        LOG_INFO("[Temperature & humidity] Humidity reading: %.1f%%", rack_humidity_rounded);
        if (rack_humidity_rounded != last_rack_humidity) {
            LOG_INFO("[Temperature & humidity] Rack humidity: %.1f%%", rack_humidity_rounded);
            environment.humidity = rack_humidity_rounded;
            xEventGroupSetBits(xEventGroup, xHumidityBitsToWaitFor);
            last_rack_humidity = rack_humidity_rounded;
        }

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("temp_hum.loop_active", loopStartUs, wcetProbeNowUs());
#endif
        vTaskDelay(pdMS_TO_TICKS(RACK_TMP_HUM_TASK_DELAY));
    }
}

float read_rack_temperature(const char unit) {
    float tempC = GetTemperature(pAht10I2c);

    if (unit == 'C') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

float read_rack_humidity(void) {
    float humidity = GetHumidity(pAht10I2c);

    if (humidity < 0.0f || humidity > 100.0f) {
        return -1.0f;
    }

    return humidity;
}
}