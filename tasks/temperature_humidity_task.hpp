#ifndef TEMPERATURE_TASK_H
#define TEMPERATURE_TASK_H

/* Choose 'C' for Celsius or 'F' for Fahrenheit. */
#define TEMPERATURE_UNITS 'C'

#ifdef __cplusplus
extern "C" {
#endif

void vTemperatureHumidityTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
