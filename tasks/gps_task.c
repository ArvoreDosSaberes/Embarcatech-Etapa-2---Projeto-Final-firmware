#include "FreeRTOS.h"
#include "rack_event_groups.h"
#include "task.h"

#include "pico/stdlib.h"

#include <hardware/timer.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <lwip/apps/mqtt.h>

#include "log_vt100.h"
#include "gps_task.h"
#include "rack_event_groups.h"
#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"

extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

// Variáveis Locais
static float latitude = -3.924263;
static float longitude = -38.453483;

static inline void publish_rack_gps_position();

void vGpsTask(void *pvParameters) {
    LOG_INFO("[GPS Task] Iniciando...");
    (void) pvParameters;

    for (;;) {

        environment.gps_position.latitude = latitude;
        environment.gps_position.longitude = longitude;
        environment.gps_position.altitude = 0;
        environment.gps_position.time = time_us_64();
        environment.gps_position.speed = 0;

        LOG_INFO("[GPS Task] Latitude: %f", latitude);
        LOG_INFO("[GPS Task] Longitude: %f", longitude);
        LOG_INFO("[GPS Task] Altitude: %f", environment.gps_position.altitude);
        LOG_INFO("[GPS Task] Time: %d", environment.gps_position.time);
        LOG_INFO("[GPS Task] Speed: %f", environment.gps_position.speed);
            
        xEventGroupSetBits(xEventGroup, xGpsBitsToWaitFor);
        
        vTaskDelay(pdMS_TO_TICKS(RACK_GPS_TASK_DELAY));
    }
}

