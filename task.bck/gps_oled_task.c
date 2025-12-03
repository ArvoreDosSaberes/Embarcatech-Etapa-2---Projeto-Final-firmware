#include "FreeRTOS.h"
#include "log_vt100.h"
#include "oled.h"
#include "oled_freeRTOS.h"
#include "task.h"

#include "rack_event_groups.h"
#include "rack_inteligente.h"

extern EventGroupHandle_t xEventGroup;
extern environment_t environment;
extern char mqtt_rack_topic[50];

void vGpsOledTask(void *pvParameters) {
    LOG_INFO("[Gps OLED Task] Iniciando...");
    (void) pvParameters;

    EventBits_t xEventBits;
    for (;;) {
        xEventBits = xEventGroupWaitBits(xEventGroup,
             xGpsBitsToWaitFor,
             pdTRUE, 
             pdTRUE,
             portMAX_DELAY);

        LOG_INFO("[Gps OLED Task] Evento recebido: %02x", xEventBits);
        if(xEventBits & xGpsBitsToWaitFor){
            LOG_INFO("[Gps OLED Task] Gps detectado!");
            takeOled();
            char latitude[20];  
            char longitude[20];
            snprintf(latitude, sizeof(latitude), "latitude: %f", environment.gps_position.latitude);
            snprintf(longitude, sizeof(longitude), "longitude: %f", environment.gps_position.longitude);
            oled_set_text_line(2, latitude, OLED_ALIGN_CENTER);
            oled_set_text_line(3, longitude, OLED_ALIGN_CENTER);
            oled_render_text();
            releaseOled();
        }else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}