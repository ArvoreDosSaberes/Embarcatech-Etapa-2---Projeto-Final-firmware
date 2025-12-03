
#include "FreeRTOS.h"
#include "log_vt100.h"
#include "oled.h"
#include "oled_freeRTOS.h"
#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "task.h"
#include "projdefs.h"
#include <stdio.h>

extern environment_t environment;
extern char mqtt_rack_topic[50];
extern EventGroupHandle_t xEventGroup;
 

void vTiltOledTask(void *pvParameters) {
    LOG_INFO("[Tilt OLED Task] Iniciando...");

    EventBits_t xEventBits;
    
    for(;;) {
        xEventBits = xEventGroupWaitBits( xEventGroup,
             xTiltBitsToWaitFor,
             pdTRUE, 
             pdTRUE,
             portMAX_DELAY);

        LOG_INFO("[Tilt OLED Task] Evento recebido: %02x", xEventBits);
        if(xEventBits & xTiltBitsToWaitFor){
            LOG_INFO("[Tilt OLED Task] Tilt detectado!");
            takeOled();
            oled_set_text_line(2, "Tilt detectado!", OLED_ALIGN_CENTER);
            oled_render_text();
            releaseOled();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

