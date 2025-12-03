#include "FreeRTOS.h"
#include "event_groups.h"
#include "lib/OLED_SSD1306/oled.h"
#include "oled_freeRTOS.h"
#include "task.h"

#include "rack_event_groups.h"
#include "rack_inteligente.h"

#include "oled.h"
#include "oled_freeRTOS.h"

#include "log_vt100.h"

extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

void vTemperatureOledTask(void *pvParameters){

    
    uint32_t uxBits;
    for(;;){
          uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xTemperatureBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            portMAX_DELAY );    // Espera indefinidamente.
        LOG_DEBUG("[Temperature OLED Task] Evento recebido: %d", uxBits);
        if ((uxBits & xTemperatureBitsToWaitFor) == xTemperatureBitsToWaitFor) {
            
            char oledText[50];
            snprintf(oledText, sizeof(oledText), "Temperatura: %.2f\n", environment.temperature);
            takeOled();
            oled_set_text_line(3, oledText, OLED_ALIGN_LEFT);
            oled_render();
            releaseOled();
        }
    }
}