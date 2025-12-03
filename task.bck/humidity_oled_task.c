#include "FreeRTOS.h"
#include "oled.h"
#include "task.h"
#include "event_groups.h"

#include <stdio.h>

#include "oled.h"
#include "oled_freeRTOS.h"

#include "log_vt100.h"

#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "rack_inteligente_parametros.h"

extern environment_t environment;
extern char mqtt_rack_topic[50];
extern bool mqtt_connected;
extern EventGroupHandle_t xEventGroup;

void vHumidityOledTask(void *pvParameters) {
    LOG_INFO("[Humidity OLED Task] Iniciando...");
    (void) pvParameters;

    EventBits_t uxBits;
    for (;;) {
        uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xHumidityBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            portMAX_DELAY );    // Espera indefinidamente.
        LOG_DEBUG("[Humidity OLED Task] Evento recebido: %d", uxBits);
        if ((uxBits & xHumidityBitsToWaitFor) == xHumidityBitsToWaitFor) {
            LOG_INFO("[Humidity OLED Task] Umidade mudou para: %.2f", environment.humidity);
            takeOled();
            char umidade[50];
            snprintf(umidade, sizeof(umidade), "Umidade: %.2f", environment.humidity);
            oled_set_text_line(2, umidade, OLED_ALIGN_CENTER);
            oled_render_text();
            releaseOled();
        }
    }
}

