#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "oled.h"
#include "oled_freeRTOS.h"
#include "task.h"

#include <string.h>

#include "log_vt100.h"

#include "rack_event_groups.h"
#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"

// Variáveis Locais
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;
static inline void publish_door_state(bool door_state);

void vDoorStateOledTask(void *pvParameters) {
    LOG_INFO("[Door State OLED Task] Iniciando...");
    (void) pvParameters;
    uint32_t uxBits;

    for (;;) {
        uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xDoorStateBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            portMAX_DELAY );    // Espera indefinidamente.
        LOG_DEBUG("[Door State OLED Task] Evento recebido: %d", uxBits);
        if ((uxBits & xDoorStateBitsToWaitFor) == xDoorStateBitsToWaitFor) {
            publish_door_state(environment.door);            
        }
    }
}

void publish_door_state(bool door_state) {
    if (!mqtt_connected) {
        LOG_WARN("[MQTT] Não conectado, não publicando estado da porta");
        takeOled();
        oled_set_text_line(5, "Não conectado", OLED_ALIGN_CENTER);
        oled_render_text();
        releaseOled();
        return;
    }
    char topic_door_state[50];
    snprintf(topic_door_state, sizeof(topic_door_state), "%s/status", mqtt_rack_topic);

    const char *message = door_state ? "1" : "0";

    LOG_INFO("[MQTT] Publicando: tópico='%s', mensagem='%s'", topic_door_state, message);
    takeOled();
    oled_set_text_line(2, "Publicando...", OLED_ALIGN_CENTER);
    oled_render_text();
    releaseOled();

    err_t err = mqtt_publish(mqtt_client, topic_door_state, message, strlen(message), 0, 0, NULL, NULL);

    if (err == ERR_OK) {
        LOG_INFO("[MQTT] Publicação enviada com sucesso");
        takeOled();
        oled_set_text_line(2, "Publicado com sucesso", OLED_ALIGN_CENTER);
        oled_render_text();
        releaseOled();
    } else {
        LOG_WARN("[MQTT] Erro ao publicar: %d", err);
        takeOled();
        oled_set_text_line(2, "Erro ao publicar", OLED_ALIGN_CENTER);
        oled_render_text();
        releaseOled();
    }
}

