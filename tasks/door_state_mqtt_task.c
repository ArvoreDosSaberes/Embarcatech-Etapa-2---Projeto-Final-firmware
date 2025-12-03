#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "oled.h"
#include "oled_freeRTOS.h"

#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"

#include "log_vt100.h"


#include "rack_event_groups.h"
#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "door_state_mqtt_task.h"

// Variáveis Locais
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
static inline void publish_door_state(bool pressed);

void vDoorStateMqttTask(void *pvParameters) {
    LOG_INFO("[Door State MQTT Task] Iniciando...");
    (void) pvParameters;
    uint32_t uxBits;

    for (;;) {
        uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xDoorStateBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            portMAX_DELAY );    // Espera indefinidamente.
        LOG_DEBUG("[Door State MQTT Task] Evento recebido: %d", uxBits);
        if ((uxBits & xDoorStateBitsToWaitFor) == xDoorStateBitsToWaitFor) {
                    
            publish_door_state(environment.door);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void publish_door_state(bool pressed) {
    if (!mqtt_connected) {
        LOG_WARN("[MQTT] Não conectado, não publicando estado da porta");
        return;
    }
    char topic_door_state[50];
    snprintf(topic_door_state, sizeof(topic_door_state), "%s/status", mqtt_rack_topic);

    const char *message = pressed ? "1" : "0";

    LOG_INFO("[MQTT] Publicando: tópico='%s', mensagem='%s'", topic_door_state, message);

    err_t err = mqtt_publish(mqtt_client, topic_door_state, message, strlen(message), 0, 0, NULL, NULL);

    if (err == ERR_OK) {
        LOG_INFO("[MQTT] Publicação enviada com sucesso");
    } else {
        LOG_WARN("[MQTT] Erro ao publicar: %d", err);
    }
}

