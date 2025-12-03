#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include <stdio.h>
#include <string.h>

#include "lwip/apps/mqtt.h"

#include "log_vt100.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"

#include "rack_event_groups.h"
#include "humidity_mqtt_task.h"

extern environment_t environment;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
extern bool mqtt_connected;
extern EventGroupHandle_t xEventGroup;

static inline void publish_rack_humidity(float humidity);

void vHumidityMqttTask(void *pvParameters) {
    LOG_INFO("[Humidity MQTT Task] Iniciando...");
    (void) pvParameters;

    EventBits_t uxBits;
    for (;;) {
        uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xHumidityBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            portMAX_DELAY );    // Espera indefinidamente.
        LOG_INFO("[Humidity MQTT Task] Evento recebido: %d", uxBits);
        if ((uxBits & xHumidityBitsToWaitFor) == xHumidityBitsToWaitFor) {
            LOG_INFO("[Humidity MQTT Task] Umidade mudou para: %.2f", environment.humidity);
            publish_rack_humidity(environment.humidity);
        }
    }
}

void publish_rack_humidity(float humidity) {
    LOG_INFO("[Humidity MQTT Task] Publicando umidade do rack");
    if (!mqtt_connected) {
        LOG_WARN("[MQTT] Não conectado, não publicando umidade do rack");
        return;
    }

    char topic_rack_humidity[50];
    snprintf(topic_rack_humidity, sizeof(topic_rack_humidity), "%s/environment/humidity", mqtt_rack_topic);

    char message[16];
    snprintf(message, sizeof(message), "%.2f", humidity);

    LOG_INFO("[Humidity MQTT Task] Publicando: tópico='%s', mensagem='%s'", topic_rack_humidity, message);

    err_t err = mqtt_publish(mqtt_client, topic_rack_humidity, message, strlen(message), 0, 0, NULL, NULL);

    if (err == ERR_OK) {
        LOG_INFO("[Humidity MQTT Task] Publicação de umidade enviada com sucesso");
    } else {
        LOG_WARN("[Humidity MQTT Task] Erro ao publicar umidade: %d", err);
    }
}
