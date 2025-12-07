#include "FreeRTOS.h"
#include "log_vt100.h"
#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "task.h"
#include "projdefs.h"
#include <stdio.h>
#include "lwip/apps/mqtt.h"
#include "mqtt_utils.h"

extern environment_t environment;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
extern bool mqtt_connected;
extern EventGroupHandle_t xEventGroup;

static void publish_tilt_mqtt(bool tilt);

void vTiltMqttTask(void *pvParameters) {
    LOG_INFO("[Tilt MQTT Task] Iniciando...");

    EventBits_t xEventBits;
    
    for(;;) {
        xEventBits = xEventGroupWaitBits( xEventGroup,
             xTiltBitsToWaitFor,
             pdTRUE, 
             pdTRUE,
             portMAX_DELAY);

        LOG_INFO("[Tilt MQTT Task] Evento recebido: %02x", xEventBits);
        if(xEventBits & xTiltBitsToWaitFor){
            LOG_INFO("[Tilt MQTT Task] Tilt detectado!");
            publish_tilt_mqtt(environment.tilt);
        }
    }
}

void publish_tilt_mqtt(bool tilt){
    if (!mqtt_connected) {
        LOG_INFO("[MQTT] Não conectado, não publicando tilt");
        return;
    }
    char topic_tilt[50];
    snprintf(topic_tilt, sizeof(topic_tilt), "%s/tilt", mqtt_rack_topic);

    const char *message = tilt ? "1" : "0";

    err_t err = mqttPublishSafe(mqtt_client, topic_tilt, message, 1, 0, 0);
    if (err == ERR_OK) {
        LOG_INFO("[Tilt MQTT] Publicação enviada com sucesso");
    } else {
        LOG_WARN("[Tilt MQTT] Erro ao publicar: %d", err);
    }
}