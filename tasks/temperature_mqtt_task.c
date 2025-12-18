#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"

#include "log_vt100.h"
#include "mqtt_utils.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "temperature_mqtt_task.h"

#include "rack_event_groups.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

extern EventGroupHandle_t xEventGroup;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
extern bool mqtt_connected;
extern environment_t environment;

static inline void publish_rack_temperature(float temperature);

void vTemperatureMqttTask(void *pvParameters) {
    LOG_INFO("[TEMPERATURE MQTT TASK] Iniciando...");
    (void) pvParameters;

    EventBits_t uxBits;
    for (;;) {
        uxBits = xEventGroupWaitBits(
            xEventGroup,        // O grupo de eventos.
            xTemperatureBitsToWaitFor,     // O(s) bit(s) a esperar.
            pdTRUE,             // Limpar o(s) bit(s) ao sair.
            pdFALSE,            // Não precisa de todos os bits (só tem 1).
            pdMS_TO_TICKS(60000) ); // Espera 60 segundos, assim mantem uma comunicação regular com o broker

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t activeStartUs = wcetProbeNowUs();
#endif

        LOG_INFO("[TEMPERATURE MQTT TASK] Temperatura mudou para: %.2f", environment.temperature);

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t publishStartUs = wcetProbeNowUs();
#endif
        publish_rack_temperature(environment.temperature);

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("temperature_mqtt.publish", publishStartUs, wcetProbeNowUs());
        wcetProbeRecord("temperature_mqtt.loop_active", activeStartUs, wcetProbeNowUs());
#endif
    }
}

void publish_rack_temperature(float temperature) {
    if (!mqtt_connected) {
        LOG_WARN("[MQTT] Não conectado, não publicando temperatura do rack");
        return;
    }
    char topic_rack_temperature[50];
    snprintf(topic_rack_temperature, sizeof(topic_rack_temperature), "%s/environment/temperature", mqtt_rack_topic);

    char message[16];
    snprintf(message, sizeof(message), "%.2f", temperature);

    LOG_INFO("[MQTT] Publicando: tópico='%s', mensagem='%s'", topic_rack_temperature, message);

    err_t err = mqttPublishSafe(mqtt_client, topic_rack_temperature, message, strlen(message), 0, 0);

    if (err == ERR_OK) {
        LOG_INFO("[Temperature MQTT] Publicação enviada com sucesso");
    } else {
        LOG_WARN("[Temperature MQTT] Erro ao publicar: %d", err);
    }
}