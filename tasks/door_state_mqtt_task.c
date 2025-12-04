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
#include "buzzer_pwm_task.h"
#include "command_mqtt_task.h"

// Variáveis Locais
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];

static inline void publish_door_state(bool gpioState);
static void check_door_open_timeout(void);

/**
 * @brief Timestamp de quando a porta foi aberta (0 = porta fechada).
 */
static TickType_t doorOpenedAt = 0;

/**
 * @brief Flag para indicar se o alerta já foi ativado.
 */
static bool doorAlertTriggered = false;

void vDoorStateMqttTask(void *pvParameters) {
    LOG_INFO("[Door State MQTT Task] Iniciando...");
    (void) pvParameters;
    uint32_t uxBits;

    for (;;) {
        /* Espera evento ou timeout para verificar porta aberta */
        uxBits = xEventGroupWaitBits(
            xEventGroup,
            xDoorStateBitsToWaitFor,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(RACK_DOOR_CHECK_INTERVAL_MS));
        
        if ((uxBits & xDoorStateBitsToWaitFor) == xDoorStateBitsToWaitFor) {
            LOG_DEBUG("[Door State MQTT Task] Evento recebido: %d", uxBits);
            publish_door_state(environment.door);
        }
        
        /* Verifica timeout de porta aberta */
        check_door_open_timeout();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Verifica se a porta está aberta há mais de 20 minutos.
 * 
 * Se o tempo limite for atingido, ativa o alerta de porta aberta.
 */
static void check_door_open_timeout(void) {
    /* Lógica invertida: pull-up faz GPIO=0 quando pressionado (porta aberta) */
    bool doorOpen = environment.door;
    
    if (doorOpen) {
        /* Porta aberta - verifica timeout */
        if (doorOpenedAt == 0) {
            /* Marca o momento em que a porta foi aberta */
            doorOpenedAt = xTaskGetTickCount();
            doorAlertTriggered = false;
            LOG_INFO("[Door] Porta aberta - iniciando contagem de 20 minutos");
        } else if (!doorAlertTriggered) {
            /* Calcula tempo decorrido */
            TickType_t elapsed = xTaskGetTickCount() - doorOpenedAt;
            TickType_t timeoutTicks = pdMS_TO_TICKS(RACK_DOOR_OPEN_ALERT_TIMEOUT_MS);
            
            if (elapsed >= timeoutTicks) {
                /* Timeout atingido - ativa alerta */
                LOG_WARN("[Door] Porta aberta há 20 minutos - ativando alerta!");
                buzzerPwmSetState(BUZZER_STATE_DOOR_OPEN);
                doorAlertTriggered = true;
                
                /* Publica ACK do buzzer ativado (se MQTT conectado) */
                if (mqtt_connected) {
                    publishCommandAck(COMMAND_TYPE_BUZZER, BUZZER_DOOR_OPEN);
                }
            }
        }
    } else {
        /* Porta fechada - reseta contagem */
        if (doorOpenedAt != 0) {
            LOG_INFO("[Door] Porta fechada - resetando contagem");
        }
        doorOpenedAt = 0;
        doorAlertTriggered = false;
    }
}

/**
 * @brief Publica o estado da porta via MQTT.
 * 
 * O botão A (pino 5) usa lógica invertida (pull-up interno):
 * - GPIO = 0 (pressionado) → porta ABERTA → publica "1"
 * - GPIO = 1 (solto) → porta FECHADA → publica "0"
 * 
 * Quando a porta é fechada, se o buzzer estiver alertando "porta aberta",
 * ele é automaticamente desligado.
 * 
 * @param gpioState Estado do GPIO (true = HIGH/solto, false = LOW/pressionado)
 */
void publish_door_state(bool gpioState) {
    /* Lógica invertida: pull-up faz GPIO=0 quando pressionado (porta aberta) */
    bool doorOpen = !gpioState;
    
    /* Se a porta foi fechada e o buzzer está alertando porta aberta, desliga */
    if (!doorOpen && getBuzzerState() == BUZZER_DOOR_OPEN) {
        LOG_INFO("[Door] Porta fechada - desligando alerta de porta aberta");
        buzzerPwmSetState(BUZZER_STATE_OFF);
        /* Publica ACK do buzzer desligado (se MQTT conectado) */
        if (mqtt_connected) {
            publishCommandAck(COMMAND_TYPE_BUZZER, BUZZER_OFF);
        }
    }
    
    if (!mqtt_connected) {
        LOG_WARN("[MQTT] Não conectado, não publicando estado da porta");
        return;
    }
    char topic_door_state[80];
    snprintf(topic_door_state, sizeof(topic_door_state), "%s/environment/door", mqtt_rack_topic);

    const char *message = doorOpen ? "0" : "1";

    LOG_INFO("[MQTT] Publicando: tópico='%s', mensagem='%s'", topic_door_state, message);

    err_t err = mqtt_publish(mqtt_client, topic_door_state, message, strlen(message), 0, 0, NULL, NULL);

    if (err == ERR_OK) {
        LOG_INFO("[MQTT] Publicação enviada com sucesso");
    } else {
        LOG_WARN("[MQTT] Erro ao publicar: %d", err);
    }
}

