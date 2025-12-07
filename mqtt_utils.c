/**
 * @file mqtt_utils.c
 * @brief Implementação de utilitários MQTT com mutex e backpressure.
 *
 * @author EmbarcaTech TIC-27
 * @version 1.0.0
 * @date 2025
 */

#include "mqtt_utils.h"
#include "log_vt100.h"
#include "task.h"
#include <string.h>

/* ============================================================================
 * Variáveis Privadas
 * ============================================================================ */

/**
 * @brief Mutex para serialização de publicações MQTT.
 */
static SemaphoreHandle_t mqttMutex = NULL;

/**
 * @brief Flag indicando se o módulo foi inicializado.
 */
static bool initialized = false;

/* ============================================================================
 * Implementação das Funções Públicas
 * ============================================================================ */

BaseType_t mqttUtilsInit(void) {
    if (initialized) {
        LOG_WARN("[MQTT Utils] Já inicializado");
        return pdPASS;
    }

    mqttMutex = xSemaphoreCreateMutex();
    if (mqttMutex == NULL) {
        LOG_WARN("[MQTT Utils] Falha ao criar mutex MQTT");
        return pdFAIL;
    }

    initialized = true;
    LOG_INFO("[MQTT Utils] Inicializado com sucesso");
    return pdPASS;
}

err_t mqttPublishSafe(mqtt_client_t *client, const char *topic,
                      const void *payload, uint16_t payloadLen,
                      uint8_t qos, uint8_t retain) {
    if (!initialized) {
        LOG_WARN("[MQTT Utils] Módulo não inicializado!");
        return ERR_ARG;
    }

    if (client == NULL || topic == NULL) {
        LOG_WARN("[MQTT Utils] Parâmetros inválidos");
        return ERR_ARG;
    }

    /* Tenta adquirir o mutex com timeout */
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(MQTT_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        LOG_WARN("[MQTT Utils] Timeout ao aguardar mutex MQTT");
        return ERR_TIMEOUT;
    }

    err_t result = ERR_OK;
    uint8_t retryCount = 0;
    TickType_t delayMs = MQTT_PUBLISH_RETRY_DELAY_MS;

    /* Loop de tentativas com backpressure exponencial */
    while (retryCount < MQTT_PUBLISH_MAX_RETRIES) {
        result = mqtt_publish(client, topic, payload, payloadLen, qos, retain, NULL, NULL);

        if (result == ERR_OK) {
            /* Publicação bem-sucedida - aplica delay pós-publicação */
            vTaskDelay(pdMS_TO_TICKS(MQTT_PUBLISH_POST_DELAY_MS));
            break;
        }

        /* Falha na publicação */
        retryCount++;
        
        if (retryCount < MQTT_PUBLISH_MAX_RETRIES) {
            LOG_WARN("[MQTT Utils] Falha ao publicar (err=%d), tentativa %d/%d. "
                     "Aguardando %lu ms...",
                     result, retryCount, MQTT_PUBLISH_MAX_RETRIES, 
                     (unsigned long)delayMs);
            
            /* Backpressure: aguarda antes de tentar novamente */
            vTaskDelay(pdMS_TO_TICKS(delayMs));
            
            /* Aumenta delay exponencialmente (máximo 1 segundo) */
            delayMs = (delayMs * 2 > 1000) ? 1000 : delayMs * 2;
        } else {
            LOG_WARN("[MQTT Utils] Falha definitiva ao publicar em '%s' "
                      "após %d tentativas (err=%d)",
                      topic, MQTT_PUBLISH_MAX_RETRIES, result);
        }
    }

    /* Libera o mutex */
    xSemaphoreGive(mqttMutex);

    return result;
}

bool mqttUtilsIsInitialized(void) {
    return initialized;
}
