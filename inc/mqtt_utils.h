/**
 * @file mqtt_utils.h
 * @brief Utilitários para publicação MQTT com mutex e backpressure.
 *
 * Este módulo fornece funções para publicação MQTT thread-safe,
 * com serialização via mutex e backpressure para evitar esgotamento
 * de pbufs do LwIP.
 *
 * @author EmbarcaTech TIC-27
 * @version 1.0.0
 * @date 2025
 */

#ifndef MQTT_UTILS_H
#define MQTT_UTILS_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "lwip/apps/mqtt.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constantes de Configuração
 * ============================================================================ */

/**
 * @brief Número máximo de tentativas de publicação antes de desistir.
 */
#define MQTT_PUBLISH_MAX_RETRIES        3

/**
 * @brief Delay base entre tentativas de publicação (ms).
 * 
 * O delay aumenta exponencialmente: base * 2^tentativa
 */
#define MQTT_PUBLISH_RETRY_DELAY_MS     100

/**
 * @brief Timeout para aguardar o mutex MQTT (ms).
 */
#define MQTT_MUTEX_TIMEOUT_MS           1000

/**
 * @brief Delay após publicação bem-sucedida para evitar sobrecarga (ms).
 * 
 * Garante que a stack TCP/IP tenha tempo para processar o pacote.
 */
#define MQTT_PUBLISH_POST_DELAY_MS      50

/* ============================================================================
 * Funções Públicas
 * ============================================================================ */

/**
 * @brief Inicializa o módulo de utilitários MQTT.
 * 
 * Cria o mutex para serialização de publicações.
 * Deve ser chamada antes de qualquer publicação MQTT.
 *
 * @return pdPASS se inicializado com sucesso, pdFAIL caso contrário.
 */
BaseType_t mqttUtilsInit(void);

/**
 * @brief Publica mensagem MQTT de forma thread-safe com backpressure.
 * 
 * Esta função:
 * 1. Adquire o mutex MQTT (serializa publicações)
 * 2. Tenta publicar com retries em caso de falha
 * 3. Aplica delay entre tentativas (backpressure)
 * 4. Libera o mutex após conclusão
 *
 * @param client    Ponteiro para o cliente MQTT.
 * @param topic     Tópico onde publicar.
 * @param payload   Dados a serem publicados.
 * @param payloadLen Tamanho dos dados em bytes.
 * @param qos       Qualidade de serviço (0, 1 ou 2).
 * @param retain    Se true, mensagem será retida pelo broker.
 *
 * @return ERR_OK se publicado com sucesso, código de erro caso contrário.
 */
err_t mqttPublishSafe(mqtt_client_t *client, const char *topic,
                      const void *payload, uint16_t payloadLen,
                      uint8_t qos, uint8_t retain);

/**
 * @brief Verifica se o mutex MQTT está disponível.
 * 
 * Útil para diagnóstico e debug.
 *
 * @return true se o mutex foi inicializado, false caso contrário.
 */
bool mqttUtilsIsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_UTILS_H
