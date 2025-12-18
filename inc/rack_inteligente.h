/**
 * @file rack_inteligente.h
 * @brief Definições globais e estruturas de dados do Rack Inteligente.
 *
 * Este header define as estruturas de dados compartilhadas entre os módulos
 * do firmware, incluindo dados ambientais, GPS e estado dos atuadores.
 *
 * @author EmbarcaTech TIC-27
 * @version 1.0.0
 * @date 2025
 */

#ifndef RACK_INTELIGENTE_H
#define RACK_INTELIGENTE_H

#include <stdbool.h>
#include "lwip/apps/mqtt.h"

/**
 * @defgroup DoorMacros Macros de Estado da Porta
 * @brief Constantes booleanas para estado da porta.
 * @{
 */

/** @brief Estado da porta: aberta. */
#define RACK_DOOR_OPEN true

/** @brief Estado da porta: fechada. */
#define RACK_DOOR_CLOSED false

/** @} */ // fim DoorMacros

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ExternVars Variáveis Externas
 * @brief Variáveis globais acessíveis por outros módulos.
 * @{
 */

/** @brief Flag de conexão MQTT. @see rack_inteligente.cpp */
extern bool mqtt_connected;

/** @brief Cliente MQTT lwIP. @see rack_inteligente.cpp */
extern mqtt_client_t *mqtt_client;

/** @brief Tópico base MQTT do rack. Formato: "{base}/{rack_id}". */
extern char mqtt_rack_topic[50];

/** @brief Nome/identificador do rack. */
extern char rack_name[50];

/** @brief Endereço IP obtido via Wi-Fi (string IPv4). */
extern char wifiIpAddress[16];

/** @} */ // fim ExternVars

/**
 * @brief Estrutura de posição GPS.
 *
 * Armazena coordenadas geográficas e metadados do GPS.
 * Valores são atualizados pela task gps_task.
 *
 * @see gps_task.h
 */
typedef struct {
    float latitude;     /**< Latitude em graus decimais (-90 a +90). */
    float longitude;    /**< Longitude em graus decimais (-180 a +180). */
    float altitude;     /**< Altitude em metros acima do nível do mar. */
    uint32_t time;      /**< Timestamp UTC em segundos desde epoch. */
    float speed;        /**< Velocidade em km/h (geralmente 0 para racks fixos). */
} gps_position_t;

/**
 * @brief Estrutura de dados ambientais do rack.
 *
 * Contém todas as leituras de sensores e estado dos atuadores.
 * Esta estrutura é atualizada pelas tasks de sensores e consultada
 * pelas tasks de publicação MQTT.
 *
 * @note Acesso deve ser protegido por mutex em operações multi-task.
 *
 * @see temperature_humidity_task.hpp
 * @see door_state_mqtt_task.h
 * @see tilt_task.hpp
 * @see gps_task.h
 */
typedef struct {
    float temperature;          /**< Temperatura em graus Celsius. */
    float humidity;             /**< Umidade relativa em porcentagem (0-100%). */
    bool door;                  /**< Estado da porta: true=aberta, false=fechada. */
    bool tilt;                  /**< Estado de inclinação: true=inclinado, false=normal. */
    gps_position_t gps_position; /**< Posição GPS do rack. */
} environment_t;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // RACK_INTELIGENTE_H
