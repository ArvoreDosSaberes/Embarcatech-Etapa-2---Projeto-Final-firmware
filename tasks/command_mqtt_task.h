/**
 * @file command_mqtt_task.h
 * @brief Task para processamento de comandos MQTT recebidos do dashboard.
 *
 * Este módulo é responsável por:
 * - Receber comandos do dashboard via tópicos MQTT
 * - Executar os comandos (porta, ventilação, buzzer)
 * - Publicar confirmações (ACK) para o dashboard
 *
 * Fluxo de comunicação:
 * 1. Dashboard publica comando em: {base}/{rack_id}/command/{door|ventilation|buzzer}
 * 2. Firmware executa o comando
 * 3. Firmware publica confirmação em: {base}/{rack_id}/ack/{door|ventilation|buzzer}
 * 4. Dashboard recebe ACK e atualiza a UI
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#ifndef COMMAND_MQTT_TASK_H
#define COMMAND_MQTT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tipos de comando suportados.
 */
typedef enum {
    COMMAND_TYPE_NONE = 0,       /**< Nenhum comando */
    COMMAND_TYPE_DOOR,           /**< Comando de porta (abrir/fechar) */
    COMMAND_TYPE_VENTILATION,    /**< Comando de ventilação (ligar/desligar) */
    COMMAND_TYPE_BUZZER          /**< Comando de buzzer (0-3) */
} CommandType;

/**
 * @brief Estados do buzzer.
 */
typedef enum {
    BUZZER_OFF = 0,              /**< Buzzer desligado */
    BUZZER_DOOR_OPEN = 1,        /**< Alerta de porta aberta */
    BUZZER_BREAK_IN = 2,         /**< Alerta de arrombamento */
    BUZZER_OVERHEAT = 3          /**< Alerta de superaquecimento */
} BuzzerState;

/**
 * @brief Estrutura para armazenar um comando na fila.
 * 
 * Usada internamente pela task de comandos para processar
 * comandos de forma assíncrona fora do callback MQTT.
 */
typedef struct {
    CommandType type;            /**< Tipo do comando */
    int value;                   /**< Valor do comando */
} CommandQueueItem;

/**
 * @brief Inicializa as variáveis e filas do módulo de comandos.
 *
 * Deve ser chamada antes de iniciar a task de comandos.
 *
 * @return true se inicialização bem-sucedida, false caso contrário
 */
bool commandMqttInit(void);

/**
 * @brief Processa um comando de porta recebido via MQTT.
 *
 * @param value 1 para abrir, 0 para fechar
 */
void processCommandDoor(int value);

/**
 * @brief Processa um comando de ventilação recebido via MQTT.
 *
 * @param value 1 para ligar, 0 para desligar
 */
void processCommandVentilation(int value);

/**
 * @brief Processa um comando de buzzer recebido via MQTT.
 *
 * @param value Estado do buzzer (0-3)
 */
void processCommandBuzzer(int value);

/**
 * @brief Publica confirmação (ACK) de comando via MQTT.
 *
 * @param commandType Tipo do comando confirmado
 * @param value Valor confirmado
 * @return true se publicação bem-sucedida, false caso contrário
 */
bool publishCommandAck(CommandType commandType, int value);

/**
 * @brief Obtém o estado atual da porta.
 *
 * @return true se porta aberta, false se fechada
 */
bool getDoorState(void);

/**
 * @brief Obtém o estado atual da ventilação.
 *
 * @return true se ventilação ligada, false se desligada
 */
bool getVentilationState(void);

/**
 * @brief Obtém o estado atual do buzzer.
 *
 * @return Estado atual do buzzer (0-3)
 */
BuzzerState getBuzzerState(void);

/**
 * @brief Inicia a task de processamento de comandos.
 * 
 * Cria a task FreeRTOS que processa comandos da fila.
 * Deve ser chamada após commandMqttInit() e após o scheduler iniciar.
 */
void commandMqttStartTask(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_MQTT_TASK_H */
