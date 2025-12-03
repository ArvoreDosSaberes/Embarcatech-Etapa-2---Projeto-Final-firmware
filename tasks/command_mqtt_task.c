/**
 * @file command_mqtt_task.c
 * @brief Implementação do módulo de processamento de comandos MQTT.
 *
 * Este módulo processa comandos recebidos do dashboard via MQTT e
 * publica confirmações (ACK) após a execução dos comandos.
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "hardware/gpio.h"

#include "log_vt100.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "command_mqtt_task.h"
#include "buzzer_pwm_task.h"

/* Referências externas */
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];

/* Estados internos dos atuadores */
static bool doorState = false;           /* false = fechada, true = aberta */
static bool ventilationState = false;    /* false = desligada, true = ligada */
static BuzzerState buzzerState = BUZZER_OFF;

/* Pinos dos atuadores (a serem definidos em rack_inteligente_parametros.h) */
#ifndef RACK_VENTILATION_PIN
#define RACK_VENTILATION_PIN 14  /* Pino padrão para ventilação */
#endif

#ifndef RACK_BUZZER_PIN
#define RACK_BUZZER_PIN 15       /* Pino padrão para buzzer (usa RACK_ALARM_PIN) */
#endif

/**
 * @brief Inicializa os pinos GPIO dos atuadores.
 * 
 * Nota: O buzzer é inicializado pelo módulo buzzer_pwm_task.
 */
static void initActuatorPins(void) {
    /* Inicializa pino de ventilação */
    gpio_init(RACK_VENTILATION_PIN);
    gpio_set_dir(RACK_VENTILATION_PIN, GPIO_OUT);
    gpio_put(RACK_VENTILATION_PIN, 0);
    
    /* Buzzer inicializado pelo módulo buzzer_pwm_task */
    
    LOG_INFO("[Command] GPIO dos atuadores inicializados");
}

/**
 * @brief Inicializa o módulo de comandos.
 * 
 * Inicializa os GPIOs dos atuadores e o módulo de buzzer PWM.
 */
bool commandMqttInit(void) {
    initActuatorPins();
    
    /* Inicializa módulo de buzzer PWM */
    if (!buzzerPwmInit()) {
        LOG_WARN("[Command] Falha ao inicializar buzzer PWM");
        return false;
    }
    
    LOG_INFO("[Command] Módulo de comandos inicializado");
    return true;
}

/**
 * @brief Publica confirmação (ACK) via MQTT.
 */
bool publishCommandAck(CommandType commandType, int value) {
    if (!mqtt_connected) {
        LOG_WARN("[Command/ACK] MQTT não conectado, não é possível enviar ACK");
        return false;
    }
    
    char topic[80];
    char payload[8];
    const char *typeStr = "";
    
    switch (commandType) {
        case COMMAND_TYPE_DOOR:
            typeStr = "door";
            break;
        case COMMAND_TYPE_VENTILATION:
            typeStr = "ventilation";
            break;
        case COMMAND_TYPE_BUZZER:
            typeStr = "buzzer";
            break;
        default:
            LOG_WARN("[Command/ACK] Tipo de comando desconhecido: %d", commandType);
            return false;
    }
    
    /* Monta o tópico de ACK: {base}/{rack_id}/ack/{type} */
    snprintf(topic, sizeof(topic), "%s/ack/%s", mqtt_rack_topic, typeStr);
    snprintf(payload, sizeof(payload), "%d", value);
    
    LOG_INFO("[Command/ACK] Publicando ACK: tópico='%s', valor='%s'", topic, payload);
    
    err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, NULL, NULL);
    
    if (err == ERR_OK) {
        LOG_INFO("[Command/ACK] ACK enviado com sucesso");
        return true;
    } else {
        LOG_WARN("[Command/ACK] Erro ao enviar ACK: %d", err);
        return false;
    }
}

/**
 * @brief Executa comando de porta e publica ACK.
 */
void processCommandDoor(int value) {
    bool targetState = (value == 1);
    
    LOG_INFO("[Command/Door] Recebido comando: %s", targetState ? "ABRIR" : "FECHAR");
    
    /* Executa o comando */
    if (targetState) {
        /* Abrir porta - simula destravando a fechadura */
        gpio_put(RACK_DOOR_STATE_PIN, 1);
        doorState = true;
        LOG_INFO("[Command/Door] Porta destravada (aberta)");
    } else {
        /* Fechar porta - simula travando a fechadura */
        gpio_put(RACK_DOOR_STATE_PIN, 0);
        doorState = false;
        LOG_INFO("[Command/Door] Porta travada (fechada)");
    }
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_DOOR, value);
}

/**
 * @brief Executa comando de ventilação e publica ACK.
 */
void processCommandVentilation(int value) {
    bool targetState = (value == 1);
    
    LOG_INFO("[Command/Ventilation] Recebido comando: %s", targetState ? "LIGAR" : "DESLIGAR");
    
    /* Executa o comando */
    gpio_put(RACK_VENTILATION_PIN, targetState ? 1 : 0);
    ventilationState = targetState;
    
    LOG_INFO("[Command/Ventilation] Ventilação %s", targetState ? "LIGADA" : "DESLIGADA");
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_VENTILATION, value);
}

/**
 * @brief Executa comando de buzzer e publica ACK.
 * 
 * Utiliza o módulo buzzer_pwm_task para gerar tons diferentes
 * conforme o tipo de alarme:
 * - 0: Desligado
 * - 1: Porta aberta (beep intermitente)
 * - 2: Arrombamento (sirene rápida)
 * - 3: Superaquecimento (pulsos graves)
 */
void processCommandBuzzer(int value) {
    /* Valida o valor recebido */
    if (value < 0 || value > 3) {
        LOG_WARN("[Command/Buzzer] Valor inválido: %d (esperado 0-3)", value);
        return;
    }
    
    BuzzerState targetState = (BuzzerState)value;
    const char *stateNames[] = {"DESLIGADO", "PORTA_ABERTA", "ARROMBAMENTO", "SUPERAQUECIMENTO"};
    
    LOG_INFO("[Command/Buzzer] Recebido comando: %s", stateNames[value]);
    
    /* Atualiza estado interno */
    buzzerState = targetState;
    
    /* Configura buzzer PWM com padrão sonoro correspondente */
    buzzerPwmSetState((BuzzerPwmState)value);
    
    LOG_INFO("[Command/Buzzer] Buzzer configurado: %s", stateNames[value]);
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_BUZZER, value);
}

/**
 * @brief Retorna o estado atual da porta.
 */
bool getDoorState(void) {
    return doorState;
}

/**
 * @brief Retorna o estado atual da ventilação.
 */
bool getVentilationState(void) {
    return ventilationState;
}

/**
 * @brief Retorna o estado atual do buzzer.
 */
BuzzerState getBuzzerState(void) {
    return buzzerState;
}
