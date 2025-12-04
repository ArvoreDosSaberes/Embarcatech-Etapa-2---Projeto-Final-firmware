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
#include "queue.h"

#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "hardware/gpio.h"

#include "log_vt100.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "command_mqtt_task.h"
#include "buzzer_pwm_task.h"
#include "door_servo_task.h"

/* Referências externas */
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];

/* Estados internos dos atuadores */
static bool ventilationState = false;    /* false = desligada, true = ligada */
static BuzzerState buzzerState = BUZZER_OFF;

/* Fila de comandos - permite desacoplar callback MQTT do processamento */
static QueueHandle_t commandQueue = NULL;

/* Handle da task de processamento de comandos */
static TaskHandle_t commandTaskHandle = NULL;

/** @brief Tamanho da fila de comandos */
#define COMMAND_QUEUE_SIZE      8

/** @brief Stack size da task de comandos */
#define COMMAND_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 3)

/** @brief Prioridade da task de comandos */
#define COMMAND_TASK_PRIORITY   (tskIDLE_PRIORITY + 4)

/* Nota: Estado da porta é gerenciado pelo módulo door_servo_task */

/* Pinos dos atuadores (definidos em rack_inteligente_parametros.h) */

#ifndef RACK_BUZZER_PIN
#define RACK_BUZZER_PIN 15       /* Pino padrão para buzzer (usa RACK_ALARM_PIN) */
#endif

/**
 * @brief Inicializa os pinos GPIO dos atuadores.
 * 
 * Nota: O buzzer é inicializado pelo módulo buzzer_pwm_task.
 * Nota: O servo da porta é inicializado pelo módulo door_servo_task.
 */
static void initActuatorPins(void) {
    /* Inicializa pino de ventilação */
    gpio_init(RACK_VENTILATOR_PIN);
    gpio_set_dir(RACK_VENTILATOR_PIN, GPIO_OUT);
    gpio_put(RACK_VENTILATOR_PIN, 0);
    
    /* Buzzer inicializado pelo módulo buzzer_pwm_task */
    /* Servo da porta inicializado pelo módulo door_servo_task */
    
    LOG_INFO("[Command] GPIO dos atuadores inicializados");
}

/**
 * @brief Inicializa o módulo de comandos.
 * 
 * Inicializa os GPIOs dos atuadores, módulos de buzzer/servo e cria a fila.
 */
bool commandMqttInit(void) {
    initActuatorPins();
    
    /* Cria fila de comandos */
    commandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(CommandQueueItem));
    if (commandQueue == NULL) {
        LOG_WARN("[Command] Falha ao criar fila de comandos");
        return false;
    }
    
    /* Inicializa módulo de servo da porta */
    if (!doorServoInit()) {
        LOG_WARN("[Command] Falha ao inicializar servo da porta");
        return false;
    }
    
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

/* ============================================================================
 * Funções internas de execução (chamadas pela task)
 * ============================================================================ */

/**
 * @brief Executa comando de porta (interno).
 * 
 * Controla o servo motor da porta:
 * - value = 1: Abre a porta (servo em 180°)
 * - value = 0: Fecha a porta (servo em 0°)
 * 
 * IMPORTANTE: Esta função usa vTaskDelay e só pode ser chamada
 * de uma task FreeRTOS, nunca de callbacks de rede.
 */
static void executeCommandDoor(int value) {
    bool targetState = (value == 1);
    
    LOG_INFO("[Command/Door] Executando comando: %s", targetState ? "ABRIR" : "FECHAR");
    
    /* Executa o comando via servo motor */
    if (targetState) {
        /* Abrir porta - move servo para 180° */
        doorServoOpen(true);  /* true = movimento suave */
        LOG_INFO("[Command/Door] Porta aberta (servo em 180°)");
    } else {
        /* Fechar porta - move servo para 0° */
        doorServoClose(true);  /* true = movimento suave */
        LOG_INFO("[Command/Door] Porta fechada (servo em 0°)");
    }
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_DOOR, value);
}

/**
 * @brief Executa comando de ventilação (interno).
 */
static void executeCommandVentilation(int value) {
    bool targetState = (value == 1);
    
    LOG_INFO("[Command/Ventilation] Executando comando: %s", targetState ? "LIGAR" : "DESLIGAR");
    
    /* Executa o comando */
    gpio_put(RACK_VENTILATOR_PIN, targetState ? 1 : 0);
    ventilationState = targetState;
    
    LOG_INFO("[Command/Ventilation] Ventilação %s", targetState ? "LIGADA" : "DESLIGADA");
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_VENTILATION, value);
}

/**
 * @brief Executa comando de buzzer (interno).
 */
static void executeCommandBuzzer(int value) {
    /* Valida o valor recebido */
    if (value < 0 || value > 3) {
        LOG_WARN("[Command/Buzzer] Valor inválido: %d (esperado 0-3)", value);
        return;
    }
    
    BuzzerState targetState = (BuzzerState)value;
    const char *stateNames[] = {"DESLIGADO", "PORTA_ABERTA", "ARROMBAMENTO", "SUPERAQUECIMENTO"};
    
    LOG_INFO("[Command/Buzzer] Executando comando: %s", stateNames[value]);
    
    /* Atualiza estado interno */
    buzzerState = targetState;
    
    /* Configura buzzer PWM com padrão sonoro correspondente */
    buzzerPwmSetState((BuzzerPwmState)value);
    
    LOG_INFO("[Command/Buzzer] Buzzer configurado: %s", stateNames[value]);
    
    /* Publica confirmação */
    publishCommandAck(COMMAND_TYPE_BUZZER, value);
}

/* ============================================================================
 * Task de processamento de comandos
 * ============================================================================ */

/**
 * @brief Task que processa comandos da fila.
 * 
 * Esta task roda em contexto próprio, permitindo o uso de vTaskDelay
 * para movimentos suaves do servo motor.
 */
static void commandProcessingTask(void *pvParameters) {
    (void)pvParameters;
    CommandQueueItem cmd;
    
    LOG_INFO("[Command] Task de processamento iniciada");
    
    for (;;) {
        /* Aguarda comando na fila (bloqueia até receber) */
        if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            LOG_DEBUG("[Command] Processando comando tipo=%d valor=%d", cmd.type, cmd.value);
            
            switch (cmd.type) {
                case COMMAND_TYPE_DOOR:
                    executeCommandDoor(cmd.value);
                    break;
                case COMMAND_TYPE_VENTILATION:
                    executeCommandVentilation(cmd.value);
                    break;
                case COMMAND_TYPE_BUZZER:
                    executeCommandBuzzer(cmd.value);
                    break;
                default:
                    LOG_WARN("[Command] Tipo de comando desconhecido: %d", cmd.type);
                    break;
            }
        }
    }
}

/**
 * @brief Inicia a task de processamento de comandos.
 */
void commandMqttStartTask(void) {
    if (commandQueue == NULL) {
        LOG_WARN("[Command] Fila não inicializada, chame commandMqttInit primeiro");
        return;
    }
    
    if (commandTaskHandle != NULL) {
        LOG_WARN("[Command] Task já está em execução");
        return;
    }
    
    BaseType_t ret = xTaskCreate(
        commandProcessingTask,
        "CommandTask",
        COMMAND_TASK_STACK_SIZE,
        NULL,
        COMMAND_TASK_PRIORITY,
        &commandTaskHandle
    );
    
    if (ret == pdPASS) {
        LOG_INFO("[Command] Task de comandos criada com sucesso");
    } else {
        LOG_WARN("[Command] Falha ao criar task de comandos");
    }
}

/* ============================================================================
 * API pública (enfileira comandos - seguro para callbacks)
 * ============================================================================ */

/**
 * @brief Enfileira comando de porta para processamento.
 * 
 * Esta função é segura para ser chamada de callbacks MQTT pois
 * apenas enfileira o comando sem bloquear.
 */
void processCommandDoor(int value) {
    LOG_INFO("[Command/Door] Recebido comando: %s", (value == 1) ? "ABRIR" : "FECHAR");
    
    if (commandQueue == NULL) {
        LOG_WARN("[Command/Door] Fila não inicializada");
        return;
    }
    
    CommandQueueItem cmd = {
        .type = COMMAND_TYPE_DOOR,
        .value = value
    };
    
    /* Enfileira sem bloquear (para uso em callbacks) */
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
        LOG_WARN("[Command/Door] Fila cheia, comando descartado");
    }
}

/**
 * @brief Enfileira comando de ventilação para processamento.
 */
void processCommandVentilation(int value) {
    LOG_INFO("[Command/Ventilation] Recebido comando: %s", (value == 1) ? "LIGAR" : "DESLIGAR");
    
    if (commandQueue == NULL) {
        LOG_WARN("[Command/Ventilation] Fila não inicializada");
        return;
    }
    
    CommandQueueItem cmd = {
        .type = COMMAND_TYPE_VENTILATION,
        .value = value
    };
    
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
        LOG_WARN("[Command/Ventilation] Fila cheia, comando descartado");
    }
}

/**
 * @brief Enfileira comando de buzzer para processamento.
 */
void processCommandBuzzer(int value) {
    const char *stateNames[] = {"DESLIGADO", "PORTA_ABERTA", "ARROMBAMENTO", "SUPERAQUECIMENTO"};
    
    if (value >= 0 && value <= 3) {
        LOG_INFO("[Command/Buzzer] Recebido comando: %s", stateNames[value]);
    } else {
        LOG_WARN("[Command/Buzzer] Valor inválido: %d", value);
        return;
    }
    
    if (commandQueue == NULL) {
        LOG_WARN("[Command/Buzzer] Fila não inicializada");
        return;
    }
    
    CommandQueueItem cmd = {
        .type = COMMAND_TYPE_BUZZER,
        .value = value
    };
    
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
        LOG_WARN("[Command/Buzzer] Fila cheia, comando descartado");
    }
}

/**
 * @brief Retorna o estado atual da porta.
 * 
 * Verifica o estado do servo motor:
 * - true: Porta aberta (servo em 180°)
 * - false: Porta fechada (servo em 0°)
 */
bool getDoorState(void) {
    return doorServoIsOpen();
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
