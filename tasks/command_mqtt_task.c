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
#include "mqtt_utils.h"

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"
#include "command_mqtt_task.h"
#include "buzzer_pwm_task.h"
#include "door_servo_task.h"
#include "watchdog_task.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

/* Referências externas */
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
/* Fila de comandos - permite desacoplar callback MQTT do processamento */
extern  QueueHandle_t commandQueue;

/* Handle da task de processamento de comandos */
extern TaskHandle_t commandTaskHandle;


/* Estados internos dos atuadores */
static bool ventilationState = false;    /* false = desligada, true = ligada */
static BuzzerState buzzerState = BUZZER_OFF;

/** @brief Tamanho da fila de comandos */
#define COMMAND_QUEUE_SIZE      8

/** @brief Prioridade da task de comandos */
#define COMMAND_TASK_PRIORITY   (tskIDLE_PRIORITY + 4)

/* Nota: Estado da porta é gerenciado pelo módulo door_servo_task */

/* Pinos dos atuadores (definidos em rack_inteligente_parametros.h) */

#ifndef RACK_BUZZER_PIN
#define RACK_BUZZER_PIN 15       /* Pino padrão para buzzer (usa RACK_ALARM_PIN) */
#endif

bool commandMqttInitialized = false;

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
    if(commandMqttInitialized) {
        LOG_WARN("[Command] Módulo de comandos já inicializado");
        return false;
    }
    
    commandMqttInitialized = true;
    
    LOG_INFO("[Command] Inicializando módulo de comandos");
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
    
    err_t err = mqttPublishSafe(mqtt_client, topic, payload, strlen(payload), 0, 0);
    
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
void commandProcessingTask(void *pvParameters) {
    (void)pvParameters;
    CommandQueueItem cmd;
    TickType_t lastStatusPublishTick = 0;
    
    LOG_INFO("[Command] Task de processamento iniciada");
    
    for (;;) {
        watchdogKick((uint32_t)WatchdogSourceCommand);

        /* Aguarda comando na fila (bloqueia até receber) */
        if (xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(500)) == pdTRUE) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
            const uint64_t cmdStartUs = wcetProbeNowUs();
#endif
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

#if ( ENABLE_RTOS_ANALYSIS == 1 )
            wcetProbeRecord("command_mqtt.cmd_process", cmdStartUs, wcetProbeNowUs());
#endif
        } else {
            if (mqtt_connected) {
                TickType_t now = xTaskGetTickCount();
                if ((now - lastStatusPublishTick) >= pdMS_TO_TICKS(60000)) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
                    const uint64_t statusStartUs = wcetProbeNowUs();
#endif
                    publishCommandAck(COMMAND_TYPE_VENTILATION, getVentilationState() ? 1 : 0);
                    publishCommandAck(COMMAND_TYPE_DOOR, getDoorState() ? 1 : 0);
                    lastStatusPublishTick = now;
#if ( ENABLE_RTOS_ANALYSIS == 1 )
                    wcetProbeRecord("command_mqtt.status_publish", statusStartUs, wcetProbeNowUs());
#endif
                }
            }
        }
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
  * @brief Enfileira comando de buzzer a partir de contexto de interrupção (ISR).
  */
 void processCommandBuzzerFromIsr(int value, BaseType_t *xHigherPriorityTaskWoken) {
     if (value < 0 || value > 3) {
         return;
     }
 
     if (commandQueue == NULL) {
         return;
     }
 
     CommandQueueItem cmd = {
         .type = COMMAND_TYPE_BUZZER,
         .value = value
     };
 
     (void)xQueueSendFromISR(commandQueue, &cmd, xHigherPriorityTaskWoken);
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
