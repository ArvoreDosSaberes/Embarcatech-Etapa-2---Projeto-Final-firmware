/**
 * @file buzzer_pwm_task.h
 * @brief Definições e interface do módulo de buzzer PWM.
 *
 * Este módulo controla o buzzer usando PWM para gerar tons diferentes
 * conforme o tipo de alarme, com padrões de alternância distintos.
 *
 * Padrões de som por tipo de alarme:
 * - BUZZER_DOOR_OPEN: Tom médio intermitente (beep-beep lento)
 * - BUZZER_BREAK_IN: Tom agudo alternante (sirene rápida)
 * - BUZZER_OVERHEAT: Tom grave pulsante (pulsos longos)
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#ifndef BUZZER_PWM_TASK_H
#define BUZZER_PWM_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Estados do buzzer com padrões sonoros distintos.
 */
typedef enum {
    BUZZER_STATE_OFF = 0,           /**< Buzzer desligado */
    BUZZER_STATE_DOOR_OPEN = 1,     /**< Alerta de porta aberta - beep intermitente */
    BUZZER_STATE_BREAK_IN = 2,      /**< Alerta de arrombamento - sirene rápida */
    BUZZER_STATE_OVERHEAT = 3       /**< Alerta de superaquecimento - pulsos graves */
} BuzzerPwmState;

/**
 * @brief Configuração de um padrão sonoro.
 * 
 * Define as frequências e tempos para um padrão de alarme.
 */
typedef struct {
    uint32_t frequencyHigh;     /**< Frequência alta do padrão (Hz) */
    uint32_t frequencyLow;      /**< Frequência baixa do padrão (Hz) */
    uint32_t durationHigh;      /**< Duração do tom alto (ms) */
    uint32_t durationLow;       /**< Duração do tom baixo (ms) */
    uint32_t pauseDuration;     /**< Duração da pausa entre ciclos (ms) */
    uint8_t cyclesBeforePause;  /**< Ciclos antes da pausa (0 = sem pausa) */
} BuzzerPattern;

/**
 * @brief Inicializa o módulo de buzzer PWM.
 *
 * Configura o pino do buzzer para operação em PWM e cria a task
 * responsável pela geração dos padrões sonoros.
 *
 * @return true se inicialização bem-sucedida, false caso contrário
 */
bool buzzerPwmInit(void);

/**
 * @brief Define o estado atual do buzzer.
 *
 * Altera o padrão sonoro sendo executado. A mudança é aplicada
 * de forma não-bloqueante pela task do buzzer.
 *
 * @param state Novo estado do buzzer (0-3)
 */
void buzzerPwmSetState(BuzzerPwmState state);

/**
 * @brief Obtém o estado atual do buzzer.
 *
 * @return Estado atual do buzzer
 */
BuzzerPwmState buzzerPwmGetState(void);

/**
 * @brief Desliga o buzzer imediatamente.
 *
 * Interrompe qualquer padrão sonoro em execução e silencia o buzzer.
 * Equivalente a buzzerPwmSetState(BUZZER_STATE_OFF).
 */
void buzzerPwmOff(void);

/**
 * @brief Emite um beep único.
 *
 * Útil para feedback de interface. Não altera o estado do alarme.
 *
 * @param frequency Frequência do beep em Hz
 * @param durationMs Duração do beep em milissegundos
 */
void buzzerPwmBeep(uint32_t frequency, uint32_t durationMs);

/**
 * @brief Task FreeRTOS do buzzer PWM.
 *
 * Gerencia a alternância de tons conforme o estado atual do alarme.
 * Esta função é chamada internamente pelo FreeRTOS.
 *
 * @param pvParameters Parâmetros da task (não utilizado)
 */
void buzzerPwmTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_PWM_TASK_H */
