/**
 * @file door_servo_task.h
 * @brief Interface do módulo de controle do servo motor da porta.
 *
 * Este módulo controla um servo motor via PWM para abrir e fechar
 * a porta do rack. O servo gira entre 0° (fechado) e 180° (aberto).
 *
 * Características:
 * - PWM a 50Hz (padrão para servos)
 * - Pulso de 500µs (0°) a 2500µs (180°)
 * - Movimento suave com interpolação opcional
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#ifndef DOOR_SERVO_TASK_H
#define DOOR_SERVO_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Definições de ângulos da porta
 * ============================================================================ */

/** @brief Ângulo do servo quando a porta está fechada (graus) */
#define DOOR_SERVO_ANGLE_CLOSED     0

/** @brief Ângulo do servo quando a porta está aberta (graus) */
#define DOOR_SERVO_ANGLE_OPEN       180

/* ============================================================================
 * Estados do servo
 * ============================================================================ */

/**
 * @brief Estados possíveis do servo da porta.
 */
typedef enum {
    DOOR_SERVO_STATE_CLOSED = 0,    /**< Porta fechada (0°) */
    DOOR_SERVO_STATE_OPEN,          /**< Porta aberta (180°) */
    DOOR_SERVO_STATE_MOVING         /**< Servo em movimento */
} DoorServoState;

/* ============================================================================
 * API pública
 * ============================================================================ */

/**
 * @brief Inicializa o módulo de servo da porta.
 *
 * Configura o PWM no pino designado para controle do servo.
 * Posiciona o servo na posição fechada (0°) ao inicializar.
 *
 * @return true se inicialização bem-sucedida, false caso contrário
 */
bool doorServoInit(void);

/**
 * @brief Abre a porta (move servo para 180°).
 *
 * Move o servo para a posição de porta aberta.
 * Se smooth = true, o movimento é gradual.
 *
 * @param smooth true para movimento suave, false para movimento imediato
 */
void doorServoOpen(bool smooth);

/**
 * @brief Fecha a porta (move servo para 0°).
 *
 * Move o servo para a posição de porta fechada.
 * Se smooth = true, o movimento é gradual.
 *
 * @param smooth true para movimento suave, false para movimento imediato
 */
void doorServoClose(bool smooth);

/**
 * @brief Define o ângulo do servo diretamente.
 *
 * @param angle Ângulo em graus (0 a 180)
 */
void doorServoSetAngle(uint8_t angle);

/**
 * @brief Obtém o ângulo atual do servo.
 *
 * @return Ângulo atual em graus (0 a 180)
 */
uint8_t doorServoGetAngle(void);

/**
 * @brief Obtém o estado atual do servo da porta.
 *
 * @return Estado atual (CLOSED, OPEN ou MOVING)
 */
DoorServoState doorServoGetState(void);

/**
 * @brief Verifica se a porta está aberta.
 *
 * @return true se porta aberta (180°), false caso contrário
 */
bool doorServoIsOpen(void);

/**
 * @brief Verifica se a porta está fechada.
 *
 * @return true se porta fechada (0°), false caso contrário
 */
bool doorServoIsClosed(void);

/**
 * @brief Desabilita o PWM do servo.
 *
 * Útil para economizar energia quando o servo não precisa
 * manter torque na posição.
 */
void doorServoDisable(void);

/**
 * @brief Habilita o PWM do servo.
 *
 * Reativa o PWM após ter sido desabilitado.
 */
void doorServoEnable(void);

#ifdef __cplusplus
}
#endif

#endif /* DOOR_SERVO_TASK_H */
