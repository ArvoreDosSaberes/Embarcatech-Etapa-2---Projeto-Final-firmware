/**
 * @file door_servo_task.c
 * @brief Implementação do módulo de controle do servo motor da porta.
 *
 * Este módulo utiliza o hardware PWM do RP2040 para controlar um servo
 * motor que abre e fecha a porta do rack.
 *
 * Especificações do servo (padrão SG90/MG90S):
 * - Frequência PWM: 50Hz (período de 20ms)
 * - Pulso mínimo: 500µs (0°)
 * - Pulso máximo: 2500µs (180°)
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#include "FreeRTOS.h"
#include "rack_inteligente.h"
#include "task.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "log_vt100.h"
#include "rack_inteligente_parametros.h"
#include "door_servo_task.h"

/* ============================================================================
 * Configurações de hardware do servo
 * ============================================================================ */

/**
 * @brief Pino do servo motor da porta.
 * 
 * Usa RACK_DOOR_SERVO_PIN se definido, senão usa RACK_DOOR_LOCK_PIN.
 */
#ifndef RACK_DOOR_SERVO_PIN
#define RACK_DOOR_SERVO_PIN RACK_DOOR_LOCK_PIN
#endif

/** @brief Frequência do PWM para servo (Hz) - padrão 50Hz */
#define SERVO_PWM_FREQ          50

/** @brief Pulso mínimo em microsegundos (0°) */
#define SERVO_PULSE_MIN_US      500

/** @brief Pulso máximo em microsegundos (180°) */
#define SERVO_PULSE_MAX_US      2500

/** @brief Ângulo máximo do servo em graus */
#define SERVO_ANGLE_MAX         DOOR_SERVO_ANGLE_OPEN

/** @brief Delay entre passos do movimento suave (ms) */
#define SERVO_SMOOTH_STEP_DELAY 15

/** @brief Incremento de ângulo por passo no movimento suave (graus) */
#define SERVO_SMOOTH_STEP_SIZE  3

/* ============================================================================
 * Variáveis do módulo
 * ============================================================================ */

/** @brief Slice PWM associado ao pino do servo */
static uint servoSlice = 0;

/** @brief Canal PWM associado ao pino do servo */
static uint servoChannel = 0;

/** @brief Valor de wrap do PWM */
static uint32_t servoWrap = 0;

/** @brief Ângulo atual do servo (0-180) */
static volatile uint8_t currentAngle = 0;

/** @brief Estado atual do servo */
static volatile DoorServoState currentState = DOOR_SERVO_STATE_CLOSED;

/** @brief Flag indicando se o módulo foi inicializado */
static bool initialized = false;

/* ============================================================================
 * Funções internas
 * ============================================================================ */

/**
 * @brief Converte ângulo em graus para valor de duty cycle do PWM.
 *
 * Mapeia o ângulo (0-180°) para o pulso correspondente (500-2500µs)
 * e calcula o valor de level para o PWM.
 *
 * @param angle Ângulo em graus (0 a 180)
 * @return Valor de level para pwm_set_chan_level
 */
static uint32_t angleToLevel(uint8_t angle) {
    /* Limita ângulo ao range válido */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }
    
    /* Calcula pulso em microsegundos
     * pulse_us = PULSE_MIN + (angle / 180) * (PULSE_MAX - PULSE_MIN)
     */
    uint32_t pulseUs = SERVO_PULSE_MIN_US + 
        ((uint32_t)angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / SERVO_ANGLE_MAX;
    
    /* Período em microsegundos (50Hz = 20000µs) */
    uint32_t periodUs = 1000000 / SERVO_PWM_FREQ;
    
    /* Calcula level proporcional ao duty cycle
     * level = (pulse_us / period_us) * wrap
     */
    uint32_t level = (pulseUs * servoWrap) / periodUs;
    
    return level;
}

/**
 * @brief Atualiza o estado baseado no ângulo atual.
 */
static void updateState(void) {
    if (currentAngle == DOOR_SERVO_ANGLE_CLOSED) {
        currentState = DOOR_SERVO_STATE_CLOSED;
    } else if (currentAngle >= DOOR_SERVO_ANGLE_OPEN) {
        currentState = DOOR_SERVO_STATE_OPEN;
    } else {
        currentState = DOOR_SERVO_STATE_MOVING;
    }
}

/* ============================================================================
 * API pública
 * ============================================================================ */

bool doorServoInit(void) {
    if (initialized) {
        LOG_WARN("[DoorServo] Módulo já inicializado");
        return true;
    }
    
    /* Configura pino para função PWM */
    gpio_set_function(RACK_DOOR_SERVO_PIN, GPIO_FUNC_PWM);
    
    /* Obtém slice e canal PWM do pino */
    servoSlice = pwm_gpio_to_slice_num(RACK_DOOR_SERVO_PIN);
    servoChannel = pwm_gpio_to_channel(RACK_DOOR_SERVO_PIN);
    
    /* Obtém clock do sistema (normalmente 125MHz) */
    uint32_t clockFreq = clock_get_hz(clk_sys);
    
    /* Calcula divisor e wrap para 50Hz
     * PWM freq = clock_freq / (divider * (wrap + 1))
     * 
     * Para 50Hz com clock de 125MHz:
     * - Usando divisor = 125 e wrap = 20000:
     *   freq = 125000000 / (125 * 20000) = 50Hz
     */
    float divider = 125.0f;
    servoWrap = clockFreq / (SERVO_PWM_FREQ * (uint32_t)divider);
    
    /* Garante que wrap está no range válido (16 bits) */
    if (servoWrap > 65535) {
        divider = (float)clockFreq / (SERVO_PWM_FREQ * 65535);
        servoWrap = 65535;
    }
    
    /* Configura PWM */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, servoWrap);
    pwm_init(servoSlice, &config, false);
    
    /* Posiciona servo na posição fechada (0°) */
    currentAngle = DOOR_SERVO_ANGLE_CLOSED;
    currentState = DOOR_SERVO_STATE_CLOSED;
    
    uint32_t level = angleToLevel(currentAngle);
    pwm_set_chan_level(servoSlice, servoChannel, level);
    
    /* Habilita PWM */
    pwm_set_enabled(servoSlice, true);
    
    initialized = true;
    
    LOG_INFO("[DoorServo] Inicializado - Pino: %u, Slice: %u, Canal: %u, Wrap: %lu",
             RACK_DOOR_SERVO_PIN, servoSlice, servoChannel, servoWrap);
    LOG_INFO("[DoorServo] Servo posicionado em 0° (porta fechada)");
    
    return true;
}

void doorServoSetAngle(uint8_t angle) {
    if (!initialized) {
        LOG_WARN("[DoorServo] Módulo não inicializado");
        return;
    }
    
    /* Limita ângulo ao range válido */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }
    
    /* Calcula e aplica nível PWM */
    uint32_t level = angleToLevel(angle);
    pwm_set_chan_level(servoSlice, servoChannel, level);
    
    currentAngle = angle;
    updateState();
    
    LOG_DEBUG("[DoorServo] Ângulo definido: %u° (level=%lu)", angle, level);
}

uint8_t doorServoGetAngle(void) {
    return currentAngle;
}

DoorServoState doorServoGetState(void) {
    return currentState;
}

void doorServoOpen(bool smooth) {
    if (!initialized) {
        LOG_WARN("[DoorServo] Módulo não inicializado");
        return;
    }
    
    LOG_INFO("[DoorServo] Abrindo porta (smooth=%s)", smooth ? "sim" : "não");
    
    if (smooth) {
        /* Movimento suave - incrementa gradualmente */
        currentState = DOOR_SERVO_STATE_MOVING;
        
        while (currentAngle < DOOR_SERVO_ANGLE_OPEN) {
            uint8_t nextAngle = currentAngle + SERVO_SMOOTH_STEP_SIZE;
            if (nextAngle > DOOR_SERVO_ANGLE_OPEN) {
                nextAngle = DOOR_SERVO_ANGLE_OPEN;
            }
            
            doorServoSetAngle(nextAngle);
            vTaskDelay(pdMS_TO_TICKS(SERVO_SMOOTH_STEP_DELAY));
        }
    } else {
        /* Movimento imediato */
        doorServoSetAngle(DOOR_SERVO_ANGLE_OPEN);
    }
    
    currentState = DOOR_SERVO_STATE_OPEN;
    LOG_INFO("[DoorServo] Porta aberta (180°)");
}

void doorServoClose(bool smooth) {
    if (!initialized) {
        LOG_WARN("[DoorServo] Módulo não inicializado");
        return;
    }
    
    LOG_INFO("[DoorServo] Fechando porta (smooth=%s)", smooth ? "sim" : "não");
    
    if (smooth) {
        /* Movimento suave - decrementa gradualmente */
        currentState = DOOR_SERVO_STATE_MOVING;
        
        while (currentAngle > DOOR_SERVO_ANGLE_CLOSED) {
            int16_t nextAngle = (int16_t)currentAngle - SERVO_SMOOTH_STEP_SIZE;
            if (nextAngle < DOOR_SERVO_ANGLE_CLOSED) {
                nextAngle = DOOR_SERVO_ANGLE_CLOSED;
            }
            
            doorServoSetAngle((uint8_t)nextAngle);
            vTaskDelay(pdMS_TO_TICKS(SERVO_SMOOTH_STEP_DELAY));
        }
    } else {
        /* Movimento imediato */
        doorServoSetAngle(DOOR_SERVO_ANGLE_CLOSED);
    }
    
    currentState = DOOR_SERVO_STATE_CLOSED;
    LOG_INFO("[DoorServo] Porta fechada (0°)");
}

bool doorServoIsOpen(void) {
    return (currentAngle >= DOOR_SERVO_ANGLE_OPEN);
}

bool doorServoIsClosed(void) {
    return (currentAngle == DOOR_SERVO_ANGLE_CLOSED);
}

void doorServoDisable(void) {
    if (!initialized) {
        return;
    }
    
    pwm_set_enabled(servoSlice, false);
    LOG_DEBUG("[DoorServo] PWM desabilitado");
}

void doorServoEnable(void) {
    if (!initialized) {
        return;
    }
    
    pwm_set_enabled(servoSlice, true);
    LOG_DEBUG("[DoorServo] PWM habilitado");
}
