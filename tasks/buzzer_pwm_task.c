/**
 * @file buzzer_pwm_task.c
 * @brief Implementação do módulo de buzzer PWM com padrões sonoros distintos.
 *
 * Este módulo utiliza o hardware PWM do RP2040 para gerar tons com
 * frequências e padrões diferentes conforme o tipo de alarme.
 *
 * Características dos alarmes:
 * - Porta aberta: beep-beep médio, intervalos longos (aviso)
 * - Arrombamento: sirene aguda, alternância rápida (urgência máxima)
 * - Superaquecimento: pulsos graves, ritmo moderado (alerta crítico)
 *
 * @author EmbarcaTech - Rack Inteligente
 * @date 2025
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "log_vt100.h"
#include "rack_inteligente_parametros.h"
#include "buzzer_pwm_task.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

/* ============================================================================
 * Configurações de hardware
 * ============================================================================ */

/** @brief Pino do buzzer (usa RACK_ALARM_PIN definido em rack_inteligente_parametros.h) */
#ifndef BUZZER_PWM_PIN
#define BUZZER_PWM_PIN RACK_ALARM_PIN
#endif

/** @brief Duty cycle padrão do PWM (50% = som mais forte) */
#define BUZZER_PWM_DUTY_CYCLE 50

/** @brief Frequência mínima suportada (Hz) */
#define BUZZER_MIN_FREQ 100

/** @brief Frequência máxima suportada (Hz) */
#define BUZZER_MAX_FREQ 10000

/* ============================================================================
 * Padrões sonoros para cada tipo de alarme
 * ============================================================================ */

/**
 * @brief Padrão para porta aberta.
 * 
 * Beep-beep intermitente com tom médio.
 * Frequência: 1000Hz / silêncio
 * Ritmo: beep 200ms, pausa 200ms, repetição após 2 ciclos com pausa longa
 */
static const BuzzerPattern patternDoorOpen = {
    .frequencyHigh = 1000,      /* Tom médio 1kHz */
    .frequencyLow = 0,          /* Silêncio entre beeps */
    .durationHigh = 200,        /* Beep de 200ms */
    .durationLow = 200,         /* Pausa de 200ms */
    .pauseDuration = 1000,      /* Pausa longa de 1s entre sequências */
    .cyclesBeforePause = 2      /* 2 beeps antes da pausa */
};

/**
 * @brief Padrão para arrombamento.
 * 
 * Sirene de alta urgência com alternância rápida entre dois tons.
 * Frequência: 2500Hz / 1800Hz (alternância de sirene)
 * Ritmo: muito rápido, sem pausas longas
 */
static const BuzzerPattern patternBreakIn = {
    .frequencyHigh = 2500,      /* Tom agudo alto */
    .frequencyLow = 1800,       /* Tom agudo baixo */
    .durationHigh = 100,        /* Alternância rápida 100ms */
    .durationLow = 100,         /* Alternância rápida 100ms */
    .pauseDuration = 0,         /* Sem pausa - contínuo */
    .cyclesBeforePause = 0      /* Sem ciclos de pausa */
};

/**
 * @brief Padrão para superaquecimento.
 * 
 * Pulsos graves e prolongados indicando alerta crítico.
 * Frequência: 500Hz / 300Hz (tons graves)
 * Ritmo: pulsos longos com pausas moderadas
 */
static const BuzzerPattern patternOverheat = {
    .frequencyHigh = 500,       /* Tom grave alto */
    .frequencyLow = 300,        /* Tom grave baixo */
    .durationHigh = 400,        /* Pulso longo 400ms */
    .durationLow = 400,         /* Pulso longo 400ms */
    .pauseDuration = 500,       /* Pausa de 500ms entre sequências */
    .cyclesBeforePause = 3      /* 3 pulsos antes da pausa */
};

/* ============================================================================
 * Variáveis do módulo
 * ============================================================================ */

/**
 * @brief Estado atual do buzzer.
 * 
 * Esta variável é declarada como 'volatile' porque:
 * 1. É acessada por múltiplas tasks/contextos de execução (IRQ e task do buzzer)
 * 2. Pode ser modificada por buzzerPwmSetState() em uma task e lida por buzzerPwmTask() em outra
 * 3. O modificador 'volatile' impede que o compilador otimize acessos à variável,
 *    garantindo que cada leitura busque o valor atual da memória
 * 4. Sem 'volatile', o compilador poderia cachear o valor em um registrador,
 *    fazendo com que mudanças de estado não fossem percebidas pela task do buzzer
 * 
 * Nota: 'volatile' garante visibilidade mas não atomicidade - por isso também
 * uso o mutex 'stateMutex' para sincronização thread-safe completa.
 */
static volatile BuzzerPwmState currentState = BUZZER_STATE_OFF;

/** @brief Handle da task do buzzer */
static TaskHandle_t buzzerTaskHandle = NULL;

/** @brief Slice PWM associado ao pino do buzzer */
static uint pwmSlice = 0;

/** @brief Canal PWM associado ao pino do buzzer */
static uint pwmChannel = 0;

/** @brief Flag para requisição de beep único */
static volatile bool beepRequested = false;

/** @brief Frequência do beep único requisitado */
static volatile uint32_t beepFrequency = 0;

/** @brief Duração do beep único requisitado */
static volatile uint32_t beepDuration = 0;

/** @brief Mutex para acesso thread-safe ao estado */
static SemaphoreHandle_t stateMutex = NULL;

/* ============================================================================
 * Funções internas de PWM
 * ============================================================================ */

/**
 * @brief Configura a frequência do PWM.
 *
 * Calcula e configura o divisor e wrap para atingir a frequência desejada.
 *
 * @param frequency Frequência em Hz (0 = desliga)
 */
static void pwmSetFrequency(uint32_t frequency) {
    if (frequency == 0) {
        /* Desliga o PWM */
        pwm_set_enabled(pwmSlice, false);
        gpio_put(BUZZER_PWM_PIN, 0);
        return;
    }
    
    /* Limita frequência ao range suportado */
    if (frequency < BUZZER_MIN_FREQ) frequency = BUZZER_MIN_FREQ;
    if (frequency > BUZZER_MAX_FREQ) frequency = BUZZER_MAX_FREQ;
    
    /* Obtém clock do sistema (normalmente 125MHz) */
    uint32_t clockFreq = clock_get_hz(clk_sys);
    
    /* Calcula divisor e wrap para a frequência desejada
     * PWM freq = clock_freq / (divider * (wrap + 1))
     * 
     * Para flexibilidade, usamos wrap fixo de 1000 e ajustamos o divisor:
     * divider = clock_freq / (freq * wrap)
     */
    uint32_t wrap = 1000;
    float divider = (float)clockFreq / ((float)frequency * wrap);
    
    /* Limita divisor ao range válido (1.0 a 255.9375) */
    if (divider < 1.0f) {
        divider = 1.0f;
        wrap = clockFreq / frequency;
        if (wrap > 65535) wrap = 65535;
    } else if (divider > 255.0f) {
        divider = 255.0f;
        wrap = clockFreq / (frequency * 255);
        if (wrap < 1) wrap = 1;
    }
    
    /* Configura PWM */
    pwm_set_clkdiv(pwmSlice, divider);
    pwm_set_wrap(pwmSlice, wrap);
    
    /* Configura duty cycle (50% para som mais forte) */
    uint32_t level = (wrap * BUZZER_PWM_DUTY_CYCLE) / 100;
    pwm_set_chan_level(pwmSlice, pwmChannel, level);
    
    /* Habilita PWM */
    pwm_set_enabled(pwmSlice, true);
}

/**
 * @brief Inicializa o hardware PWM para o buzzer.
 */
static void pwmHardwareInit(void) {
    /* Configura pino para função PWM */
    gpio_set_function(BUZZER_PWM_PIN, GPIO_FUNC_PWM);
    
    /* Obtém slice e canal PWM do pino */
    pwmSlice = pwm_gpio_to_slice_num(BUZZER_PWM_PIN);
    pwmChannel = pwm_gpio_to_channel(BUZZER_PWM_PIN);
    
    /* Configuração inicial do PWM (desligado) */
    pwm_config config = pwm_get_default_config();
    pwm_init(pwmSlice, &config, false);
    
    LOG_DEBUG("[Buzzer/PWM] Hardware inicializado - Slice: %u, Canal: %u", 
              pwmSlice, pwmChannel);
}

/* ============================================================================
 * Funções internas da task
 * ============================================================================ */

/**
 * @brief Obtém o padrão sonoro para o estado atual.
 *
 * @param state Estado do buzzer
 * @return Ponteiro para o padrão correspondente ou NULL se desligado
 */
static const BuzzerPattern* getPatternForState(BuzzerPwmState state) {
    switch (state) {
        case BUZZER_STATE_DOOR_OPEN:
            return &patternDoorOpen;
        case BUZZER_STATE_BREAK_IN:
            return &patternBreakIn;
        case BUZZER_STATE_OVERHEAT:
            return &patternOverheat;
        default:
            return NULL;
    }
}

/**
 * @brief Executa um ciclo do padrão sonoro.
 *
 * @param pattern Padrão a executar
 * @param cycleCount Ponteiro para contador de ciclos
 * @param inHighPhase Ponteiro para flag de fase (true = tom alto)
 */
static void executePatternCycle(const BuzzerPattern *pattern, 
                                 uint8_t *cycleCount, 
                                 bool *inHighPhase) {
    if (*inHighPhase) {
        /* Fase do tom alto */
        pwmSetFrequency(pattern->frequencyHigh);
        vTaskDelay(pdMS_TO_TICKS(pattern->durationHigh));
        *inHighPhase = false;
    } else {
        /* Fase do tom baixo ou silêncio */
        pwmSetFrequency(pattern->frequencyLow);
        vTaskDelay(pdMS_TO_TICKS(pattern->durationLow));
        *inHighPhase = true;
        (*cycleCount)++;
        
        /* Verifica se precisa de pausa longa */
        if (pattern->cyclesBeforePause > 0 && 
            *cycleCount >= pattern->cyclesBeforePause) {
            pwmSetFrequency(0);  /* Silêncio */
            vTaskDelay(pdMS_TO_TICKS(pattern->pauseDuration));
            *cycleCount = 0;
        }
    }
}

/**
 * @brief Processa beep único se requisitado.
 *
 * @return true se processou beep, false caso contrário
 */
static bool processBeepRequest(void) {
    if (!beepRequested) {
        return false;
    }
    
    /* Executa beep único */
    uint32_t freq = beepFrequency;
    uint32_t duration = beepDuration;
    beepRequested = false;
    
    LOG_DEBUG("[Buzzer/PWM] Beep único: %lu Hz, %lu ms", freq, duration);
    
    pwmSetFrequency(freq);
    vTaskDelay(pdMS_TO_TICKS(duration));
    pwmSetFrequency(0);
    
    return true;
}

/* ============================================================================
 * API pública
 * ============================================================================ */

bool buzzerPwmInit(void) {
    /* Cria mutex para acesso thread-safe */
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {
        LOG_WARN("[Buzzer/PWM] Falha ao criar mutex");
        return false;
    }
    
    /* Inicializa hardware PWM */
    pwmHardwareInit();
    
    /* Cria task do buzzer */
    BaseType_t result = xTaskCreate(
        buzzerPwmTask,
        "BuzzerPWM",
        RACK_BUZZER_TASK_STACK_SIZE,
        NULL,
        RACK_BUZZER_TASK_PRIORITY,
        &buzzerTaskHandle
    );
    
    if (result != pdPASS) {
        LOG_WARN("[Buzzer/PWM] Falha ao criar task");
        return false;
    }
    
    LOG_INFO("[Buzzer/PWM] Módulo inicializado com sucesso");
    return true;
}

void buzzerPwmSetState(BuzzerPwmState state) {
    if (state > BUZZER_STATE_OVERHEAT) {
        LOG_WARN("[Buzzer/PWM] Estado inválido: %d", state);
        return;
    }
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        BuzzerPwmState oldState = currentState;
        currentState = state;
        xSemaphoreGive(stateMutex);
        
        const char *stateNames[] = {
            "DESLIGADO", "PORTA_ABERTA", "ARROMBAMENTO", "SUPERAQUECIMENTO"
        };
        
        if (oldState != state) {
            LOG_INFO("[Buzzer/PWM] Estado alterado: %s -> %s", 
                     stateNames[oldState], stateNames[state]);
        }
    } else {
        LOG_WARN("[Buzzer/PWM] Timeout ao definir estado");
    }
}

BuzzerPwmState buzzerPwmGetState(void) {
    BuzzerPwmState state = BUZZER_STATE_OFF;
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    
    return state;
}

void buzzerPwmOff(void) {
    buzzerPwmSetState(BUZZER_STATE_OFF);
}

void buzzerPwmBeep(uint32_t frequency, uint32_t durationMs) {
    if (frequency < BUZZER_MIN_FREQ || frequency > BUZZER_MAX_FREQ) {
        LOG_WARN("[Buzzer/PWM] Frequência fora do range: %lu Hz", frequency);
        return;
    }
    
    beepFrequency = frequency;
    beepDuration = durationMs;
    beepRequested = true;
    
    /* Notifica task para processar beep imediatamente */
    if (buzzerTaskHandle != NULL) {
        xTaskNotifyGive(buzzerTaskHandle);
    }
}

void buzzerPwmTask(void *pvParameters) {
    (void)pvParameters;
    
    const BuzzerPattern *currentPattern = NULL;
    BuzzerPwmState lastState = BUZZER_STATE_OFF;
    uint8_t cycleCount = 0;
    bool inHighPhase = true;
    
    LOG_INFO("[Buzzer/PWM] Task iniciada");
    
    for (;;) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t loopStartUs = wcetProbeNowUs();
#endif
        /* Processa beep único se requisitado */
        if (processBeepRequest()) {
            /* Após beep, continua com estado atual */
#if ( ENABLE_RTOS_ANALYSIS == 1 )
            wcetProbeRecord("buzzer_pwm.beep_request", loopStartUs, wcetProbeNowUs());
#endif
            continue;
        }
        
        /* Obtém estado atual */
        BuzzerPwmState state = buzzerPwmGetState();
        
        /* Verifica se estado mudou */
        if (state != lastState) {
            lastState = state;
            currentPattern = getPatternForState(state);
            cycleCount = 0;
            inHighPhase = true;
            
            if (currentPattern == NULL) {
                /* Estado OFF - desliga buzzer */
                pwmSetFrequency(0);
            }
        }
        
        /* Executa padrão se ativo */
        if (currentPattern != NULL) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
            const uint64_t patternStartUs = wcetProbeNowUs();
#endif
            executePatternCycle(currentPattern, &cycleCount, &inHighPhase);
#if ( ENABLE_RTOS_ANALYSIS == 1 )
            wcetProbeRecord("buzzer_pwm.pattern_cycle", patternStartUs, wcetProbeNowUs());
#endif
        } else {
            /* Buzzer desligado - aguarda mudança de estado */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RACK_BUZZER_TASK_DELAY));
        }

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("buzzer_pwm.loop_active", loopStartUs, wcetProbeNowUs());
#endif
    }
}
