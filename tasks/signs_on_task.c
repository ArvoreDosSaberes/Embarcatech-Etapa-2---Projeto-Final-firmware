#include "FreeRTOS.h"
#include "log_vt100.h"
#include "oled.h"
#include "oled_freeRTOS.h"
#include "timers.h"
#include "task.h"
#include <hardware/gpio.h>
#include <string.h>

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

static TimerHandle_t signsOnTimerHandle = NULL;
static bool signsOnLedState = false;

static void signsOnTimerCallback(TimerHandle_t timerHandle) {
    (void)timerHandle;

#if ( ENABLE_RTOS_ANALYSIS == 1 )
    const uint64_t cbStartUs = wcetProbeNowUs();
#endif
    gpio_put(LEDB, signsOnLedState);
    signsOnLedState = !signsOnLedState;

#if ( ENABLE_RTOS_ANALYSIS == 1 )
    wcetProbeRecord("signs_on.timer_cb", cbStartUs, wcetProbeNowUs());
#endif
}

void signsOnInit(void) {
    if (signsOnTimerHandle != NULL) {
        return;
    }

    gpio_init(LEDB);
    gpio_set_dir(LEDB, GPIO_OUT);
    gpio_put(LEDB, false);

    signsOnLedState = false;
    signsOnTimerHandle = xTimerCreate("signs_on_tm",
                                     pdMS_TO_TICKS(RACK_SIGN_ON_TASK_DELAY),
                                     pdTRUE,
                                     NULL,
                                     signsOnTimerCallback);
    if (signsOnTimerHandle == NULL) {
        LOG_WARN("SignsOnTimer: falha ao criar timer");
    }
}

void signsOnStart(void) {
    signsOnInit();
    if (signsOnTimerHandle == NULL) {
        return;
    }
    (void)xTimerStart(signsOnTimerHandle, 0);
}

void signsOnStop(void) {
    if (signsOnTimerHandle == NULL) {
        return;
    }
    (void)xTimerStop(signsOnTimerHandle, 0);
    gpio_put(LEDB, false);
    signsOnLedState = false;
}
