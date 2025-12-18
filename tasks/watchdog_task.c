#include "watchdog_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/watchdog.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

static volatile uint32_t receivedMask = 0;
static volatile uint32_t requiredMaskValue = 0;

static TickType_t heartbeatWindowTicks = 0;
static uint32_t watchdogTimeoutMs = 0;

static void watchdogSupervisorTask(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWindowStart = xTaskGetTickCount();

    for (;;) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t loopStartUs = wcetProbeNowUs();
#endif
        TickType_t now = xTaskGetTickCount();

        if ((now - lastWindowStart) >= heartbeatWindowTicks) {
            uint32_t currentReceived;
            uint32_t currentRequired;

            taskENTER_CRITICAL();
            currentReceived = receivedMask;
            currentRequired = requiredMaskValue;
            receivedMask = 0;
            taskEXIT_CRITICAL();

            if ((currentReceived & currentRequired) == currentRequired) {
                watchdog_update();
            }

            lastWindowStart = now;
        }

        vTaskDelay(pdMS_TO_TICKS(250));

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("watchdog.loop_active", loopStartUs, wcetProbeNowUs());
#endif
    }
}

bool watchdogTaskInit(uint32_t timeoutMs, uint32_t requiredMask, uint32_t heartbeatWindowMs) {
    if (timeoutMs == 0 || heartbeatWindowMs == 0 || heartbeatWindowMs >= timeoutMs) {
        return false;
    }

    watchdogTimeoutMs = timeoutMs;
    requiredMaskValue = requiredMask;
    heartbeatWindowTicks = pdMS_TO_TICKS(heartbeatWindowMs);

    receivedMask = 0;

    watchdog_enable(watchdogTimeoutMs, 1);

    BaseType_t result = xTaskCreate(
        watchdogSupervisorTask,
        "watchdog",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL);

    return (result == pdPASS);
}

void watchdogKick(uint32_t sourceBit) {
    taskENTER_CRITICAL();
    receivedMask |= sourceBit;
    taskEXIT_CRITICAL();
}

void watchdogSetRequiredMask(uint32_t requiredMask) {
    taskENTER_CRITICAL();
    requiredMaskValue = requiredMask;
    taskEXIT_CRITICAL();
}

uint32_t watchdogGetRequiredMask(void) {
    uint32_t value;
    taskENTER_CRITICAL();
    value = requiredMaskValue;
    taskEXIT_CRITICAL();
    return value;
}
