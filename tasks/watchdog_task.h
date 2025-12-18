#ifndef WATCHDOG_TASK_H
#define WATCHDOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    WatchdogSourceNetworkPoll = (1u << 0),
    WatchdogSourceCommand     = (1u << 1),
    WatchdogSourceRtosMonitor = (1u << 2)
} WatchdogSource;

bool watchdogTaskInit(uint32_t timeoutMs, uint32_t requiredMask, uint32_t heartbeatWindowMs);
void watchdogKick(uint32_t sourceBit);
void watchdogSetRequiredMask(uint32_t requiredMask);
uint32_t watchdogGetRequiredMask(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_TASK_H */
