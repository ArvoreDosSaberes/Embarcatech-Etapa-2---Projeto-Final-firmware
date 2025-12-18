#ifndef RTOS_TRACE_H
#define RTOS_TRACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ( ENABLE_RTOS_ANALYSIS == 1 )

typedef enum
{
    RTOS_TRACE_REASON_UNKNOWN = 0,
    RTOS_TRACE_REASON_PREEMPT = 1,
    RTOS_TRACE_REASON_DELAY = 2,
    RTOS_TRACE_REASON_QUEUE_RECV = 3,
    RTOS_TRACE_REASON_QUEUE_SEND = 4,
    RTOS_TRACE_REASON_NOTIFY = 5,
    RTOS_TRACE_REASON_EVENTGROUP = 6,
} rtos_trace_reason_t;

typedef enum
{
    RTOS_TRACE_EVENT_SWITCH_IN = 1,
    RTOS_TRACE_EVENT_SWITCH_OUT = 2,
    RTOS_TRACE_EVENT_MARK = 3,
} rtos_trace_event_t;

void rtosTraceInit(void);

void rtosTraceSetNextSwitchOutReason(rtos_trace_reason_t reason, uint32_t waitTicks);

void rtosTraceOnSwitchIn(void *tcbPtr, const char *taskName);
void rtosTraceOnSwitchOut(void *tcbPtr, const char *taskName);

int rtosTracePopLogLine(char *out, uint32_t outSize);

#else

static inline void rtosTraceInit(void) {}
static inline void rtosTraceSetNextSwitchOutReason(int reason, uint32_t waitTicks)
{
    (void)reason;
    (void)waitTicks;
}
static inline void rtosTraceOnSwitchIn(void *tcbPtr, const char *taskName)
{
    (void)tcbPtr;
    (void)taskName;
}
static inline void rtosTraceOnSwitchOut(void *tcbPtr, const char *taskName)
{
    (void)tcbPtr;
    (void)taskName;
}
static inline int rtosTracePopLogLine(char *out, uint32_t outSize)
{
    (void)out;
    (void)outSize;
    return 0;
}

#endif

#ifdef __cplusplus
}
#endif

#endif
