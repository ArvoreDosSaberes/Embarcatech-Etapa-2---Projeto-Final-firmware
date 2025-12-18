#include "rtos_trace.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
uint64_t time_us_64(void);
static const char *sanitizeTaskName(const char *taskName, char *out, size_t outSize)
{
    if (out == NULL || outSize == 0u)
    {
        return "?";
    }

    if (taskName == NULL)
    {
        out[0] = '?';
        out[1] = '\0';
        return out;
    }

    size_t j = 0u;
    for (size_t i = 0u; taskName[i] != '\0' && j + 1u < outSize; i++)
    {
        char c = taskName[i];
        if (c == ' ')
        {
            c = '_';
        }
        out[j++] = c;
    }
    out[j] = '\0';
    return out;
}
#ifdef __cplusplus
}
#endif

#ifndef RTOS_TRACE_MAX_LINES
#define RTOS_TRACE_MAX_LINES (64u)
#endif

#ifndef RTOS_TRACE_LINE_SIZE
#define RTOS_TRACE_LINE_SIZE (140u)
#endif

typedef struct
{
    char line[RTOS_TRACE_LINE_SIZE];
} rtos_trace_line_t;

static rtos_trace_line_t gLines[RTOS_TRACE_MAX_LINES];
static volatile uint32_t gWriteIdx;
static volatile uint32_t gReadIdx;

static rtos_trace_reason_t gNextSwitchOutReason;
static uint32_t gNextSwitchOutWaitTicks;

static void vPushLine(const char *line)
{
    uint32_t next = (uint32_t)((gWriteIdx + 1u) % RTOS_TRACE_MAX_LINES);
    if (next == gReadIdx)
    {
        return;
    }

    strncpy(gLines[gWriteIdx].line, line, RTOS_TRACE_LINE_SIZE - 1u);
    gLines[gWriteIdx].line[RTOS_TRACE_LINE_SIZE - 1u] = '\0';
    gWriteIdx = next;
}

void rtosTraceInit(void)
{
    taskENTER_CRITICAL();
    gWriteIdx = 0u;
    gReadIdx = 0u;
    gNextSwitchOutReason = RTOS_TRACE_REASON_UNKNOWN;
    gNextSwitchOutWaitTicks = 0u;
    taskEXIT_CRITICAL();
}

void rtosTraceSetNextSwitchOutReason(rtos_trace_reason_t reason, uint32_t waitTicks)
{
    gNextSwitchOutReason = reason;
    gNextSwitchOutWaitTicks = waitTicks;
}

void rtosTraceOnSwitchIn(void *tcbPtr, const char *taskName)
{
    char buf[RTOS_TRACE_LINE_SIZE];
    char taskNameSan[32];
    uint64_t ts = time_us_64();

    snprintf(buf, sizeof(buf), "[TraceKV] ts_us=%llu ev=switch_in task=%s tcb=%p",
             (unsigned long long)ts,
             sanitizeTaskName(taskName, taskNameSan, sizeof(taskNameSan)),
             tcbPtr);

    vPushLine(buf);
}

void rtosTraceOnSwitchOut(void *tcbPtr, const char *taskName)
{
    char buf[RTOS_TRACE_LINE_SIZE];
    char taskNameSan[32];
    uint64_t ts = time_us_64();

    const rtos_trace_reason_t reason = gNextSwitchOutReason;
    const uint32_t waitTicks = gNextSwitchOutWaitTicks;

    gNextSwitchOutReason = RTOS_TRACE_REASON_PREEMPT;
    gNextSwitchOutWaitTicks = 0u;

    snprintf(buf, sizeof(buf), "[TraceKV] ts_us=%llu ev=switch_out task=%s tcb=%p reason=%u wait_ticks=%u",
             (unsigned long long)ts,
             sanitizeTaskName(taskName, taskNameSan, sizeof(taskNameSan)),
             tcbPtr,
             (unsigned)reason,
             (unsigned)waitTicks);

    vPushLine(buf);
}

int rtosTracePopLogLine(char *out, uint32_t outSize)
{
    if (out == NULL || outSize == 0u)
    {
        return 0;
    }

    if (gReadIdx == gWriteIdx)
    {
        return 0;
    }

    strncpy(out, gLines[gReadIdx].line, (size_t)outSize - 1u);
    out[outSize - 1u] = '\0';
    gReadIdx = (uint32_t)((gReadIdx + 1u) % RTOS_TRACE_MAX_LINES);
    return 1;
}

#endif
