#include "wcet_probe.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
uint64_t time_us_64(void);
#ifdef __cplusplus
}
#endif

#ifndef WCET_PROBE_MAX_LINES
#define WCET_PROBE_MAX_LINES (64u)
#endif

#ifndef WCET_PROBE_LINE_SIZE
#define WCET_PROBE_LINE_SIZE (140u)
#endif

typedef struct
{
    char line[WCET_PROBE_LINE_SIZE];
} wcet_probe_line_t;

static wcet_probe_line_t gLines[WCET_PROBE_MAX_LINES];
static volatile uint32_t gWriteIdx;
static volatile uint32_t gReadIdx;

static void vPushLine(const char *line)
{
    uint32_t next = (uint32_t)((gWriteIdx + 1u) % WCET_PROBE_MAX_LINES);
    if (next == gReadIdx)
    {
        return;
    }

    strncpy(gLines[gWriteIdx].line, line, WCET_PROBE_LINE_SIZE - 1u);
    gLines[gWriteIdx].line[WCET_PROBE_LINE_SIZE - 1u] = '\0';
    gWriteIdx = next;
}

static void vPushLineIsr(const char *line)
{
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    vPushLine(line);
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

uint64_t wcetProbeNowUs(void)
{
    return time_us_64();
}

static void vRecordInternal(const char *name, uint64_t startUs, uint64_t endUs, uint8_t isr)
{
    char buf[WCET_PROBE_LINE_SIZE];
    const char *taskName = NULL;

    if (isr == 0u)
    {
        taskName = pcTaskGetName(NULL);
    }

    const uint64_t durUs = (endUs >= startUs) ? (endUs - startUs) : 0u;

    snprintf(buf, sizeof(buf), "[WcetKV] ts_us=%llu name=%s dur_us=%llu task=%s isr=%u",
             (unsigned long long)endUs,
             (name != NULL) ? name : "?",
             (unsigned long long)durUs,
             (taskName != NULL) ? taskName : "ISR",
             (unsigned)isr);

    vPushLine(buf);
}

void wcetProbeRecord(const char *name, uint64_t startUs, uint64_t endUs)
{
    vRecordInternal(name, startUs, endUs, 0u);
}

void wcetProbeRecordIsr(const char *name, uint64_t startUs, uint64_t endUs)
{
    char buf[WCET_PROBE_LINE_SIZE];
    const uint64_t durUs = (endUs >= startUs) ? (endUs - startUs) : 0u;

    snprintf(buf, sizeof(buf), "[WcetKV] ts_us=%llu name=%s dur_us=%llu task=%s isr=%u",
             (unsigned long long)endUs,
             (name != NULL) ? name : "?",
             (unsigned long long)durUs,
             "ISR",
             1u);

    vPushLineIsr(buf);
}

int wcetProbePopLogLine(char *out, uint32_t outSize)
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
    gReadIdx = (uint32_t)((gReadIdx + 1u) % WCET_PROBE_MAX_LINES);
    return 1;
}

#endif
