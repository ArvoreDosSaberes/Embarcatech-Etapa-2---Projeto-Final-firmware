#ifndef WCET_PROBE_H
#define WCET_PROBE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ( ENABLE_RTOS_ANALYSIS == 1 )

uint64_t wcetProbeNowUs(void);

void wcetProbeRecord(const char *name, uint64_t startUs, uint64_t endUs);
void wcetProbeRecordIsr(const char *name, uint64_t startUs, uint64_t endUs);

int wcetProbePopLogLine(char *out, uint32_t outSize);

#else

static inline uint64_t wcetProbeNowUs(void) { return 0u; }
static inline void wcetProbeRecord(const char *name, uint64_t startUs, uint64_t endUs)
{
    (void)name;
    (void)startUs;
    (void)endUs;
}
static inline void wcetProbeRecordIsr(const char *name, uint64_t startUs, uint64_t endUs)
{
    (void)name;
    (void)startUs;
    (void)endUs;
}
static inline int wcetProbePopLogLine(char *out, uint32_t outSize)
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
