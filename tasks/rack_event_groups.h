#ifndef RACK_EVENT_GROUPS_H
#define RACK_EVENT_GROUPS_H

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define xHumidityBitsToWaitFor                (1 << 0)
#define xTemperatureBitsToWaitFor             (1 << 1)
#define xDoorStateBitsToWaitFor               (1 << 2)
#define xTiltBitsToWaitFor                    (1 << 3)
#define xGpsBitsToWaitFor                     (1 << 4)

#ifdef __cplusplus
extern "C" {
#endif

extern EventGroupHandle_t xEventGroup;

bool create_event_group();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // RACK_EVENT_GROUPS_H