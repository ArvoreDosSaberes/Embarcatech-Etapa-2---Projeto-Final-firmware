#ifndef DOOR_STATE_TASK_H
#define DOOR_STATE_TASK_H

#include <stdint.h>
#include <sys/cdefs.h>

#ifdef __cplusplus
extern "C" {
#endif

void doorStateChangeCallBack(unsigned int gpio, uint32_t events);

#ifdef __cplusplus
}
#endif

#endif
