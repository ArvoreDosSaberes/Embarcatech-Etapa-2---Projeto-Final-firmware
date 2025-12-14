#ifndef DOOR_STATE_CALLBACK_H
#define DOOR_STATE_CALLBACK_H

#include <stdint.h>
#include <sys/cdefs.h>

#ifdef __cplusplus
extern "C" {
#endif

void gpioStateChangeCallBack(unsigned int gpio, uint32_t events);

#ifdef __cplusplus
}
#endif

#endif
