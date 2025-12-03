#ifndef OLED_FREERTOS_H
#define OLED_FREERTOS_H

#include "portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t initOledSemaphore();
BaseType_t takeOled();
BaseType_t releaseOled();

#ifdef __cplusplus
}
#endif

#endif