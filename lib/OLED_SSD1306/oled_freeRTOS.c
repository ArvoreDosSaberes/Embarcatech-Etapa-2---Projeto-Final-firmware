#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "semphr.h"

#include "log_vt100.h"
#include "oled_freeRTOS.h"

SemaphoreHandle_t oledSemaphore = NULL;

BaseType_t initOledSemaphore() {
    oledSemaphore = xSemaphoreCreateMutex();
    if (oledSemaphore == NULL) {
        LOG_WARN("[OLED] Semaphore not initialized");
        return pdFAIL;
    }
    LOG_INFO("[OLED] Semaphore initialized");
    return pdPASS;
}

BaseType_t takeOled() {
    if (oledSemaphore == NULL) {
        LOG_WARN("[OLED] Semaphore not initialized");
        return pdFAIL;
    }
    LOG_INFO("[OLED] Semaphore taken");
    return xSemaphoreTake(oledSemaphore, portMAX_DELAY);
}

BaseType_t releaseOled() {
    if (oledSemaphore == NULL) {
        LOG_WARN("[OLED] Semaphore not initialized");
        return pdFAIL;
    }
    LOG_INFO("[OLED] Semaphore released");
    return xSemaphoreGive(oledSemaphore);
}
