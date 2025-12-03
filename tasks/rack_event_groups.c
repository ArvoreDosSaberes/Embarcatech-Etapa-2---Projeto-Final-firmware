#include "FreeRTOS.h"
#include "event_groups.h"
#include "stdio.h"
#include <stdbool.h>

#include "oled.h"
#include "log_vt100.h"

EventGroupHandle_t xEventGroup;

bool create_event_group() {
    xEventGroup = xEventGroupCreate();
    if (xEventGroup == NULL) {
        LOG_WARN("[FreeRTOS] Erro ao criar grupo de eventos");
        return false;
    }
    LOG_INFO("[FreeRTOS] Grupo de eventos criados");
    return true;
}
