#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#include <stdio.h>
#include <sys/cdefs.h>

#include "log_vt100.h"

#include "hardware/gpio.h"

#include "door_state_task.h"
#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "rack_inteligente_parametros.h"

// Variáveis Locais
static bool last_rack_door_state = false;
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

void doorStateChangeCallBack(unsigned int gpio, uint32_t events){

  if (gpio != RACK_DOOR_STATE_PIN) {
    return;
  }
  uint rack_door_state = gpio_get(RACK_DOOR_STATE_PIN);
  LOG_INFO("[DOOR STATE.doorStateChangeCallBack] Door state changed IRQ, "
           "gpio=%u events=0x%08lx state=%u",
           gpio, (unsigned long)events, rack_door_state);

  if (rack_door_state != last_rack_door_state) {
    LOG_INFO("[DOOR STATE] Estado mudou para: %s",
             rack_door_state ? "Fechada" : "Aberta");

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xEventGroup, xDoorStateBitsToWaitFor, &xHigherPriorityTaskWoken);

    environment.door = rack_door_state;
    last_rack_door_state = rack_door_state;
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
