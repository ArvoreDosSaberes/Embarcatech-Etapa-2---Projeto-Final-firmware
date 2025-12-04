#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#include <stdio.h>
#include <sys/cdefs.h>

#include "hardware/gpio.h"

// NOTA: LOG_INFO removido do ISR callback - não é seguro usar printf em contexto de interrupção

#include "door_state_callback.h"
#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "rack_inteligente_parametros.h"

// Variáveis Locais
static bool last_rack_door_state = false;
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

void doorStateChangeCallBack(unsigned int gpio, uint32_t events){
  // NOTA: Callback de ISR - NÃO usar LOG_INFO aqui pois printf não é ISR-safe
  // e pode causar corrupção da saída serial ou travamento

  if (gpio != RACK_DOOR_STATE_PIN) {
    return;
  }
  uint rack_door_state = !gpio_get(RACK_DOOR_STATE_PIN);

  if (rack_door_state != last_rack_door_state) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xEventGroup, xDoorStateBitsToWaitFor, &xHigherPriorityTaskWoken);

    environment.door = rack_door_state;
    last_rack_door_state = rack_door_state;
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
