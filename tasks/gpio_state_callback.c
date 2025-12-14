#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#include <stdio.h>
#include <sys/cdefs.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"

// NOTA: LOG_INFO removido do ISR callback - não é seguro usar printf em contexto de interrupção

#include "gpio_state_callback.h"
#include "command_mqtt_task.h"
#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "rack_inteligente_parametros.h"

// Variáveis Locais
static bool last_rack_door_state = false;
static uint64_t lastButtonBIrqUs = 0;
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

void gpioStateChangeCallBack(unsigned int gpio, uint32_t events){
  // NOTA: Callback de ISR - NÃO usar LOG_INFO aqui pois printf não é ISR-safe
  // e pode causar corrupção da saída serial ou travamento

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (gpio == RACK_DOOR_STATE_PIN) {
    uint rack_door_state = !gpio_get(RACK_DOOR_STATE_PIN);

    if (rack_door_state != last_rack_door_state) {
      xEventGroupSetBitsFromISR(xEventGroup, xDoorStateBitsToWaitFor, &xHigherPriorityTaskWoken);

      environment.door = rack_door_state;
      last_rack_door_state = rack_door_state;
    }
  } else if (gpio == RACK_BUTTON_B_PIN) {
    if ((events & GPIO_IRQ_EDGE_FALL) != 0) {
      uint64_t nowUs = time_us_64();
      if ((nowUs - lastButtonBIrqUs) > 200000) {
        lastButtonBIrqUs = nowUs;
        processCommandBuzzerFromIsr(BUZZER_OFF, &xHigherPriorityTaskWoken);
      }
    }
  } else {
    return;
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
