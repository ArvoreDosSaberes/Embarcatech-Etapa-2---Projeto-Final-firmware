#include "FreeRTOS.h"
#include "oled.h"
#include "oled_freeRTOS.h"
#include "task.h"
#include <hardware/gpio.h>
#include <string.h>

#include "rack_inteligente_parametros.h"
#include "rack_inteligente.h"

void vSignsOnTask(void *pvParameters) {
    (void) pvParameters;

    // Configura os pinos dos LEDs como saída
    gpio_init(LEDB);
    gpio_set_dir(LEDB, GPIO_OUT);

    bool state = false;
    for (;;) {
        gpio_put(LEDB, state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(RACK_SIGN_ON_TASK_DELAY));
    }
}