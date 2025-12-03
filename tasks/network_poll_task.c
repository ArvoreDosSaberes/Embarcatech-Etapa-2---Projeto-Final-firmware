#include "FreeRTOS.h"
#include "log_vt100.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "rack_inteligente_parametros.h"

#include "network_poll_task.h"

/**
 * @brief Task that polls the network.
 *
 * @param pvParameters unused
 */
void vNetworkPollTask(void *pvParameters) {
    LOG_INFO("[Network Poll Task] Iniciando...\n");
    (void) pvParameters;
    bool led_on = false;

    for (;;) {
        LOG_INFO("[Network Poll Task] Polling...\n");
        cyw43_arch_poll();

        led_on = !led_on;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

        vTaskDelay(pdMS_TO_TICKS(RACK_NETWORK_POLL_TASK_DELAY));
    }
}
