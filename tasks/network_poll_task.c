#include "FreeRTOS.h"
#include "log_vt100.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "rack_inteligente_parametros.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

#include "network_poll_task.h"
#include "watchdog_task.h"

/* Variáveis externas para reconexão MQTT */
extern bool mqtt_connected;
extern bool mqtt_reconnect_requested;

/* Função de reconexão MQTT definida em rack_inteligente.cpp */
extern bool mqttTryReconnect(void);

/**
 * @brief Task que realiza polling da rede e gerencia reconexão MQTT.
 *
 * Esta task é responsável por:
 * - Manter o stack de rede ativo via cyw43_arch_poll()
 * - Piscar LED indicando atividade de rede
 * - Verificar e iniciar reconexão MQTT quando necessário
 *
 * @param pvParameters Não utilizado
 */
void vNetworkPollTask(void *pvParameters) {
    LOG_INFO("[Network Poll Task] Iniciando...");
    (void) pvParameters;
    bool led_on = false;
    uint32_t poll_count = 0;

    for (;;) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t loopStartUs = wcetProbeNowUs();
#endif
        printf(".");
        cyw43_arch_poll();

        watchdogKick((uint32_t)WatchdogSourceNetworkPoll);

        /* Pisca LED para indicar atividade */
        led_on = !led_on;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

        /* Verifica reconexão MQTT a cada 10 iterações (~1 segundo) */
        poll_count++;
        if (poll_count >= 10) {
            poll_count = 0;
            
            if (!mqtt_connected && mqtt_reconnect_requested) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
                const uint64_t reconnectStartUs = wcetProbeNowUs();
#endif
                LOG_DEBUG("[Network Poll] MQTT desconectado, tentando reconectar...");
                mqttTryReconnect();
#if ( ENABLE_RTOS_ANALYSIS == 1 )
                wcetProbeRecord("network_poll.mqtt_reconnect", reconnectStartUs, wcetProbeNowUs());
#endif
            }
        }

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("network_poll.loop_active", loopStartUs, wcetProbeNowUs());
#endif

        vTaskDelay(pdMS_TO_TICKS(RACK_NETWORK_POLL_TASK_DELAY));
    }
}
