#include "FreeRTOS.h"
#include "task.h"

 #include <string.h>

#include "oled.h"
#include "oled_freeRTOS.h"

#include "rack_inteligente.h"

#include "tasks/buzzer_pwm_task.h"
#include "lib/keyboard_menu_freertos/menu_oled_task.h"

#include "oled_status_task.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "wcet_probe.h"
#endif

static void renderStatusScreen(void) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
    const uint64_t renderStartUs = wcetProbeNowUs();
#endif
    static BuzzerPwmState lastAlarmState = (BuzzerPwmState)-1;
    static char lastIpAddress[32] = {0};

    if (menuOledIsOpen()) {
        return;
    }

    BuzzerPwmState alarmState = buzzerPwmGetState();
    const char *alarmText = "ALARM:OFF";

    switch (alarmState) {
        case BUZZER_STATE_DOOR_OPEN:
            alarmText = "ALARM:DOOR";
            break;
        case BUZZER_STATE_BREAK_IN:
            alarmText = "ALARM:BREAKIN";
            break;
        case BUZZER_STATE_OVERHEAT:
            alarmText = "ALARM:HEAT";
            break;
        default:
            alarmText = "ALARM:OFF";
            break;
    }

    if ((alarmState == lastAlarmState) && (strncmp(lastIpAddress, wifiIpAddress, sizeof(lastIpAddress)) == 0)) {
        return;
    }

    if (takeOled() != pdPASS) {
        return;
    }

    oled_clear();
    oled_set_text_line(0, rack_name, OLED_ALIGN_CENTER);
    oled_set_text_line(2, wifiIpAddress, OLED_ALIGN_CENTER);
    oled_set_text_line(7, alarmText, OLED_ALIGN_CENTER);
    oled_render_text();

    lastAlarmState = alarmState;
    strncpy(lastIpAddress, wifiIpAddress, sizeof(lastIpAddress) - 1);
    lastIpAddress[sizeof(lastIpAddress) - 1] = '\0';

    releaseOled();

#if ( ENABLE_RTOS_ANALYSIS == 1 )
    wcetProbeRecord("oled_status.render", renderStartUs, wcetProbeNowUs());
#endif
}

void vOledStatusTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
#if ( ENABLE_RTOS_ANALYSIS == 1 )
        const uint64_t loopStartUs = wcetProbeNowUs();
#endif
        renderStatusScreen();

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        wcetProbeRecord("oled_status.loop_active", loopStartUs, wcetProbeNowUs());
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
