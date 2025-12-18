#ifndef KEYBOARD_MENU_PARAMETERS_H
#define KEYBOARD_MENU_PARAMETERS_H

#include "FreeRTOS.h"
#include "event_groups.h"

#include "freertos_app_sizing.h"


#define KBD_POLL_TASK_STACK_SIZE       APP_TASK_STACK_DEPTH_KBD_POLL
#define KBD_TASK_PRIORITY              (tskIDLE_PRIORITY)
#define KBD_TASK_DELAY                 (500)
#define KBD_TIME_WAIT_DELAY            (1300000)

#define MENU_OLED_TASK_STACK_SIZE      APP_TASK_STACK_DEPTH_MENU_OLED
#define MENU_OLED_TASK_PRIORITY        (tskIDLE_PRIORITY + 5)
#define MENU_OLED_TASK_DELAY           (500)

#define MENU_LED_PIN 11 //vervelho

#endif // KEYBOARD_MENU_PARAMETERS_H