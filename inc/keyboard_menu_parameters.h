#ifndef KEYBOARD_MENU_PARAMETERS_H
#define KEYBOARD_MENU_PARAMETERS_H

#include "FreeRTOS.h"
#include "event_groups.h"


#define KBD_POLL_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define KBD_TASK_PRIORITY              (tskIDLE_PRIORITY)
#define KBD_TASK_DELAY                 (500)
#define KBD_TIME_WAIT_DELAY            (1300000)

#define MENU_OLED_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)
#define MENU_OLED_TASK_PRIORITY        (tskIDLE_PRIORITY + 5)
#define MENU_OLED_TASK_DELAY           (500)

#define MENU_LED_PIN 11 //vervelho

#endif // KEYBOARD_MENU_PARAMETERS_H