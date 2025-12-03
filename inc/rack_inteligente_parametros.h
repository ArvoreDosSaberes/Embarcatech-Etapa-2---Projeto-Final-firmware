#ifndef WIFI_SSID
#error Need to define WIFI_SSID
#endif

#ifndef WIFI_PASSWORD
#error Need to define WIFI_PASSWORD
#endif

#ifndef MQTT_BROKER
#error Need to define MQTT_BROKER
#endif

#ifndef MQTT_PORT
#error Need to define MQTT_PORT
#endif

#ifndef MQTT_CLIENT_ID
#error Need to define MQTT_CLIENT_ID
#endif

#ifndef MQTT_USERNAME
#error Need to define MQTT_USERNAME
#endif

#ifndef MQTT_PASSWORD
#error Need to define MQTT_PASSWORD
#endif

#ifndef MQTT_BASE_TOPIC
#error Need to define MQTT_BASE_TOPIC
#endif

#ifndef MQTT_RACK_NUMBER
#error Need to define MQTT_RACK_NUMBER
#endif

#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define I2C_BAUD_RATE 400000

/** 
 * @brief Pino do Buzzer.
 */
#define RACK_ALARM_PIN      10
/**
 * @brief Pino que indica o estado da porta (entrada).
 */
#define RACK_DOOR_STATE_PIN 5
/**
 * @brief Pino do travamento da porta (saída).
 */
#define RACK_DOOR_LOCK_PIN  2
/**
 * @brief Tempo limite para alerta de porta aberta (20 minutos em ms).
 */
#define DOOR_OPEN_ALERT_TIMEOUT_MS  (20 * 60 * 1000)
/**
 * @brief Intervalo de verificação do timeout (1 segundo).
 */
#define DOOR_CHECK_INTERVAL_MS      (1000)



#define LEDG 13 //verde
#define LEDB 12 //azul
#define LEDR 11 //vermelho

/* Parâmetros da task do buzzer PWM */
#define RACK_BUZZER_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 2)
#define RACK_BUZZER_TASK_PRIORITY       (tskIDLE_PRIORITY + 4)
#define RACK_BUZZER_TASK_DELAY          (100)

#define RACK_OLED_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 3)
#define RACK_OLED_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)

#define RACK_MQTT_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 3)
#define RACK_MQTT_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)

#define RACK_POLLING_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 2)
#define RACK_POLLING_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define RACK_POLLING_TASK_DELAY             (500)

#define RACK_SIGN_ON_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 2)
#define RACK_SIGN_ON_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define RACK_SIGN_ON_TASK_DELAY             (1000)

#define RACK_NETWORK_POLL_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE * 2)
// NOTA: Prioridade aumentada para garantir que a stack TCP/IP seja processada regularmente
// Prioridade 0 (idle) causava perda de conexão Wi-Fi e timeouts MQTT
#define RACK_NETWORK_POLL_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)
#define RACK_NETWORK_POLL_TASK_DELAY        (300)

#define RACK_TILT_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 2)
#define RACK_TILT_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)
#define RACK_TILT_TASK_DELAY                (500)

#define RACK_GPS_TASK_DELAY                 (3000)

#define RACK_TMP_HUM_TASK_DELAY             (1000)

#define TILT_THRESHOLD                      (0.015f)


#define RACK_WIFI_TIMEOUT                   (20000)
