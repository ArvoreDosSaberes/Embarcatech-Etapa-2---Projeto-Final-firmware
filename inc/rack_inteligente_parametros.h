/**
 * @file rack_inteligente_parametros.h
 * @brief Parâmetros de configuração do firmware Rack Inteligente.
 *
 * Este arquivo define todas as constantes de configuração do firmware,
 * incluindo pinos GPIO, parâmetros de tasks FreeRTOS, timeouts e limiares.
 *
 * @section Configuracao Configuração via CMake
 * As seguintes macros devem ser definidas via CMake (env.cmake):
 * - WIFI_SSID: Nome da rede Wi-Fi
 * - WIFI_PASSWORD: Senha da rede Wi-Fi
 * - MQTT_BROKER: Endereço do broker MQTT
 * - MQTT_PORT: Porta do broker MQTT
 * - MQTT_CLIENT_ID: ID do cliente MQTT
 * - MQTT_USERNAME: Usuário MQTT
 * - MQTT_PASSWORD: Senha MQTT
 * - MQTT_BASE_TOPIC: Tópico base MQTT
 * - MQTT_RACK_NUMBER: Número/ID do rack
 *
 * @author EmbarcaTech TIC-27
 * @version 1.0.0
 * @date 2025
 */

#ifndef RACK_INTELIGENTE_PARAMETROS_H
#define RACK_INTELIGENTE_PARAMETROS_H

/* ============================================================================
 * Verificação de Macros Obrigatórias (definidas via CMake)
 * ============================================================================ */

#ifndef WIFI_SSID
#error "WIFI_SSID não definido. Configure em env.cmake."
#endif

#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD não definido. Configure em env.cmake."
#endif

#ifndef MQTT_BROKER
#error "MQTT_BROKER não definido. Configure em env.cmake."
#endif

#ifndef MQTT_PORT
#error "MQTT_PORT não definido. Configure em env.cmake."
#endif

#ifndef MQTT_CLIENT_ID
#error "MQTT_CLIENT_ID não definido. Configure em env.cmake."
#endif

#ifndef MQTT_USERNAME
#error "MQTT_USERNAME não definido. Configure em env.cmake."
#endif

#ifndef MQTT_PASSWORD
#error "MQTT_PASSWORD não definido. Configure em env.cmake."
#endif

#ifndef MQTT_BASE_TOPIC
#error "MQTT_BASE_TOPIC não definido. Configure em env.cmake."
#endif

#ifndef MQTT_RACK_NUMBER
#error "MQTT_RACK_NUMBER não definido. Configure em env.cmake."
#endif

/* ============================================================================
 * Pinos GPIO - LEDs da BitDogLab
 * ============================================================================ */

/**
 * @defgroup LedPins Pinos de LEDs
 * @brief Pinos GPIO dos LEDs RGB da placa BitDogLab.
 * @{
 */

/** @brief Pino do LED verde. */
#define LEDG 11

/** @brief Pino do LED azul. */
#define LEDB 12

/** @brief Pino do LED vermelho. */
#define LEDR 13

/** @} */ // fim LedPins

/* ============================================================================
 * Configuração I2C
 * ============================================================================ */

/**
 * @defgroup I2CConfig Configuração I2C
 * @brief Parâmetros do barramento I2C para sensores.
 * @{
 */

/** @brief Porta I2C utilizada (i2c0 ou i2c1). */
#define I2C_PORT i2c0

/** @brief Pino SDA (dados) do I2C. */
#define I2C_SDA_PIN 0

/** @brief Pino SCL (clock) do I2C. */
#define I2C_SCL_PIN 1

/** @brief Frequência do clock I2C em Hz (400kHz = Fast Mode). */
#define I2C_BAUD_RATE 400000

/** @} */ // fim I2CConfig

/* ============================================================================
 * Coordenadas GPS Padrão
 * ============================================================================ */

/**
 * @defgroup GPSDefault Coordenadas GPS Padrão
 * @brief Coordenadas GPS fixas do rack (Aquiraz-CE, Brasil).
 * @{
 */

/** @brief Latitude padrão em graus decimais. */
#define RACK_LATITUDE  (-3.9012)

/** @brief Longitude padrão em graus decimais. */
#define RACK_LONGITUDE (-38.3876)

/** @} */ // fim GPSDefault
 /** 
  * @brief Pino do Buzzer.
  */
 #define RACK_ALARM_PIN      10
 /**
  * @brief Pino que indica o estado da porta (entrada).
  */
 #define RACK_DOOR_STATE_PIN 5
 /**
  * @brief Pino do botão B (entrada).
  */
 #define RACK_BUTTON_B_PIN   6
 /**
  * @brief Pino do travamento da porta (saída).
  */
 #define RACK_DOOR_LOCK_PIN  2
/**
 * @brief Tempo limite para alerta de porta aberta (20 minutos em ms).
 */
#define RACK_DOOR_OPEN_ALERT_TIMEOUT_MS  (20 * 60 * 1000)
/**
 * @brief Intervalo de verificação do timeout (1 segundo).
 */
#define RACK_DOOR_CHECK_INTERVAL_MS      (1000)
/**
 * @brief Pino do Acionamento do Ventilador
 */
#define RACK_VENTILATOR_PIN             (LEDR)
/**
 * @brief Pino do servo motor da porta.
 * 
 * O servo controla a abertura/fechamento da porta.
 * Usa o mesmo pino que era usado para a trava.
 */
#define RACK_DOOR_SERVO_PIN             RACK_DOOR_LOCK_PIN

/** @brief Ângulo do servo quando a porta está fechada (graus) */
#define DOOR_SERVO_ANGLE_CLOSED     0
/** @brief Ângulo do servo quando a porta está aberta (graus) */
#define DOOR_SERVO_ANGLE_OPEN       179

/* Parâmetros da task do buzzer PWM */
#define RACK_BUZZER_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE * 1)
#define RACK_BUZZER_TASK_PRIORITY           (tskIDLE_PRIORITY + 4)
#define RACK_BUZZER_TASK_DELAY              (1000)

#define RACK_OLED_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 2)
#define RACK_OLED_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)

#define RACK_MQTT_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 2)
#define RACK_MQTT_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)

#define RACK_DOOR_MQTT_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 1)
#define RACK_DOOR_MQTT_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)

#define RACK_POLLING_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 1)
#define RACK_POLLING_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define RACK_POLLING_TASK_DELAY             (5000)

#define RACK_GPS_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE * 2)
#define RACK_GPS_TASK_PRIORITY              (tskIDLE_PRIORITY + 5)
/**
 * @brief Intervalo de publicação GPS em milissegundos.
 * 
 * Para posição fixa, 30 segundos é adequado para reduzir carga na rede
 * e evitar esgotamento de pbufs do LwIP.
 */
#define RACK_GPS_TASK_DELAY                 (30000)

#define RACK_GPS_MQTT_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE * 2)
#define RACK_GPS_MQTT_TASK_PRIORITY         (tskIDLE_PRIORITY + 5)
#define RACK_GPS_MQTT_TASK_DELAY            (1000)

#define RACK_TMP_HUM_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 2)
#define RACK_TMP_HUM_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define RACK_TMP_HUM_TASK_DELAY             (1000)

#define RACK_TMP_MQTT_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE * 2)
#define RACK_TMP_MQTT_TASK_PRIORITY         (tskIDLE_PRIORITY + 5)

#define RACK_SIGN_ON_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 1)
#define RACK_SIGN_ON_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define RACK_SIGN_ON_TASK_DELAY             (1000)

#define RACK_NETWORK_POLL_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE * 1)
// NOTA: Prioridade aumentada para garantir que a stack TCP/IP seja processada regularmente
// Prioridade 0 (idle) causava perda de conexão Wi-Fi e timeouts MQTT
#define RACK_NETWORK_POLL_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)
#define RACK_NETWORK_POLL_TASK_DELAY        (5000)

#define RACK_TILT_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE * 2)
#define RACK_TILT_TASK_PRIORITY             (tskIDLE_PRIORITY + 5)
#define RACK_TILT_TASK_DELAY                (2000)

#define RTOS_MONITOR_STACK_SIZE             (configMINIMAL_STACK_SIZE * 3)
#define RTOS_MONITOR_TASK_PRIORITY          (tskIDLE_PRIORITY + 2)
/**
 * @brief Número máximo de tarefas para análise de watermark.
 * 
 * Define o tamanho do buffer para armazenar informações das tarefas
 * durante a análise de stack watermark.
 */
#define RTOS_MONITOR_MAX_TASKS              (20)

/**
 * @brief Intervalo de monitoramento em milissegundos.
 */
#define RTOS_MONITOR_INTERVAL_MS            (30000)

/** @brief Tamanho da fila de comandos */
#define COMMAND_QUEUE_SIZE      8

/** @brief Stack size da task de comandos */
#define COMMAND_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 3)

/** @brief Prioridade da task de comandos */
#define COMMAND_TASK_PRIORITY   (tskIDLE_PRIORITY + 4)


/** @brief Limiar de inclinação para detecção de tilt (em g). */
#define TILT_THRESHOLD                      (0.015f)

/** @brief Timeout de conexão Wi-Fi em milissegundos. */
#define RACK_WIFI_TIMEOUT                   (20000)

/** @} */ // fim TaskConfig

#endif // RACK_INTELIGENTE_PARAMETROS_H
