/**
 * @file rack_inteligente.cpp
 * @brief Firmware principal do sistema Rack Inteligente.
 *
 * Este arquivo contém o ponto de entrada do firmware e a inicialização de todos
 * os subsistemas do rack inteligente, incluindo:
 * - Conexão Wi-Fi e cliente MQTT
 * - Tasks FreeRTOS para sensores e atuadores
 * - Comunicação com dashboard via MQTT
 * - Controle de porta via servo motor
 * - Alarmes sonoros via PWM
 *
 * @section Arquitetura
 * O firmware utiliza FreeRTOS com múltiplas tasks para:
 * - Leitura de sensores (temperatura, umidade, inclinação, GPS)
 * - Monitoramento de estado da porta
 * - Comunicação MQTT (publicação e assinatura)
 * - Interface OLED com menu
 * - Teclado matricial
 *
 * @section TopicosAck Tópicos MQTT com ACK
 * O firmware implementa confirmação de comandos:
 * - Recebe: {base}/{rack_id}/command/{door|ventilation|buzzer}
 * - Publica ACK: {base}/{rack_id}/ack/{door|ventilation|buzzer}
 *
 * @author Carlos Delfino <consultoria@carlosdelfino.eti.br>
 * @author EmbarcaTech TIC-27
 * @version 1.0.0
 * @date 2025
 * 
 * @copyright Copyright (c) 2025 EmbarcaTech
 */
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "pico/cyw43_arch.h"
#include <hardware/timer.h>
#include <lwip/arch.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#include "I2C.hpp"

#include "oled.h"
#include "oled_freeRTOS.h"

#include "rack_event_groups.h"
#include "menu_event_group.h"

#include "rack_inteligente.h"
#include "rack_inteligente_parametros.h"
#include "keyboard_menu_parameters.h"

#include "log_vt100.h"

#include "gps_task.h"
#include "keyboard_task.h"
#include "tilt_task.hpp"
#include "menu_oled_task.h"
#include "network_poll_task.h"
#include "signs_on_task.h"

#include "rtos_monitor.h"

#include "door_state_callback.h"


/*
#include "tilt_oled_task.h"
#include "humidity_oled_task.h"
#include "temperature_oled_task.h"
#include "door_state_oled_task.h"
#include "gps_oled_task.h"
*/

#include "door_state_mqtt_task.h"
#include "tilt_mqtt_task.h"
#include "temperature_humidity_task.hpp"
#include "humidity_mqtt_task.h"
#include "temperature_mqtt_task.h"
#include "gps_mqtt_task.h"
#include "command_mqtt_task.h"
#include "buzzer_pwm_task.h"
#include "door_servo_task.h"
#include "rtos_monitor.h"

extern "C" {

/**
 * @defgroup GlobalVars Variáveis Globais
 * @brief Variáveis compartilhadas entre módulos do firmware.
 * @{
 */

/** @brief Objeto I2C para comunicação com sensores (DHT, OLED, etc). */
I2C i2c(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN);

/** @brief Estrutura com dados ambientais do rack (temperatura, umidade, porta, inclinação). */
environment_t environment;

/** @brief Flag indicando se cliente MQTT está conectado ao broker. */
bool mqtt_connected = false;

/** @brief Ponteiro para o cliente MQTT lwIP. */
mqtt_client_t *mqtt_client;

/** @brief Tópico base MQTT do rack no formato "{base}/{rack_id}". */
char mqtt_rack_topic[50];

/** @brief Nome/identificador do rack (definido por MQTT_RACK_NUMBER). */
char rack_name[50];

/** @} */ // fim GlobalVars

/**
 * @defgroup LocalVars Variáveis Locais
 * @brief Variáveis de uso interno do módulo principal.
 * @{
 */

/** @brief Endereço IP do broker MQTT resolvido via DNS. */
static ip_addr_t broker_ip;

/** @brief Flag indicando recepção de comando de porta via MQTT. */
static bool mqttInTopicDoor = false;

/** @brief Flag indicando recepção de comando de ventilação via MQTT. */
static bool mqttInTopicVentilation = false;

/** @brief Flag indicando recepção de comando de buzzer via MQTT. */
static bool mqttInTopicBuzzer = false;

/** @brief Flag de alerta (mantido para compatibilidade). @deprecated */
static bool mqttInTopicAlert = false;

/** @} */ // fim LocalVars

/* ============================================================================
 * Protótipos de Funções
 * ============================================================================ */

static void mqtt_connection_callback(mqtt_client_t *client, void *arg,
                                     mqtt_connection_status_t status);
static void dns_check_callback(const char *name, const ip_addr_t *ipaddr,
                               void *callback_arg);
static void inpub_cb(void *arg, const char *topic, u32_t tot_len);
static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static inline void openDoor();
static inline void closeDoor();
static inline void turnOnAlarm();
static inline void turnOffAlarm();

/**
 * @brief Função principal do firmware.
 *
 * Inicializa todos os subsistemas na seguinte ordem:
 * 1. Wi-Fi e conexão com rede
 * 2. I2C para sensores
 * 3. Display OLED
 * 4. Cliente MQTT
 * 5. Tasks FreeRTOS para sensores e atuadores
 *
 * Após inicialização, inicia o scheduler FreeRTOS.
 * Esta função não retorna em operação normal.
 *
 * @return int Código de erro (apenas em caso de falha na inicialização)
 * @retval 0 Sucesso (não deve retornar)
 * @retval -1 Falha na inicialização de Wi-Fi
 * @retval -1 Falha na conexão MQTT
 */
int main() {
  stdio_init_all();
  log_set_level(LOG_LEVEL_INFO);

  LOG_INFO("#################################");
  LOG_INFO("# Rack Inteligente              #");
  LOG_INFO("# Versão: 0.1.0                 #");
  LOG_INFO("# Autor: Carlos Delfino         #");
  LOG_INFO("# Data: 2025-11-16              #");
  LOG_INFO("#################################");

  LOG_INFO("Contagem regressiva... ____");
  for (int i = 200; i > 0; i--) {
    LOG_INFO("\r\x1b[4C%04d", i);
    sleep_ms(1);
  }

  // Inicializa Wi-Fi
  LOG_INFO("[Wi-Fi] Inicializando...");
  if (cyw43_arch_init()) {
    LOG_WARN("[Wi-Fi] Erro na inicialização do Wi-Fi");
    return -1;
  }
  cyw43_arch_enable_sta_mode();
  
  LOG_INFO("[I2C] Inicializando...");
  // NOTA: Objeto I2C deve ser estático para permanecer válido após vTaskStartScheduler
  // pois as tasks usam ponteiro para este objeto
  i2c.begin();
  i2c.setClock(I2C_BAUD_RATE);

  snprintf(rack_name, sizeof(rack_name), "%s", MQTT_RACK_NUMBER);

  LOG_INFO("[OLED] Inicializando...");
  oled_init();
  oled_clear();
  oled_set_text_line(0, "Rack Inteligente", OLED_ALIGN_CENTER);
  oled_set_text_line(2, "Iniciando...", OLED_ALIGN_CENTER);
  oled_set_text_line(6, rack_name, OLED_ALIGN_CENTER);
  oled_render_text();

  LOG_INFO("[OLED] Inicializando semaforo...");
  if (initOledSemaphore() != pdPASS) {
    LOG_WARN("[OLED] Erro ao inicializar semaforo do OLED");
    return -1;
  }
  LOG_INFO("[OLED] Semaforo inicializado com sucesso!");


  LOG_INFO("[Wi-Fi] Conectando...");
  oled_set_text_line(2, "Conectando...", OLED_ALIGN_CENTER);
  oled_render_text();
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                         CYW43_AUTH_WPA2_AES_PSK, RACK_WIFI_TIMEOUT)) {
    LOG_WARN("[Wi-Fi] Falha na conexão Wi-Fi");
    oled_set_text_line(2, "Falha na conexão Wi-Fi", OLED_ALIGN_CENTER);
    oled_render_text();
    return -1;
  } else {
    LOG_INFO("[Wi-Fi] Conectado com sucesso!");
    oled_set_text_line(2, "Conectado com sucesso!", OLED_ALIGN_CENTER);
    oled_render_text();
  }

  oled_set_text_line(4, "WiFi Conectado", OLED_ALIGN_CENTER);
  oled_set_text_line(5, WIFI_SSID, OLED_ALIGN_CENTER);

  // Configura GPIO do botão
  LOG_INFO("[GPIO] Configurando GPIO do estado da porta...");
  //oled_set_text_line(2, "Configurando GPIO", OLED_ALIGN_CENTER);
  //oled_render_text();
  gpio_init(RACK_DOOR_STATE_PIN);
  gpio_set_dir(RACK_DOOR_STATE_PIN, GPIO_IN);
  gpio_pull_up(RACK_DOOR_STATE_PIN); // <<< ATENÇÃO: pull-up ativado
  gpio_set_irq_enabled_with_callback(
      RACK_DOOR_STATE_PIN,
      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
      true,
      doorStateChangeCallBack);

  LOG_INFO("[MQTT] Configurando tópico MQTT...");
//  oled_set_text_line(2, "Configurando MQTT", OLED_ALIGN_CENTER);
//  oled_render_text();
  snprintf(mqtt_rack_topic, sizeof(mqtt_rack_topic), "%s/%s", MQTT_BASE_TOPIC,
           rack_name);

  LOG_INFO("[MQTT] Tópico MQTT base: %s", MQTT_BASE_TOPIC);
  LOG_INFO("[MQTT] Tópico MQTT rack: %s", mqtt_rack_topic);
  char mqttTopicMsg[50];
  snprintf(mqttTopicMsg, sizeof(mqttTopicMsg), "MQTT: %s", mqtt_rack_topic);
//  oled_set_text_line(2, mqttTopicMsg, OLED_ALIGN_CENTER);
//  oled_render_text();

  // Inicializa cliente MQTT
  LOG_INFO("[MQTT] Inicializando cliente MQTT...");
//  oled_set_text_line(2, "Inicializando MQTT", OLED_ALIGN_CENTER);
//  oled_render_text();
  mqtt_client = mqtt_client_new();

  // Resolve DNS do broker MQTT
  LOG_INFO("[DNS] Resolvendo broker MQTT...");
//  oled_set_text_line(2, "Resolv. broker MQTT", OLED_ALIGN_CENTER);
//  oled_render_text();
  err_t err =
      dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
  if (err == ERR_OK) {
    dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
  } else if (err == ERR_INPROGRESS) {
    LOG_INFO("[DNS] Resolvendo...");
//    oled_set_text_line(2, "Resolvendo...", OLED_ALIGN_CENTER);
//    oled_render_text();
    while (ip_addr_isany(&broker_ip)) busy_wait_ms(1000);
  } else {
    LOG_WARN("[DNS] Erro ao resolver DNS: %d", err);
//    oled_set_text_line(2, "Erro ao resolver DNS", OLED_ALIGN_CENTER);
//    oled_render_text();
    return -1;
  }

  LOG_INFO("[DNS] Resolvido: %s -> %s", MQTT_BROKER, ipaddr_ntoa(&broker_ip));
//  oled_set_text_line(3, ipaddr_ntoa(&broker_ip), OLED_ALIGN_CENTER);
//  oled_render_text();

  LOG_INFO("[MQTT] Aguardando tempo, para MQTT conectar");
  int countTime = 30;
  while(!mqtt_connected && countTime > 0){
    busy_wait_ms(1000);
    log_write(LOG_LEVEL_INFO, "[MQTT] Aguardando MQTT conectar (%d)...\r", countTime);
//    oled_set_text_line(2, "Aguardando MQTT", OLED_ALIGN_CENTER);
//    oled_render_text();
    countTime--;
  }
  if (mqtt_client == NULL) {
    LOG_WARN("[MQTT] Erro ao inicializar cliente MQTT");
//    oled_set_text_line(2, "Erro ao inicializar MQTT", OLED_ALIGN_CENTER);
//    oled_render_text();
    return -1;
  }
  LOG_INFO("[MQTT] Conectado ao broker MQTT.");
//  oled_set_text_line(2, "Conectado MQTT", OLED_ALIGN_CENTER);
//  oled_render_text();
  char mqttDoorTopic[50];
  char mqttAlertTopic[50];
  snprintf(mqttDoorTopic, sizeof(mqttDoorTopic), "%s/%s/door", MQTT_BASE_TOPIC,
           rack_name);
  snprintf(mqttAlertTopic, sizeof(mqttAlertTopic), "%s/%s/alert",
           MQTT_BASE_TOPIC, rack_name);
  mqtt_set_inpub_callback(mqtt_client, inpub_cb, indata_cb, NULL);

  LOG_INFO("[Event Group] Criando grupo de eventos...");
//  oled_set_text_line(2, "Criando grupo de eventos", OLED_ALIGN_CENTER);
//  oled_render_text();
  if (!create_event_group()) {
    LOG_WARN("[Event Group] Erro ao criar grupo de eventos");
//    oled_set_text_line(2, "Erro ao criar grupo de eventos", OLED_ALIGN_CENTER);
//    oled_render_text();
    return -1;
  }

  LOG_INFO("[Menu Event Group] Criando grupo de eventos do menu...");
//  oled_set_text_line(2, "Criando grupo de eventos do menu", OLED_ALIGN_CENTER);
//  oled_render_text();
  if(!create_menu_event_group()){
    LOG_WARN("[Menu Event Group] Erro ao criar grupo de eventos do Menu");
//    oled_set_text_line(2, "Erro ao criar grupo de eventos do Menu", OLED_ALIGN_CENTER);
//    oled_render_text();
    return -1;
  }


  // Cria tasks FreeRTOS para cada funcionalidade
  LOG_INFO("[FreeRTOS] Criando tasks...");

  xTaskCreate(vRTOSMonitorTask, "rtos_monitor",
              RTOS_MONITOR_STACK_SIZE,
              NULL, RTOS_MONITOR_TASK_PRIORITY,
              NULL);
  xTaskCreate(vNetworkPollTask, "network_poll_task",
              RACK_NETWORK_POLL_TASK_STACK_SIZE,
              NULL, RACK_NETWORK_POLL_TASK_PRIORITY,
              NULL);
  
  xTaskCreate(vTiltTask, "tilt_task",
              RACK_POLLING_TASK_STACK_SIZE,
              static_cast<void*>(&i2c), RACK_TILT_TASK_PRIORITY,
              NULL);
  xTaskCreate(vTemperatureHumidityTask, "temp_hum_task",
               RACK_TMP_HUM_TASK_STACK_SIZE,
               static_cast<void*>(&i2c), RACK_TMP_HUM_TASK_PRIORITY,
               NULL);
  xTaskCreate(vGpsTask, "gps_task",
               RACK_GPS_TASK_STACK_SIZE,
               NULL, RACK_GPS_TASK_PRIORITY,
               NULL);
  xTaskCreate( keyboard_task, "kbd_task", 
               KBD_POLL_TASK_STACK_SIZE, 
               NULL, KBD_TASK_PRIORITY, 
               NULL);

  xTaskCreate( vMenuOledTask, "menu_task",
               MENU_OLED_TASK_STACK_SIZE,
               NULL, MENU_OLED_TASK_PRIORITY,
               NULL);

  xTaskCreate(vDoorStateMqttTask, "door_state_mqtt_task",
              RACK_DOOR_MQTT_TASK_STACK_SIZE,
              NULL, RACK_DOOR_MQTT_TASK_PRIORITY,
              NULL);
  xTaskCreate(vGpsMqttTask, "gps_mqtt_task",
              RACK_GPS_MQTT_TASK_STACK_SIZE,
              NULL, RACK_GPS_MQTT_TASK_PRIORITY,
              NULL);
  xTaskCreate(vTiltMqttTask, "tilt_mqtt_task", 
              RACK_MQTT_TASK_STACK_SIZE,
              NULL, RACK_MQTT_TASK_PRIORITY,
              NULL);
  xTaskCreate(vTemperatureMqttTask, "temp_mqtt_task", 
              RACK_TMP_MQTT_TASK_STACK_SIZE,
              NULL, RACK_TMP_MQTT_TASK_PRIORITY,
              NULL);
  xTaskCreate(vHumidityMqttTask, "humidity_mqtt_task", 
              RACK_MQTT_TASK_STACK_SIZE,
              NULL, RACK_MQTT_TASK_PRIORITY,
              NULL);

  xTaskCreate(vSignsOnTask, "signs_on_task", 
              RACK_SIGN_ON_TASK_STACK_SIZE,
              NULL, RACK_SIGN_ON_TASK_PRIORITY,
              NULL);

  // Inicia o scheduler do FreeRTOS (não retorna em operação normal)
  LOG_INFO("[FreeRTOS] Iniciando scheduler...");
//  oled_set_text_line(2, "Iniciando scheduler", OLED_ALIGN_CENTER);
//  oled_render_text();
  vTaskStartScheduler();

  // Se chegar aqui, houve falha ao iniciar o scheduler
  for (;;) {
    tight_loop_contents();
  }
}

/**
 * @brief Callback chamado quando uma mensagem MQTT chega.
 * 
 * Identifica o tópico recebido para posterior processamento dos dados.
 * Tópicos suportados:
 * - {base}/{rack}/command/door - Comando de porta
 * - {base}/{rack}/command/ventilation - Comando de ventilação
 * - {base}/{rack}/command/buzzer - Comando de buzzer
 * - {base}/{rack}/door - Compat. legada (door)
 * - {base}/{rack}/alert - Compat. legada (buzzer)
 */
static void inpub_cb(void *arg, const char *topic_in, u32_t tot_len) {
  LOG_INFO("[MQTT] Mensagem chegando no tópico: %s, tamanho: %lu", topic_in,
           (unsigned long)tot_len);
  
  // Reseta todas as flags
  mqttInTopicDoor = false;
  mqttInTopicVentilation = false;
  mqttInTopicBuzzer = false;
  mqttInTopicAlert = false;
  
  char topic[80];
  
  // Verifica tópico de comando de porta (novo padrão)
  snprintf(topic, sizeof(topic), "%s/%s/command/door", MQTT_BASE_TOPIC,
           MQTT_RACK_NUMBER);
  if (strcmp(topic_in, topic) == 0) {
    mqttInTopicDoor = true;
    LOG_DEBUG("[MQTT] Tópico identificado: command/door");
    return;
  }
  
  // Verifica tópico de comando de ventilação
  snprintf(topic, sizeof(topic), "%s/%s/command/ventilation", MQTT_BASE_TOPIC,
           MQTT_RACK_NUMBER);
  if (strcmp(topic_in, topic) == 0) {
    mqttInTopicVentilation = true;
    LOG_DEBUG("[MQTT] Tópico identificado: command/ventilation");
    return;
  }
  
  // Verifica tópico de comando de buzzer
  snprintf(topic, sizeof(topic), "%s/%s/command/buzzer", MQTT_BASE_TOPIC,
           MQTT_RACK_NUMBER);
  if (strcmp(topic_in, topic) == 0) {
    mqttInTopicBuzzer = true;
    LOG_DEBUG("[MQTT] Tópico identificado: command/buzzer");
    return;
  }
  
  // Compatibilidade com tópicos legados
  snprintf(topic, sizeof(topic), "%s/%s/door", MQTT_BASE_TOPIC,
           MQTT_RACK_NUMBER);
  if (strcmp(topic_in, topic) == 0) {
    mqttInTopicDoor = true;
    LOG_DEBUG("[MQTT] Tópico legado identificado: door");
    return;
  }
  
  snprintf(topic, sizeof(topic), "%s/%s/alert", MQTT_BASE_TOPIC,
           MQTT_RACK_NUMBER);
  if (strcmp(topic_in, topic) == 0) {
    mqttInTopicAlert = true;
    LOG_DEBUG("[MQTT] Tópico legado identificado: alert");
    return;
  }
}

/**
 * @brief Callback chamado quando os dados da mensagem MQTT chegam.
 * 
 * Processa os comandos recebidos e publica confirmação (ACK) após execução.
 * O ACK é publicado no tópico {base}/{rack}/ack/{type}.
 */
static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
  // data NÃO é terminado em '\0', então cuidado
  char payload[65];
  u16_t copy_len = (len < (sizeof(payload) - 1)) ? len : (sizeof(payload) - 1);
  memcpy(payload, data, copy_len);
  payload[copy_len] = '\0';
  LOG_INFO("[MQTT] Dados recebidos (len=%u): %s", len, payload);

  // Converte payload para inteiro
  int value = 0;
  if (len >= 1) {
    value = payload[0] - '0';  // Converte char para int
  }

  // Processa comandos de porta
  if (mqttInTopicDoor) {
    LOG_INFO("[MQTT/Command] Processando comando de porta: %d", value);
    processCommandDoor(value);
  }
  // Processa comandos de ventilação
  else if (mqttInTopicVentilation) {
    LOG_INFO("[MQTT/Command] Processando comando de ventilação: %d", value);
    processCommandVentilation(value);
  }
  // Processa comandos de buzzer
  else if (mqttInTopicBuzzer) {
    LOG_INFO("[MQTT/Command] Processando comando de buzzer: %d", value);
    processCommandBuzzer(value);
  }
  // Compatibilidade legada: alert = buzzer
  else if (mqttInTopicAlert) {
    LOG_INFO("[MQTT/Command] Processando comando legado de alert: %d", value);
    if (value == 1) {
      processCommandBuzzer(BUZZER_OVERHEAT);  // Ativa buzzer
    } else {
      processCommandBuzzer(BUZZER_OFF);  // Desativa buzzer
    }
  }
}

/**
 * @brief Funções legadas mantidas para compatibilidade.
 * 
 * Estas funções agora delegam para o módulo door_servo_task
 * que controla o servo motor da porta.
 */
void openDoor() { 
  doorServoOpen(true);  /* Movimento suave */
  LOG_DEBUG("[Legacy] openDoor chamado - servo em 180°");
}

void closeDoor() { 
  doorServoClose(true);  /* Movimento suave */
  LOG_DEBUG("[Legacy] closeDoor chamado - servo em 0°");
}

void turnOnAlarm() { 
  buzzerPwmSetState(BUZZER_STATE_OVERHEAT); 
  LOG_DEBUG("[Legacy] turnOnAlarm chamado - usando PWM");
}

void turnOffAlarm() { 
  buzzerPwmOff(); 
  LOG_DEBUG("[Legacy] turnOffAlarm chamado - usando PWM");
}


/**
 * @brief Subscreve aos tópicos de comando MQTT.
 * 
 * Chamado após conexão bem-sucedida ao broker.
 */
static void subscribeToCommandTopics(mqtt_client_t *client) {
  char topic[80];
  
  // Subscreve ao tópico de comando de porta
  snprintf(topic, sizeof(topic), "%s/%s/command/door", MQTT_BASE_TOPIC, MQTT_RACK_NUMBER);
  mqtt_subscribe(client, topic, 0, NULL, NULL);
  LOG_INFO("[MQTT] Subscrito a: %s", topic);
  
  // Subscreve ao tópico de comando de ventilação
  snprintf(topic, sizeof(topic), "%s/%s/command/ventilation", MQTT_BASE_TOPIC, MQTT_RACK_NUMBER);
  mqtt_subscribe(client, topic, 0, NULL, NULL);
  LOG_INFO("[MQTT] Subscrito a: %s", topic);
  
  // Subscreve ao tópico de comando de buzzer
  snprintf(topic, sizeof(topic), "%s/%s/command/buzzer", MQTT_BASE_TOPIC, MQTT_RACK_NUMBER);
  mqtt_subscribe(client, topic, 0, NULL, NULL);
  LOG_INFO("[MQTT] Subscrito a: %s", topic);
  
  // Compatibilidade: subscreve aos tópicos legados
  snprintf(topic, sizeof(topic), "%s/%s/door", MQTT_BASE_TOPIC, MQTT_RACK_NUMBER);
  mqtt_subscribe(client, topic, 0, NULL, NULL);
  LOG_INFO("[MQTT] Subscrito (legado) a: %s", topic);
  
  snprintf(topic, sizeof(topic), "%s/%s/alert", MQTT_BASE_TOPIC, MQTT_RACK_NUMBER);
  mqtt_subscribe(client, topic, 0, NULL, NULL);
  LOG_INFO("[MQTT] Subscrito (legado) a: %s", topic);
}

// Callback de conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg,
                                     mqtt_connection_status_t status) {
  mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
  
  if (mqtt_connected) {
    LOG_INFO("[MQTT] Conectado ao broker com sucesso!");
    
    // Inicializa módulo de comandos (cria fila e inicializa hardware)
    commandMqttInit();
    
    // Inicia task de processamento de comandos (processa fila de forma assíncrona)
    commandMqttStartTask();
    
    // Subscreve aos tópicos de comando
    subscribeToCommandTopics(client);
  } else {
    LOG_WARN("[MQTT] Falha na conexão ao broker: status=%d", status);
  }
}

// Callback de DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr,
                        void *callback_arg) {

  if (ipaddr != NULL) {
    broker_ip = *ipaddr;
    LOG_INFO("[DNS] Broker IP: %s", ipaddr_ntoa(&broker_ip));

    LOG_INFO("[MQTT] Client ID: %s", MQTT_RACK_NUMBER);
    struct mqtt_connect_client_info_t ci = {
        MQTT_CLIENT_ID,
        MQTT_USERNAME,
        MQTT_PASSWORD,
        60,
        NULL,
        NULL,
        2,
        false
        // plus any extra fields in the correct order, if they exist
    };
    LOG_INFO("[MQTT] Conectando ao broker MQTT...");
    mqtt_client_connect(mqtt_client, &broker_ip, MQTT_PORT,
                        mqtt_connection_callback, NULL, &ci);
  }
}

/*-----------------------------------------------------------
 * Callbacks obrigatórios do FreeRTOS (conforme FreeRTOSConfig.h)
 *----------------------------------------------------------*/

/**
 * @brief Hook chamado pelo FreeRTOS quando detecta stack overflow.
 * 
 * Esta função é chamada quando configCHECK_FOR_STACK_OVERFLOW está habilitado
 * e uma task excede seu espaço de stack alocado.
 * 
 * @param xTask Handle da task que causou o overflow
 * @param pcTaskName Nome da task que causou o overflow
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;  
    LOG_WARN("[FreeRTOS] STACK OVERFLOW detectado na task: %s", pcTaskName);
    // Loop infinito para facilitar debug - o sistema deve ser reiniciado
    bool state = false;
    for (;;) {
      gpio_put(LEDR, state);
      state = !state;
      busy_wait_ms(200);
    }
}

/**
 * @brief Callback chamado quando alocação de memória falha.
 */
void vApplicationMallocFailedHook(void)
{
    LOG_WARN("[FreeRTOS] Memória insuficiente!");
    /* Parar execução em caso de falha de alocação */
    taskDISABLE_INTERRUPTS();
    bool state = false;
    for (;;)
    {
      gpio_put(LEDR, state);
      state = !state;
      busy_wait_ms(200);
    }
}

/**
 * @brief Função para prover memória estática para a tarefa Idle.
 *
 * @param ppxIdleTaskTCBBuffer Ponteiro para o TCB da tarefa Idle.
 * @param ppxIdleTaskStackBuffer Ponteiro para a pilha da tarefa Idle.
 * @param pulIdleTaskStackSize Tamanho da pilha.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * @brief Função para prover memória estática para a tarefa Timer.
 *
 * @param ppxTimerTaskTCBBuffer Ponteiro para o TCB da tarefa Timer.
 * @param ppxTimerTaskStackBuffer Ponteiro para a pilha da tarefa Timer.
 * @param pulTimerTaskStackSize Tamanho da pilha.
 */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
}
