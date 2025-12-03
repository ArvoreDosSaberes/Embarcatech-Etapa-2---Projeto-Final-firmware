#include "FreeRTOS.h"
#include "log_vt100.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

#include "rack_event_groups.h"
#include "rack_inteligente.h"
#include "task.h"


extern char mqtt_rack_topic[50];
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

static inline void publish_rack_gps_position();

void vGpsMqttTask(void *pvParameters) {
    LOG_INFO("[Gps MQTT Task] Iniciando...");
    (void) pvParameters;

    EventBits_t xEventBits;

    for(;;) {
        xEventBits = xEventGroupWaitBits( xEventGroup,
             xGpsBitsToWaitFor,
             pdTRUE, 
             pdTRUE,
             portMAX_DELAY);

        LOG_INFO("[Gps MQTT Task] Evento recebido: %02x", xEventBits);
        if(xEventBits & xGpsBitsToWaitFor){
            LOG_INFO("[Gps MQTT Task] Gps detectado!");
            LOG_INFO("[Gps MQTT Task] Latitude: %f", environment.gps_position.latitude);
            LOG_INFO("[Gps MQTT Task] Longitude: %f", environment.gps_position.longitude);
            LOG_INFO("[Gps MQTT Task] Altitude: %f", environment.gps_position.altitude);
            LOG_INFO("[Gps MQTT Task] Time: %d", environment.gps_position.time);
            LOG_INFO("[Gps MQTT Task] Speed: %f", environment.gps_position.speed);
            publish_rack_gps_position();
        }else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static inline void publish_rack_gps_position() {
    char mqttTopicMsg[50];
    snprintf(mqttTopicMsg, sizeof(mqttTopicMsg), "%s/gps", mqtt_rack_topic);
    char gps_position_json[100];
    snprintf(gps_position_json, sizeof(gps_position_json), "{"\
        "\"latitude\": %f, "\
        "\"longitude\": %f, "\
        "\"altitude\": %f, "\
        "\"time\": %d, "\
        "\"speed\": %f }",\
        environment.gps_position.latitude, 
        environment.gps_position.longitude, 
        environment.gps_position.altitude, 
        environment.gps_position.time, 
        environment.gps_position.speed);
    mqtt_publish(mqtt_client, 
        mqttTopicMsg, 
        gps_position_json, 
        strlen(gps_position_json), 
        2,
        0,
        NULL,
        NULL);
}
