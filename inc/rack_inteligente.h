#ifndef RACK_INTELIGENTE_H
#define RACK_INTELIGENTE_H

#include <stdbool.h>
#include "lwip/apps/mqtt.h"

#define RACK_DOOR_OPEN true
#define RACK_DOOR_CLOSED false

#ifdef __cplusplus
extern "C" {
#endif
extern bool mqtt_connected;
extern mqtt_client_t *mqtt_client;
extern char mqtt_rack_topic[50];
extern char rack_name[50];

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint32_t time;
    float speed;
    
} gps_position_t;

typedef struct {
    float temperature;
    float humidity;
    bool door;
    bool tilt;
    gps_position_t gps_position;
} environment_t;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // RACK_INTELIGENTE_H
