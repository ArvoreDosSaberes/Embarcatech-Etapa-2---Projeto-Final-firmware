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