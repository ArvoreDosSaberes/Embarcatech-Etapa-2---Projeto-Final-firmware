add_library(rack_includes INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
 )

target_include_directories(rack_includes INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(rack_includes INTERFACE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    MQTT_BROKER="${MQTT_BROKER}"
    MQTT_USERNAME="${MQTT_USERNAME}"
    MQTT_PASSWORD="${MQTT_PASSWORD}"
    MQTT_BASE_TOPIC="${MQTT_BASE_TOPIC}"
    MQTT_RACK_NUMBER="${MQTT_RACK_NUMBER}"
    MQTT_PORT="${MQTT_PORT}"
    MQTT_CLIENT_ID="${MQTT_CLIENT_ID}"
)

target_link_libraries(rack_includes INTERFACE
    pico_cyw43_arch_lwip_threadsafe_background
)