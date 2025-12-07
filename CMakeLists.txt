# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel CACHE INTERNAL "Path to FreeRTOS kernel")

if(EXISTS "${CMAKE_SOURCE_DIR}/../env.cmake")
    include("${CMAKE_SOURCE_DIR}/../env.cmake")
    message(STATUS "Arquivo env.cmake carregado com sucesso.")
else()
    message(FATAL_ERROR "Arquivo env.cmake não encontrado!")
endif() 

# Variáveis default (caso não estejam no .env)
if(NOT DEFINED ENV{WIFI_SSID})
    message(WARNING "Variável WIFI_SSID não definida no .env.")
    set(ENV{WIFI_SSID} "ArvoreDosSaberes")
endif()
if(NOT DEFINED ENV{WIFI_PASSWORD})
    message(WARNING "Variável WIFI_PASSWORD não definida no .env.")
    set(ENV{WIFI_PASSWORD} "Arduino2022")
endif()
if(NOT DEFINED ENV{MQTT_BROKER})
    message(WARNING "Variável MQTT_BROKER não definida no .env.")
    set(ENV{MQTT_BROKER} "mqtt.rapport.tec.br")
endif()
if(NOT DEFINED ENV{MQTT_PORT})
    message(WARNING "Variável MQTT_PORT não definida no env.cmake.")
    set(ENV{MQTT_PORT} "1883")
endif()
if(NOT DEFINED ENV{MQTT_CLIENT_ID})
    message(WARNING "Variável MQTT_CLIENT_ID não definida no env.cmake.")
    set(ENV{MQTT_CLIENT_ID} "Rack Inteligente")
endif()
if(NOT DEFINED ENV{MQTT_USERNAME})
    message(FATAL_ERROR "Variável MQTT_USERNAME não definida no env.cmake. Favor, defina essa variável com o nome de usuário do seu broker MQTT.")
endif()
if(NOT DEFINED ENV{MQTT_PASSWORD})
    message(FATAL_ERROR "Variável MQTT_PASSWORD não definida no env.cmake. Favor, defina essa variável com a senha do seu broker MQTT.")
endif()
if(NOT DEFINED ENV{MQTT_BASE_TOPIC})
    message(WARNING "Variável MQTT_BASE_TOPIC não definida no env.cmake.")
    set(ENV{MQTT_BASE_TOPIC} "rack_inteligente")
endif()
if(NOT DEFINED ENV{MQTT_RACK_NUMBER})
    message(FATAL_ERROR "Variável MQTT_RACK_NUMBER não definida no env.cmake. Favor, defina essa variável com o número do seu rack.")
endif()
if(NOT DEFINED ENV{MQTT_RACK_NUMBER})
    message(FATAL_ERROR "Variável MQTT_RACK_NUMBER não definida no env.cmake. Favor, defina essa variável com o número do seu rack.")
endif()

# Nível de log: -1 desativa todos os logs; quanto maior, mais verboso
if(NOT DEFINED ENV{LOG_LEVEL} OR "$ENV{LOG_LEVEL}" STREQUAL "")
    # Valor default se não houver LOG_LEVEL no env.cmake ou se vier vazio
    # os valores devem estar entre -1 e 3
    message(WARNING "Variável LOG_LEVEL não definida no env.cmake. Favor, defina essa variável com o nível de log desejado.")
    set(ENV{LOG_LEVEL} "1")
endif()

# Habilita análise de watermark de pilha das tarefas FreeRTOS
# Defina ENABLE_STACK_WATERMARK=1 no env.cmake para ativar
if(DEFINED ENV{ENABLE_STACK_WATERMARK} AND "$ENV{ENABLE_STACK_WATERMARK}" STREQUAL "1")
    set(ENABLE_STACK_WATERMARK 1)
    message(STATUS "Stack Watermark habilitado para análise de pilha das tarefas")
else()
    set(ENABLE_STACK_WATERMARK 0)
endif()

set(LOG_LEVEL "$ENV{LOG_LEVEL}")
message(STATUS "LOG_LEVEL: ${LOG_LEVEL}")


set(WIFI_SSID "$ENV{WIFI_SSID}")
set(WIFI_PASSWORD "$ENV{WIFI_PASSWORD}")
set(MQTT_BROKER "$ENV{MQTT_BROKER}")
set(MQTT_PORT "$ENV{MQTT_PORT}")
set(MQTT_CLIENT_ID "$ENV{MQTT_CLIENT_ID}")
set(MQTT_USERNAME "$ENV{MQTT_USERNAME}")
set(MQTT_PASSWORD "$ENV{MQTT_PASSWORD}")
set(MQTT_BASE_TOPIC "$ENV{MQTT_BASE_TOPIC}")
set(MQTT_RACK_NUMBER "$ENV{MQTT_RACK_NUMBER}")

message(STATUS "WIFI_SSID: ${WIFI_SSID}")
message(STATUS "WIFI_PASSWORD: ${WIFI_PASSWORD}")
message(STATUS "MQTT_USERNAME: ${MQTT_USERNAME}")
message(STATUS "MQTT_PASSWORD: ${MQTT_PASSWORD}")
message(STATUS "MQTT_BROKER: ${MQTT_BROKER}")
message(STATUS "MQTT_PORT: ${MQTT_PORT}")
message(STATUS "MQTT_CLIENT_ID: ${MQTT_CLIENT_ID}")
message(STATUS "MQTT_BASE_TOPIC: ${MQTT_BASE_TOPIC}")
message(STATUS "MQTT_RACK_NUMBER: ${MQTT_RACK_NUMBER}")


# Define the name of an MQTT broker/server to enable this example
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "Wi-Fi SSID for examples")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "Wi-Fi password for examples")
set(MQTT_BROKER "${MQTT_BROKER}" CACHE INTERNAL "MQTT broker for examples")
set(MQTT_USERNAME "${MQTT_USERNAME}" CACHE INTERNAL "MQTT username for examples")
set(MQTT_PASSWORD "${MQTT_PASSWORD}" CACHE INTERNAL "MQTT password for examples")
set(MQTT_BASE_TOPIC "${MQTT_BASE_TOPIC}" CACHE INTERNAL "MQTT base topic for examples")
set(MQTT_RACK_NUMBER "${MQTT_RACK_NUMBER}" CACHE INTERNAL "MQTT rack number for examples")
set(MQTT_PORT "${MQTT_PORT}" CACHE INTERNAL "MQTT port for examples")
set(MQTT_CLIENT_ID "${MQTT_CLIENT_ID}" CACHE INTERNAL "MQTT client ID for examples")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(${CMAKE_SOURCE_DIR}/FreeRTOSConfig.cmake)

set(ProjectName rack_inteligente)

project(Rack_Inteligente C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Disponibiliza LOG_LEVEL como macro de compilação para todos os alvos deste projeto
add_compile_definitions(LOG_LEVEL=${LOG_LEVEL})

# Disponibiliza ENABLE_STACK_WATERMARK como macro de compilação
add_compile_definitions(ENABLE_STACK_WATERMARK=${ENABLE_STACK_WATERMARK})

add_subdirectory(tasks)
add_subdirectory(inc)
add_subdirectory(lib)

# Add executable. Default name is the project name, version 0.1
add_executable(rack_inteligente rack_inteligente.cpp) 

pico_set_program_name(rack_inteligente "rack_inteligente")
pico_set_program_version(rack_inteligente "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rack_inteligente 0)
pico_enable_stdio_usb(rack_inteligente 1)

# Add the standard library to the build
target_link_libraries(rack_inteligente PUBLIC
        pico_stdlib
        hardware_adc

        rack_includes
        log_vt100
 
        i2c_proxy
        oled
        aht10
        mpu6050
        keyboard_menu_freertos
        
        rack_tasks
        )

link_freertos(rack_inteligente)

# Add the standard include files to the build
target_include_directories(rack_inteligente PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
#pico_cyw43_arch_lwip_threadsafe_background
#pico_cyw43_arch_lwip_sys_freertos
target_link_libraries(rack_inteligente PUBLIC
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
        )

target_compile_definitions(rack_inteligente PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        MQTT_BROKER=\"${MQTT_BROKER}\"
        MQTT_USERNAME=\"${MQTT_USERNAME}\"
        MQTT_PASSWORD=\"${MQTT_PASSWORD}\"
        MQTT_BASE_TOPIC=\"${MQTT_BASE_TOPIC}\"
        MQTT_RACK_NUMBER=\"${MQTT_RACK_NUMBER}\"
        MQTT_PORT=\"${MQTT_PORT}\"
        MQTT_CLIENT_ID=\"${MQTT_CLIENT_ID}\"
    )        

pico_add_extra_outputs(rack_inteligente)

