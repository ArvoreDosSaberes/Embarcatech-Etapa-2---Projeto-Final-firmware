
add_library(rtos_analysis STATIC
    rtos_monitor.c
    rtos_trace.c
    wcet_probe.c
)

# Configura includes e links do FreeRTOS (centralizado)
link_freertos(rtos_analysis)

target_include_directories(rtos_analysis PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/inc
)

target_link_libraries(rtos_analysis PUBLIC
    pico_stdlib

    hardware_timer
    hardware_gpio

    log_vt100
)

