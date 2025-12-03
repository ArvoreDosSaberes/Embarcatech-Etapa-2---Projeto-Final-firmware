
file(GLOB RACK_TASKS_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

add_library(rack_tasks STATIC
    ${RACK_TASKS_SOURCES}
)

target_include_directories(rack_tasks PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(rack_tasks
        pico_stdlib
        hardware_i2c
        hardware_dma
        hardware_timer
        hardware_adc
        hardware_pio
        hardware_clocks
        hardware_pwm
        FreeRTOS-Kernel-Heap4
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt

        rack_includes

        log_vt100
        aht10
        oled
        i2c_proxy
        mpu6050
        keyboard_menu_freertos
)