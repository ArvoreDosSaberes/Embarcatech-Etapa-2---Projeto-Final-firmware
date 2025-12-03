add_library(oled STATIC
    oled.c
    ssd1306_i2c.c
)

# Configuração do FreeRTOS
if(FREERTOS_ENABLED)
    # Adiciona oled_freeRTOS.c
    target_sources(oled PRIVATE
        oled_freeRTOS.c
    )
    
    # Define a macro de uso do FreeRTOS
    target_compile_definitions(oled PUBLIC
        OLED_USE_FREERTOS=1
    )
    
    # Configura includes e links do FreeRTOS (centralizado)
    link_freertos(oled)
else()
    # FreeRTOS não está disponível
    target_compile_definitions(oled PUBLIC
        OLED_USE_FREERTOS=0
    )
endif()

target_include_directories(oled PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(oled PUBLIC
    pico_stdlib
    
    i2c_proxy
    log_vt100
)