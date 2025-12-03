add_library(flash_persist STATIC
    flash_persist.c
)

# Configuração do FreeRTOS
if(FREERTOS_ENABLED)
    # Adiciona I2C_freeRTOS.cpp
    target_sources(flash_persist PRIVATE
        flash_persist.c
    )
    
    # Define a macro de uso do FreeRTOS
    target_compile_definitions(flash_persist PUBLIC
        FLASH_PERSIST_USE_FREERTOS=1
    )
    
    # Configura includes e links do FreeRTOS (centralizado)
    link_freertos(flash_persist)
else()
    # FreeRTOS não está disponível
    target_compile_definitions(flash_persist PUBLIC
        FLASH_PERSIST_USE_FREERTOS=0
    )
endif()

target_include_directories(flash_persist PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(flash_persist PUBLIC
    pico_stdlib
    hardware_flash
    hardware_sync
    
    log_vt100
)

