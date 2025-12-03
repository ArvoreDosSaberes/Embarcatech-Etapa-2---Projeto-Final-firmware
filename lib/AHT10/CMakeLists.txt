add_library(aht10 STATIC
    aht10.cpp
)

target_include_directories(aht10 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(aht10
    hardware_i2c
    i2c_proxy

    log_vt100
)