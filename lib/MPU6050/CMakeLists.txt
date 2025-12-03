add_library(mpu6050 STATIC
    MPU6050.cpp
)

target_include_directories(mpu6050 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(mpu6050
    i2c_proxy
    log_vt100
)
