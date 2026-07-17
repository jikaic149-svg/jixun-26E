add_library(device_motor STATIC
    "device_motor.c")

target_include_directories(device_motor PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_motor PUBLIC
    bsp
    mspm0_build_options)
