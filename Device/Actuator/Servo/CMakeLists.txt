add_library(device_servo STATIC
    "device_servo.c")

target_include_directories(device_servo PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_servo PUBLIC
    bsp
    mspm0_build_options)
