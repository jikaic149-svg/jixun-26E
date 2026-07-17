add_library(device_serial STATIC
    "device_serial.c")

target_include_directories(device_serial PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_serial PUBLIC
    bsp
    mspm0_build_options)
