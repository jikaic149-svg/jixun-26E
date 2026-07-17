add_library(device_encoder STATIC
    "device_encoder.c")

target_include_directories(device_encoder PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_encoder PUBLIC
    bsp
    mspm0_build_options
    m)
