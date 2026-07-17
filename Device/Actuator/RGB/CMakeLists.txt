add_library(device_rgb STATIC
    "device_rgb.c")

target_include_directories(device_rgb PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_rgb PUBLIC
    bsp
    mspm0_build_options)
