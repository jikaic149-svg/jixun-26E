add_library(device_line_tracking STATIC
    "device_line_tracking.c")

target_include_directories(device_line_tracking PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_line_tracking PUBLIC
    algorithm_line_tracking
    bsp
    mspm0_build_options
    m)
