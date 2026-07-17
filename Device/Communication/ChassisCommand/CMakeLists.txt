add_library(device_chassis_command STATIC
    "device_chassis_command.c")

target_include_directories(device_chassis_command PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_chassis_command PUBLIC
    mspm0_build_options
    m)
