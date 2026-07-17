add_library(device_key STATIC
    "device_key.c")

add_library(device_key_input STATIC
    "device_key_input.c")

target_include_directories(device_key PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_include_directories(device_key_input PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_key PUBLIC
    mspm0_build_options)

target_link_libraries(device_key_input PUBLIC
    device_key
    bsp
    mspm0_build_options)
