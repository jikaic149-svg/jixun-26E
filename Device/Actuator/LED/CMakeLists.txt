add_library(device_led STATIC
    "device_led.c")

target_include_directories(device_led PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_led PUBLIC
    bsp
    mspm0_build_options)
