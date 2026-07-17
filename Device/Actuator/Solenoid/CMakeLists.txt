add_library(device_solenoid STATIC
    "device_solenoid.c")

target_include_directories(device_solenoid PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_solenoid PUBLIC
    bsp
    mspm0_build_options)
