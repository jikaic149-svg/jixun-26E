add_library(device_icm45686 STATIC
    "device_icm45686.c")

target_include_directories(device_icm45686 PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_icm45686 PUBLIC
    device_imu
    mspm0_build_options
    m)
