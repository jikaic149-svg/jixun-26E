add_library(device_qd4310 STATIC
    "device_qd4310.c")

target_include_directories(device_qd4310 PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_qd4310 PUBLIC
    bsp
    mspm0_build_options
    m)
