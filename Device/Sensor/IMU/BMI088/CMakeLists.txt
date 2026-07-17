add_library(device_bmi088 STATIC
    "device_bmi088.c")

target_include_directories(device_bmi088 PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_bmi088 PUBLIC
    mspm0_build_options
    m)
