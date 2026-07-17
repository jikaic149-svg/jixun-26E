add_library(device_jy61p STATIC
    "device_jy61p.c")

target_include_directories(device_jy61p PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_jy61p PUBLIC
    mspm0_build_options)
