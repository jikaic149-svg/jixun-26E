add_library(algorithm_crc STATIC
    "src/algorithm_crc.c")

target_include_directories(algorithm_crc PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_crc PUBLIC
    mspm0_build_options)
