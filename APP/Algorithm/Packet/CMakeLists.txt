add_library(algorithm_packet STATIC
    "src/algorithm_packet.c")

target_include_directories(algorithm_packet PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_packet PUBLIC
    mspm0_build_options)
