add_library(algorithm_spsc_ring STATIC
    "src/algorithm_spsc_ring.c")

target_include_directories(algorithm_spsc_ring PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_spsc_ring PUBLIC
    mspm0_build_options)
