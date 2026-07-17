add_library(algorithm_filter STATIC
    "src/algorithm_filter.c")

target_include_directories(algorithm_filter PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_filter PUBLIC
    mspm0_build_options
    m)
