add_library(algorithm_line_tracking STATIC
    "src/algorithm_line_tracking.c")

target_include_directories(algorithm_line_tracking PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_line_tracking PUBLIC
    mspm0_build_options
    m)
