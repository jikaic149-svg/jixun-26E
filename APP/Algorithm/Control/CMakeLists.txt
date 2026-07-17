add_library(algorithm_control STATIC
    "src/algorithm_control.c")

target_include_directories(algorithm_control PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_control PUBLIC
    mspm0_build_options
    m)
