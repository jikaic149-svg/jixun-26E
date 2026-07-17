add_library(algorithm_attitude STATIC
    "src/algorithm_attitude.c")

target_include_directories(algorithm_attitude PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_attitude PUBLIC
    mspm0_build_options
    m)
