add_library(algorithm_lqr STATIC
    "src/algorithm_lqr.c")

target_include_directories(algorithm_lqr PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_lqr PUBLIC
    mspm0_build_options)
