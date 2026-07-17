add_library(algorithm_adrc STATIC
    "src/algorithm_adrc.c")

target_include_directories(algorithm_adrc PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_adrc PUBLIC
    mspm0_build_options)
