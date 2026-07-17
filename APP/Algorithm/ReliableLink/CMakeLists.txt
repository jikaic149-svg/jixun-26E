if(NOT TARGET algorithm_packet)
    message(FATAL_ERROR
        "APP/Algorithm/ReliableLink requires the algorithm_packet target")
endif()

add_library(algorithm_reliable_link STATIC
    "src/algorithm_reliable_link.c")

target_include_directories(algorithm_reliable_link PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(algorithm_reliable_link PUBLIC
    algorithm_packet)
