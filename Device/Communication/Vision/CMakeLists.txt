if(NOT TARGET algorithm_packet OR NOT TARGET algorithm_reliable_link)
    message(FATAL_ERROR
        "Device/Communication/Vision requires algorithm_packet and algorithm_reliable_link")
endif()

add_library(device_vision STATIC
    "device_vision.c")

target_include_directories(device_vision PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_vision PUBLIC
    algorithm_packet
    algorithm_reliable_link)
