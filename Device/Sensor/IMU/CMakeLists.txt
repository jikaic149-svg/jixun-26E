if(NOT TARGET algorithm_attitude)
    message(FATAL_ERROR "Device/Sensor/IMU requires algorithm_attitude")
endif()

add_library(device_imu STATIC
    "device_imu.c")

target_include_directories(device_imu PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_imu PUBLIC
    algorithm_attitude
    mspm0_build_options)
