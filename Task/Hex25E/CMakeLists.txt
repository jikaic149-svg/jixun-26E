add_library(task_hex25e STATIC
    "hex25e_route.c"
    "task_hex25e_comm.c"
    "task_hex25e_vofa.c"
    "task_hex25e_imu.c"
    "task_hex25e_vision.c"
    "task_hex25e.c")

target_include_directories(task_hex25e PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/config")

target_link_libraries(task_hex25e PUBLIC
    device_serial
    device_vision
    device_jy61p
    device_motor
    device_led
    device_rgb
    device_servo
    device_encoder
    mode_car
    mspm0_build_options
    m)
