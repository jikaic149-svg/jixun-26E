add_library(mode_car STATIC
    "src/mode_car_common.c"
    "src/mode_car_idle.c"
    "src/mode_car_straight.c"
    "src/mode_car_turning.c"
    "Spin/mode_car_spin.c"
    "src/mode_car_curve.c"
    "Tracking/mode_car_tracking.c"
    "Control/mode_car_speed_controller.c"
    "Control/mode_car_angle_controller.c"
    "src/mode_car_brake.c"
    "src/mode_car_done.c")

target_include_directories(mode_car PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(mode_car PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_link_libraries(mode_car PUBLIC
    algorithm_control
    mspm0_build_options)
