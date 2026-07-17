add_library(mode_gimbal STATIC
    "src/mode_gimbal_foundation.c"
    "src/mode_gimbal_stabilization.c"
    "src/mode_gimbal_vision.c"
    "src/mode_gimbal_calibration.c"
    "src/mode_gimbal_scheduler.c")

target_include_directories(mode_gimbal PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(mode_gimbal PUBLIC
    mspm0_build_options)
