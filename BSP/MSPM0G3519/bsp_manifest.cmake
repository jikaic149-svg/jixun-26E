# Board resources plus PA29/TIMG6 RGB and the 40 MHz HFXT clock tree.
set(BSP_SOURCES
    "${BSP_DIR}/bsp_init.c"
    "${CMAKE_SOURCE_DIR}/BSP/bsp_not_configured.c"
    "${BSP_DIR}/bsp_key.c"
    "${BSP_DIR}/bsp_encoder.c"
    "${BSP_DIR}/bsp_led.c"
    "${BSP_DIR}/bsp_motor.c"
    "${BSP_DIR}/bsp_pwm.c"
    "${BSP_DIR}/bsp_rgb.c"
    "${BSP_DIR}/bsp_servo.c"
    "${BSP_DIR}/bsp_solenoid.c"
    "${BSP_DIR}/bsp_time.c"
    "${BSP_DIR}/bsp_uart.c")

set(BSP_DRIVERLIB_MODULES
    common
    timer
    gpio
    uart
    dma
    factoryregion
    sysctl)
