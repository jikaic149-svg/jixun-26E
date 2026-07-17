# G3507 uses its sole QEI timer for encoder 0 and TIMG7 capture plus a
# software direction decode for encoder 1.
set(BSP_SOURCES
    "${BSP_DIR}/bsp_init.c"
    "${CMAKE_SOURCE_DIR}/BSP/bsp_not_configured.c"
    "${BSP_DIR}/bsp_key.c"
    "${BSP_DIR}/bsp_encoder.c"
    "${BSP_DIR}/bsp_led.c"
    "${BSP_DIR}/bsp_rgb.c"
    "${BSP_DIR}/bsp_time.c")

set(BSP_DRIVERLIB_MODULES
    common
    gpio
    timer)
