set(FREERTOS_KERNEL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Kernel")

set(FREERTOS_KERNEL_SOURCES
    "${FREERTOS_KERNEL_ROOT}/tasks.c"
    "${FREERTOS_KERNEL_ROOT}/queue.c"
    "${FREERTOS_KERNEL_ROOT}/list.c"
    "${FREERTOS_KERNEL_ROOT}/portable/GCC/ARM_CM0/port.c"
    "${FREERTOS_KERNEL_ROOT}/portable/GCC/ARM_CM0/portasm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Config/freertos_support.c")

if(TEMPLATE_FREERTOS_USE_TIMERS)
    list(APPEND FREERTOS_KERNEL_SOURCES
        "${FREERTOS_KERNEL_ROOT}/timers.c")
endif()
if(TEMPLATE_FREERTOS_USE_EVENT_GROUPS)
    list(APPEND FREERTOS_KERNEL_SOURCES
        "${FREERTOS_KERNEL_ROOT}/event_groups.c")
endif()
if(TEMPLATE_FREERTOS_USE_STREAM_BUFFERS)
    list(APPEND FREERTOS_KERNEL_SOURCES
        "${FREERTOS_KERNEL_ROOT}/stream_buffer.c")
endif()
if(TEMPLATE_FREERTOS_USE_HEAP_4)
    list(APPEND FREERTOS_KERNEL_SOURCES
        "${FREERTOS_KERNEL_ROOT}/portable/MemMang/heap_4.c")
endif()

add_library(freertos_kernel STATIC ${FREERTOS_KERNEL_SOURCES})

target_include_directories(freertos_kernel PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Config"
    "${FREERTOS_KERNEL_ROOT}/include"
    "${FREERTOS_KERNEL_ROOT}/portable/GCC/ARM_CM0"
    "${GENERATED_DIR}"
    "${SDK_SOURCE}"
    "${SDK_SOURCE}/third_party/CMSIS/Core/Include")
target_link_libraries(freertos_kernel PUBLIC
    mspm0_build_options
    ti_driverlib)
target_compile_definitions(freertos_kernel PUBLIC
    TEMPLATE_FREERTOS_USE_HEAP_4=$<BOOL:${TEMPLATE_FREERTOS_USE_HEAP_4}>
    TEMPLATE_FREERTOS_USE_TIMERS=$<BOOL:${TEMPLATE_FREERTOS_USE_TIMERS}>
    TEMPLATE_FREERTOS_USE_EVENT_GROUPS=$<BOOL:${TEMPLATE_FREERTOS_USE_EVENT_GROUPS}>
    TEMPLATE_FREERTOS_USE_STREAM_BUFFERS=$<BOOL:${TEMPLATE_FREERTOS_USE_STREAM_BUFFERS}>
    TEMPLATE_FREERTOS_USE_TRACE=$<BOOL:${TEMPLATE_FREERTOS_USE_TRACE}>)
target_compile_options(freertos_kernel PRIVATE
    -Wno-pedantic
    -Wno-unused-parameter)
add_dependencies(freertos_kernel generate_bsp_config)

message(STATUS "FreeRTOS heap_4       : ${TEMPLATE_FREERTOS_USE_HEAP_4}")
message(STATUS "FreeRTOS timers       : ${TEMPLATE_FREERTOS_USE_TIMERS}")
message(STATUS "FreeRTOS event groups : ${TEMPLATE_FREERTOS_USE_EVENT_GROUPS}")
message(STATUS "FreeRTOS stream buffer: ${TEMPLATE_FREERTOS_USE_STREAM_BUFFERS}")
message(STATUS "FreeRTOS trace        : ${TEMPLATE_FREERTOS_USE_TRACE}")
