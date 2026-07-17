cmake_minimum_required(VERSION 3.25)

set(MSPM0_DEVICE "MSPM0G3519" CACHE STRING "Target device")
set_property(CACHE MSPM0_DEVICE PROPERTY STRINGS MSPM0G3507 MSPM0G3519)

set(TEMPLATE_TASK "minimal" CACHE STRING "Template task")
set_property(CACHE TEMPLATE_TASK PROPERTY STRINGS
    minimal hex25e)

option(TEMPLATE_FREERTOS_USE_HEAP_4
    "Enable FreeRTOS dynamic allocation through heap_4 for this firmware"
    OFF)
option(TEMPLATE_FREERTOS_USE_TIMERS
    "Compile FreeRTOS software timers for this firmware"
    OFF)
option(TEMPLATE_FREERTOS_USE_EVENT_GROUPS
    "Compile FreeRTOS event groups for this firmware"
    OFF)
option(TEMPLATE_FREERTOS_USE_STREAM_BUFFERS
    "Compile FreeRTOS stream/message buffers for this firmware"
    OFF)
option(TEMPLATE_FREERTOS_USE_TRACE
    "Enable the FreeRTOS trace facility for this firmware"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_SPSC_RING
    "Compile the RTOS-independent SPSC byte ring component"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_PACKET
    "Compile the ASCII CRC16 LF packet codec"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_RELIABLE_LINK
    "Compile ACK, timeout, retry, and duplicate suppression support"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_CONTROL
    "Compile hardware-independent control primitives"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_FILTER
    "Compile reusable low-pass, moving-average, and median filters"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_ATTITUDE
    "Compile the hardware-independent 6-axis attitude estimator"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_LINE_TRACKING
    "Compile the difference-over-sum line-position estimator"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_LQR
    "Compile fixed, scheduled, and incremental LQR components"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_ADRC
    "Compile the first-order ESO and PI-ESO components"
    OFF)
option(TEMPLATE_ENABLE_ALGORITHM_CRC
    "Compile the reentrant CRC-16/IBM and Modbus component"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_KEY
    "Compile the Device key gesture state machine"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_MOTOR
    "Compile the signed-command motor Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_LED
    "Compile the discrete indicator LED Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_RGB
    "Compile the addressable RGB indicator Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_SERVO
    "Compile the PWM hobby-servo Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_SOLENOID
    "Compile the active-high solenoid Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_QD4310
    "Compile the QD4310 CAN motor Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_ENCODER
    "Compile the encoder distance/speed Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_LINE_TRACKING
    "Compile the configurable multi-channel line-tracking Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_JY61P
    "Compile the JY61P byte-stream Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_BMI088
    "Compile the BMI088 register-level Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_ICM45686
    "Compile the ICM-45686 register-level Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_VISION
    "Compile the transport-independent vision protocol Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_IMU
    "Compile the mapped, calibrated IMU Device"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_CHASSIS_COMMAND
    "Compile the typed jixun3 chassis serial command parser"
    OFF)
option(TEMPLATE_ENABLE_DEVICE_SERIAL
    "Compile the generic buffered serial Device"
    OFF)
# Resolve only the transitive modules that an explicitly selected feature
# needs. These are ordinary variables rather than forced cache values, so
# disabling the parent feature on the next configure does not leave hidden
# modules enabled.
set(TEMPLATE_BUILD_ALGORITHM_SPSC_RING
    "${TEMPLATE_ENABLE_ALGORITHM_SPSC_RING}")
set(TEMPLATE_BUILD_ALGORITHM_PACKET
    "${TEMPLATE_ENABLE_ALGORITHM_PACKET}")
set(TEMPLATE_BUILD_ALGORITHM_RELIABLE_LINK
    "${TEMPLATE_ENABLE_ALGORITHM_RELIABLE_LINK}")
set(TEMPLATE_BUILD_ALGORITHM_CONTROL
    "${TEMPLATE_ENABLE_ALGORITHM_CONTROL}")
set(TEMPLATE_BUILD_ALGORITHM_FILTER
    "${TEMPLATE_ENABLE_ALGORITHM_FILTER}")
set(TEMPLATE_BUILD_ALGORITHM_ATTITUDE
    "${TEMPLATE_ENABLE_ALGORITHM_ATTITUDE}")
set(TEMPLATE_BUILD_ALGORITHM_LINE_TRACKING
    "${TEMPLATE_ENABLE_ALGORITHM_LINE_TRACKING}")
set(TEMPLATE_BUILD_ALGORITHM_LQR
    "${TEMPLATE_ENABLE_ALGORITHM_LQR}")
set(TEMPLATE_BUILD_ALGORITHM_ADRC
    "${TEMPLATE_ENABLE_ALGORITHM_ADRC}")
set(TEMPLATE_BUILD_ALGORITHM_CRC
    "${TEMPLATE_ENABLE_ALGORITHM_CRC}")
set(TEMPLATE_BUILD_DEVICE_IMU
    "${TEMPLATE_ENABLE_DEVICE_IMU}")
set(TEMPLATE_BUILD_DEVICE_QD4310
    "${TEMPLATE_ENABLE_DEVICE_QD4310}")
set(TEMPLATE_BUILD_DEVICE_LINE_TRACKING
    "${TEMPLATE_ENABLE_DEVICE_LINE_TRACKING}")
if(TEMPLATE_ENABLE_DEVICE_VISION)
    set(TEMPLATE_BUILD_ALGORITHM_PACKET ON)
    set(TEMPLATE_BUILD_ALGORITHM_RELIABLE_LINK ON)
endif()
if(TEMPLATE_BUILD_ALGORITHM_RELIABLE_LINK)
    set(TEMPLATE_BUILD_ALGORITHM_PACKET ON)
endif()
if(TEMPLATE_BUILD_DEVICE_IMU)
    set(TEMPLATE_BUILD_ALGORITHM_ATTITUDE ON)
endif()
if(TEMPLATE_TASK STREQUAL "hex25e")
    set(TEMPLATE_BUILD_ALGORITHM_CONTROL ON)
    set(TEMPLATE_BUILD_ALGORITHM_PACKET ON)
    set(TEMPLATE_BUILD_ALGORITHM_RELIABLE_LINK ON)
    set(TEMPLATE_ENABLE_DEVICE_KEY ON)
    set(TEMPLATE_ENABLE_DEVICE_MOTOR ON)
    set(TEMPLATE_ENABLE_DEVICE_LED ON)
    set(TEMPLATE_ENABLE_DEVICE_RGB ON)
    set(TEMPLATE_ENABLE_DEVICE_SERVO ON)
    set(TEMPLATE_ENABLE_DEVICE_ENCODER ON)
    set(TEMPLATE_ENABLE_DEVICE_JY61P ON)
    set(TEMPLATE_ENABLE_DEVICE_SERIAL ON)
    set(TEMPLATE_ENABLE_DEVICE_VISION ON)
endif()
if(TEMPLATE_BUILD_DEVICE_LINE_TRACKING)
    set(TEMPLATE_BUILD_ALGORITHM_LINE_TRACKING ON)
endif()

project(TI_MSPM0_TEMPLATE LANGUAGES C ASM)

if(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release)$")
    message(FATAL_ERROR
        "CMAKE_BUILD_TYPE must be Debug or Release, got '${CMAKE_BUILD_TYPE}'")
endif()
if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR
        "This template requires GNU Arm Embedded GCC")
endif()
get_filename_component(C_COMPILER_NAME "${CMAKE_C_COMPILER}" NAME)
if(NOT C_COMPILER_NAME STREQUAL "arm-none-eabi-gcc.exe"
        AND NOT C_COMPILER_NAME STREQUAL "arm-none-eabi-gcc")
    message(FATAL_ERROR
        "Unexpected C compiler: ${CMAKE_C_COMPILER}")
endif()

get_filename_component(TI_MSPM0_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
set(DEFAULT_MSPM0_SDK_ROOT
    "${TI_MSPM0_ROOT}/TI ENV/mspm0_sdk_2_10_00_04")
set(DEFAULT_SYSCONFIG_ROOT
    "${TI_MSPM0_ROOT}/TI ENV/sysconfig_1.27.1")
set(MSPM0_SDK_ROOT
    "${DEFAULT_MSPM0_SDK_ROOT}"
    CACHE PATH "TI MSPM0 SDK root")
set(SYSCONFIG_ROOT
    "${DEFAULT_SYSCONFIG_ROOT}"
    CACHE PATH "TI SysConfig root")
file(TO_CMAKE_PATH "${MSPM0_SDK_ROOT}" MSPM0_SDK_ROOT)
file(TO_CMAKE_PATH "${SYSCONFIG_ROOT}" SYSCONFIG_ROOT)

set(SDK_SOURCE "${MSPM0_SDK_ROOT}/source")
set(SYSCONFIG_NODE "${SYSCONFIG_ROOT}/nodejs/node.exe")
set(SYSCONFIG_CLI "${SYSCONFIG_ROOT}/dist/cli.js")
set(SYSCONFIG_PRODUCT "${MSPM0_SDK_ROOT}/.metadata/product.json")

if(MSPM0_DEVICE STREQUAL "MSPM0G3507")
    set(MSPM0_PACKAGE "LQFP-64(PM)")
    set(MSPM0_DEFINE "__MSPM0G3507__")
    set(MSPM0_STARTUP
        "${SDK_SOURCE}/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c")
    set(MSPM0_LINKER
        "${SDK_SOURCE}/ti/devices/msp/m0p/linker_files/gcc/mspm0g3507.lds")
    set(BSP_DIR "${CMAKE_SOURCE_DIR}/BSP/MSPM0G3507")
elseif(MSPM0_DEVICE STREQUAL "MSPM0G3519")
    set(MSPM0_PACKAGE "LQFP-80(PN)")
    set(MSPM0_DEFINE "__MSPM0G3519__")
    set(MSPM0_STARTUP
        "${SDK_SOURCE}/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g351x_gcc.c")
    set(MSPM0_LINKER
        "${SDK_SOURCE}/ti/devices/msp/m0p/linker_files/gcc/mspm0g3519.lds")
    set(BSP_DIR "${CMAKE_SOURCE_DIR}/BSP/MSPM0G3519")
else()
    message(FATAL_ERROR "Unsupported MSPM0_DEVICE='${MSPM0_DEVICE}'")
endif()

if(NOT TEMPLATE_TASK STREQUAL "minimal"
        AND NOT TEMPLATE_TASK STREQUAL "hex25e")
    message(FATAL_ERROR "Unsupported TEMPLATE_TASK='${TEMPLATE_TASK}'")
endif()

foreach(required_file
        "${SDK_SOURCE}/ti/driverlib/driverlib.h"
        "${SYSCONFIG_NODE}"
        "${SYSCONFIG_CLI}"
        "${SYSCONFIG_PRODUCT}"
        "${MSPM0_STARTUP}"
        "${MSPM0_LINKER}"
        "${BSP_DIR}/${MSPM0_DEVICE}.syscfg"
        "${BSP_DIR}/bsp_resource_map.h"
        "${BSP_DIR}/bsp_init.c"
        "${CMAKE_SOURCE_DIR}/BSP/bsp_not_configured.c")
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR "Required file is missing: ${required_file}")
    endif()
endforeach()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_library(mspm0_build_options INTERFACE)
target_compile_definitions(mspm0_build_options INTERFACE
    "${MSPM0_DEFINE}")
target_compile_options(mspm0_build_options INTERFACE
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mtune=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wpedantic
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-Os>)

# Keep the manifest explicit. A new feature adds only its DriverLib module;
# no source glob is allowed.
include("${CMAKE_SOURCE_DIR}/config/cmake/DriverLibModules.cmake")
include("${BSP_DIR}/bsp_manifest.cmake")
foreach(bsp_source IN LISTS BSP_SOURCES)
    if(NOT EXISTS "${bsp_source}")
        message(FATAL_ERROR "BSP source is missing: ${bsp_source}")
    endif()
endforeach()
add_library(ti_driverlib STATIC)
target_include_directories(ti_driverlib SYSTEM PUBLIC
    "${SDK_SOURCE}"
    "${SDK_SOURCE}/third_party/CMSIS/Core/Include")
target_link_libraries(ti_driverlib PUBLIC mspm0_build_options)
target_compile_options(ti_driverlib PRIVATE
    -Wno-pedantic
    -Wno-unused-parameter
    -Wno-maybe-uninitialized)
mspm0_add_driverlib_modules(ti_driverlib ${BSP_DRIVERLIB_MODULES})

set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
set(GENERATED_CONFIG_C "${GENERATED_DIR}/ti_msp_dl_config.c")
set(GENERATED_CONFIG_H "${GENERATED_DIR}/ti_msp_dl_config.h")
set(SYSCONFIG_SCRIPT "${BSP_DIR}/${MSPM0_DEVICE}.syscfg")

add_custom_command(
    OUTPUT "${GENERATED_CONFIG_C}" "${GENERATED_CONFIG_H}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_DIR}"
    COMMAND "${SYSCONFIG_NODE}" "${SYSCONFIG_CLI}"
            --product "${SYSCONFIG_PRODUCT}"
            --compiler gcc
            --device "${MSPM0_DEVICE}"
            --package "${MSPM0_PACKAGE}"
            --script "${SYSCONFIG_SCRIPT}"
            --output "${GENERATED_DIR}"
    DEPENDS
        "${SYSCONFIG_SCRIPT}"
        "${SYSCONFIG_PRODUCT}"
        "${SYSCONFIG_CLI}"
    COMMENT "Generate ${MSPM0_DEVICE} BSP with TI SysConfig"
    VERBATIM)
set_source_files_properties(
    "${GENERATED_CONFIG_C}" "${GENERATED_CONFIG_H}"
    PROPERTIES GENERATED TRUE)
set_source_files_properties("${GENERATED_CONFIG_C}"
    PROPERTIES COMPILE_OPTIONS "-Wno-pedantic")
add_custom_target(generate_bsp_config
    DEPENDS "${GENERATED_CONFIG_C}" "${GENERATED_CONFIG_H}")

add_library(bsp STATIC
    ${BSP_SOURCES}
    "${GENERATED_CONFIG_C}"
    "${GENERATED_CONFIG_H}")
target_include_directories(bsp PUBLIC
    "${CMAKE_SOURCE_DIR}/BSP")
target_include_directories(bsp PRIVATE
    "${BSP_DIR}")
target_include_directories(bsp SYSTEM PRIVATE
    "${GENERATED_DIR}")
target_link_libraries(bsp PUBLIC mspm0_build_options)
target_link_libraries(bsp PRIVATE ti_driverlib)
add_dependencies(bsp generate_bsp_config)

add_subdirectory(APP/Algorithm)
add_subdirectory(Device)

if(TEMPLATE_TASK STREQUAL "hex25e")
    add_subdirectory(Mode/Car)
endif()

add_subdirectory(FreeRTOS)
add_subdirectory(Task)

set(FIRMWARE_TARGET "${MSPM0_DEVICE}.elf")
add_executable("${FIRMWARE_TARGET}"
    "${CMAKE_SOURCE_DIR}/main.c"
    "${MSPM0_STARTUP}")
set_source_files_properties("${MSPM0_STARTUP}"
    PROPERTIES COMPILE_OPTIONS "-Wno-pedantic")
target_include_directories("${FIRMWARE_TARGET}" PRIVATE
    "${CMAKE_SOURCE_DIR}"
    "${GENERATED_DIR}")
target_link_libraries("${FIRMWARE_TARGET}" PRIVATE
    task_app
    freertos_kernel
    bsp
    mspm0_build_options
    ti_driverlib
    c
    m)

set(MAP_FILE "${CMAKE_BINARY_DIR}/${MSPM0_DEVICE}.map")
target_link_options("${FIRMWARE_TARGET}" PRIVATE
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    "-Wl,-Map=${MAP_FILE}"
    "-T${MSPM0_LINKER}")

set(HEX_FILE "${CMAKE_BINARY_DIR}/${MSPM0_DEVICE}.hex")
set(BIN_FILE "${CMAKE_BINARY_DIR}/${MSPM0_DEVICE}.bin")
set(MAIN_FLASH_SECTIONS
    --only-section=.intvecs
    --only-section=.text
    --only-section=.ramfunc
    --only-section=.rodata
    --only-section=.preinit_array
    --only-section=.init_array
    --only-section=.fini_array
    --only-section=.ARM.exidx
    --only-section=.ARM.extab
    --only-section=.data)
add_custom_command(TARGET "${FIRMWARE_TARGET}" POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" ${MAIN_FLASH_SECTIONS}
            -O ihex "$<TARGET_FILE:${FIRMWARE_TARGET}>" "${HEX_FILE}"
    COMMAND "${CMAKE_OBJCOPY}" ${MAIN_FLASH_SECTIONS}
            --gap-fill=0xFF -O binary
            "$<TARGET_FILE:${FIRMWARE_TARGET}>" "${BIN_FILE}"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:${FIRMWARE_TARGET}>"
    COMMENT "Create main-Flash-only ${MSPM0_DEVICE}.hex and .bin"
    VERBATIM)

message(STATUS "MSPM0 device     : ${MSPM0_DEVICE}")
message(STATUS "Template task : ${TEMPLATE_TASK}")
message(STATUS "BSP              : ${BSP_DIR}")
get_target_property(DRIVERLIB_SOURCES ti_driverlib SOURCES)
message(STATUS "BSP sources      : ${BSP_SOURCES}")
message(STATUS "DriverLib sources: ${DRIVERLIB_SOURCES}")
message(STATUS "FreeRTOS tasks   : ON (static allocation)")
