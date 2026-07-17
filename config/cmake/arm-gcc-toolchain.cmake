set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_GCC_ROOT "D:/GCC/ARMGCC/10 2021.10" CACHE PATH "GNU Arm Embedded Toolchain root")
file(TO_CMAKE_PATH "${ARM_GCC_ROOT}" ARM_GCC_ROOT)

set(_ARM_GCC_BIN "${ARM_GCC_ROOT}/bin")
set(CMAKE_C_COMPILER "${_ARM_GCC_BIN}/arm-none-eabi-gcc.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER "${_ARM_GCC_BIN}/arm-none-eabi-gcc.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_AR "${_ARM_GCC_BIN}/arm-none-eabi-ar.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB "${_ARM_GCC_BIN}/arm-none-eabi-ranlib.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJCOPY "${_ARM_GCC_BIN}/arm-none-eabi-objcopy.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJDUMP "${_ARM_GCC_BIN}/arm-none-eabi-objdump.exe" CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE "${_ARM_GCC_BIN}/arm-none-eabi-size.exe" CACHE FILEPATH "" FORCE)

foreach(_tool
        CMAKE_C_COMPILER
        CMAKE_ASM_COMPILER
        CMAKE_AR
        CMAKE_RANLIB
        CMAKE_OBJCOPY
        CMAKE_OBJDUMP
        CMAKE_SIZE)
    if(NOT EXISTS "${${_tool}}")
        message(FATAL_ERROR "Missing GNU Arm tool: ${${_tool}}")
    endif()
endforeach()

unset(_ARM_GCC_BIN)
