add_library(task_app STATIC
    "task_app.c")

target_include_directories(task_app PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/config")

target_link_libraries(task_app PUBLIC
    freertos_kernel
    mspm0_build_options)

if(TEMPLATE_TASK STREQUAL "hex25e")
    add_subdirectory(Hex25E)
    target_compile_definitions(task_app PUBLIC
        TEMPLATE_TASK_HEX25E=1)
    target_link_libraries(task_app PUBLIC
        task_hex25e
        device_key_input)
endif()
