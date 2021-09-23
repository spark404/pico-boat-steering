cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico-boat-steering C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME pico-boat-steering)

pico_sdk_init()
add_executable(pico-boat-steering
    main.c
)

target_sources(${PROJECT_NAME} PRIVATE
        main.c
        )

target_link_libraries(${PROJECT_NAME} PRIVATE
        pico_stdlib
        hardware_pio
        hardware_pwm
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})

