set(PICO_BOARD pico2 CACHE STRING "Board type" FORCE)
set(PICO_PLATFORM rp2350 CACHE STRING "Platform type" FORCE)

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)

pico_sdk_init()

link_directories(libmicroros)
add_executable(pico_micro_ros_example
    pico_micro_ros_example.c
    pico_uart_transport.c
    aux/spi_dma.c
    aux/i2c_dma.c
    aux/pmw3901.c
    aux/mpu9250.c
    aux/sharpIR.c
    aux/hcsr04.c
)

target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_adc
    hardware_dma
)

target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
# UART1 (GPIO8/TX, GPIO9/RX) initialised directly by the transport layer.
# stdio not needed for micro-ROS; disable both to avoid pin conflicts.
pico_enable_stdio_usb(pico_micro_ros_example 0)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
