cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH "../pico-sdk")
include(pico_sdk_import.cmake)
project(USBServoPlatform C CXX ASM)
pico_sdk_init()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(USBServoPlatformApp)