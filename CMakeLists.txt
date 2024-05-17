cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE
        ${CMAKE_CURRENT_LIST_DIR}/toolchain-arm-none-eabi.cmake)
project(EVB1122)
enable_language(ASM)
enable_language(C)
enable_language(CXX)

set(CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(cmsis-core INTERFACE)
target_include_directories(cmsis-core INTERFACE CMSIS-Core/Include)

# f429
add_library(cmsis-f4 STATIC)
set(CMSIS_F4_DIR CMSIS/Drivers/CMSIS/Device/ST/STM32F4xx)
set(CMSIS_F4_GCC_DIR ${CMSIS_F4_DIR}/Source/Templates/gcc/)
target_sources(cmsis-f4 PRIVATE 
  ${CMSIS_F4_GCC_DIR}/startup_stm32f429xx.s
  ${CMSIS_F4_GCC_DIR}/vectors_stm32f429xx.c)
target_include_directories(cmsis-f4 PUBLIC ${CMSIS_F4_DIR}/Include)
target_link_libraries(cmsis-f4 PUBLIC cmsis-core)

# hal
add_library(hal STATIC)
file(GLOB HAL_SOURCES Driver/stm32f4xx/peripheral/src/*.c)
target_sources(hal PRIVATE ${HAL_SOURCES})
target_sources(hal PRIVATE Driver/stm32f4xx/cmsis/src/system_stm32f4xx.c)
target_include_directories(hal PUBLIC Driver/stm32f4xx/peripheral/inc)
target_link_libraries(hal PUBLIC cmsis-f4)

# use heap_4.c
add_library(freeRTOS STATIC)

