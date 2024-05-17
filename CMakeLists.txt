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

add_compile_definitions(
        STM32F429xx
        STM32_PLATFORM
        USE_HAL_DRIVER
        EVBKS5_E
        RUN_APP3
)

add_library(config INTERFACE)
target_include_directories(config INTERFACE Config)

add_library(cmsis-core INTERFACE)
target_include_directories(cmsis-core INTERFACE CMSIS-Core/Include)

# f429
add_library(cmsis-f4 STATIC)
set(CMSIS_F4_DIR CMSIS/Drivers/CMSIS/Device/ST/STM32F4xx)
set(CMSIS_F4_GCC_DIR ${CMSIS_F4_DIR}/Source/Templates/gcc/)
# target_include_directories(cmsis-f4 PUBLIC ${CMSIS_F4_DIR}/Include)
# target_include_directories(cmsis-f4 PUBLIC Driver/stm32f4xx/peripheral/inc)
target_sources(cmsis-f4 PRIVATE
        ${CMSIS_F4_GCC_DIR}/startup_stm32f429xx.s
)
target_include_directories(cmsis-f4 PUBLIC Driver/stm32f4xx/cmsis/inc)
target_include_directories(cmsis-f4 PUBLIC Driver/stm32f4xx/peripheral/inc)
target_sources(cmsis-f4 PRIVATE ${CMSIS_F4_DIR}/Source/Templates/system_stm32f4xx.c)
target_link_libraries(cmsis-f4 PUBLIC cmsis-core)

# hal
add_library(hal STATIC)
file(GLOB HAL_SOURCES Driver/stm32f4xx/peripheral/src/*.c)
target_sources(hal PRIVATE ${HAL_SOURCES})
target_include_directories(hal PUBLIC Driver/stm32f4xx/peripheral/inc)
target_link_libraries(hal PUBLIC cmsis-f4)

# use heap_4.c
add_library(freeRTOS STATIC)
file(GLOB FREERTOS_SOURCES Middleware/freertos/*.c)
target_sources(freeRTOS PRIVATE ${FREERTOS_SOURCES})
target_sources(freeRTOS PRIVATE Middleware/freertos/portable/MemMang/heap_4.c)
# https://github.com/FreeRTOS/FreeRTOS-Kernel/blob/main/portable/GCC/ARM_CM4F/portmacro.h
target_sources(freeRTOS PRIVATE Middleware/freertos/portable/GCC/ARM_CM4F/port.c)
target_include_directories(freeRTOS PUBLIC Middleware/freertos/include)
target_include_directories(freeRTOS PUBLIC Middleware/freertos/portable/GCC/ARM_CM4F)
target_link_libraries(freeRTOS PUBLIC hal config)

add_library(cmsis-rtos STATIC)
target_sources(cmsis-rtos PRIVATE Middleware/freertos/CMSIS_RTOS/cmsis_os.c)
target_include_directories(cmsis-rtos PUBLIC Middleware/freertos/CMSIS_RTOS)
target_link_libraries(cmsis-rtos PUBLIC freeRTOS)

# app
set(APP_TARGET ${PROJECT_NAME}.elf)
add_executable(${APP_TARGET})
file(GLOB APP_SOURCES App/common/src/*.c)
target_sources(${APP_TARGET} PRIVATE ${APP_SOURCES})
target_include_directories(${APP_TARGET} PUBLIC App/common/inc)
# common/banyan
file(GLOB BANYAN_SOURCES Middleware/common/src/*.c)
target_sources(${APP_TARGET} PRIVATE ${BANYAN_SOURCES})
target_include_directories(${APP_TARGET} PUBLIC Middleware/common/inc)
# usb core
file(GLOB USB_CORE_SOURCES Middleware/usb/device/Core/Src/*.c)
target_sources(${APP_TARGET} PRIVATE ${USB_CORE_SOURCES})
target_include_directories(${APP_TARGET} PUBLIC Middleware/usb/device/Core/Inc)
# usb class
file(GLOB USB_CLASS_SOURCES Middleware/usb/device/Class/CDC/Src/*.c)
target_sources(${APP_TARGET} PRIVATE ${USB_CLASS_SOURCES})
target_include_directories(${APP_TARGET} PUBLIC Middleware/usb/device/Class/CDC/Inc)
# platform
file(GLOB PLATFORM_SOURCES Middleware/platform/stm32/src/*.c)
target_sources(${APP_TARGET} PRIVATE ${PLATFORM_SOURCES})
target_include_directories(${APP_TARGET} PUBLIC Middleware/platform/stm32/inc)
target_link_libraries(${APP_TARGET} PRIVATE hal cmsis-rtos freeRTOS)

set(LD_SCRIPT_PATH ldscript.ld)
get_filename_component(LD_SCRIPT_PATH_ABSOLUTE ${LD_SCRIPT_PATH} ABSOLUTE)
set_target_properties(${APP_TARGET} PROPERTIES
        LINK_DEPENDS ${LD_SCRIPT_PATH_ABSOLUTE}
        LINK_FLAGS "-T ${LD_SCRIPT_PATH_ABSOLUTE} \
                    -Wl,-Map=${PROJECT_NAME}.map "
)
