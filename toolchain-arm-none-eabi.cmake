# https://github.com/jobroe/cmake-arm-embedded/blob/master/toolchain-arm-none-eabi.cmake
#
# Author:  Johannes Bruder License: See LICENSE.TXT file included in the project
#
# CMake arm-none-eabi toolchain file
#

# Append current directory to CMAKE_MODULE_PATH for making device specific cmake
# modules visible
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# Target definition
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# ---------------------------------------------------------------------------------------
# Set toolchain paths
# ---------------------------------------------------------------------------------------
# Set system depended extensions
if (WIN32)
    set(TOOLCHAIN_EXT ".exe")
else ()
    set(TOOLCHAIN_EXT "")
endif ()
set(TOOLCHAIN arm-none-eabi)
set(C_COMPILER_NAME ${TOOLCHAIN}-gcc${TOOLCHAIN_EXT})
set(CXX_COMPILER_NAME ${TOOLCHAIN}-g++${TOOLCHAIN_EXT})
set(ASM_COMPILER_NAME ${TOOLCHAIN}-gcc${TOOLCHAIN_EXT})

option(TOOLCHAIN_PREFIX
        "Path to the toolchain, where `bin`, `lib` and `include` are sitting."
        "")
if (NOT TOOLCHAIN_PREFIX)
    find_program(C_COMPILER ${C_COMPILER_NAME})
    find_program(CXX_COMPILER ${CXX_COMPILER_NAME})
    find_program(ASM_COMPILER ${ASM_COMPILER_NAME})
    set(CMAKE_C_COMPILER
            ${C_COMPILER}
            CACHE INTERNAL "C Compiler")
    set(CMAKE_CXX_COMPILER
            ${CXX_COMPILER}
            CACHE INTERNAL "C++ Compiler")
    set(CMAKE_ASM_COMPILER
            ${ASM_COMPILER}
            CACHE INTERNAL "ASM Compiler")

    get_filename_component(TOOLCHAIN_BIN_DIR ${C_COMPILER} DIRECTORY)
    set(TOOLCHAIN_PREFIX ${TOOLCHAIN_BIN_DIR}/..)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PREFIX}/${TOOLCHAIN}
            ${CMAKE_PREFIX_PATH})
    message(STATUS "using automatic `TOOLCHAIN_PREFIX`: " ${TOOLCHAIN_PREFIX})
else ()
    message(STATUS "Using custom `TOOLCHAIN_PREFIX`: " ${TOOLCHAIN_PREFIX})
    if (NOT EXISTS ${TOOLCHAIN_PREFIX})
        message(FATAL_ERROR "TOOLCHAIN_PREFIX does not exist: " ${TOOLCHAIN_PREFIX})
    endif ()
    set(C_COMPILER ${TOOLCHAIN_PREFIX}/bin/${C_COMPILER_NAME})
    set(CXX_COMPILER ${TOOLCHAIN_PREFIX}/bin/${CXX_COMPILER_NAME})
    set(ASM_COMPILER ${TOOLCHAIN_PREFIX}/bin/${ASM_COMPILER_NAME})
    set(CMAKE_C_COMPILER
            ${C_COMPILER}
            CACHE INTERNAL "C Compiler")
    set(CMAKE_CXX_COMPILER
            ${CXX_COMPILER}
            CACHE INTERNAL "C++ Compiler")
    set(CMAKE_ASM_COMPILER
            ${ASM_COMPILER}
            CACHE INTERNAL "ASM Compiler")
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PREFIX}/${TOOLCHAIN}
            ${CMAKE_PREFIX_PATH})
endif ()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CPU_TYPE "cortex-m0plus")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# cmake-format: off
# ---------------------------------------------------------------------------------------
# Set compiler flags
# ---------------------------------------------------------------------------------------
# -mthumb               Generat thumb instructions.
# -fno-builtin          Do not use built-in functions provided by GCC.
# -Wall                 Print only standard warnings, for all use Wextra
# -ffunction-sections   Place each function item into its own section in the output file.
# -fdata-sections       Place each data item into its own section in the output file.
# -fomit-frame-pointer  Omit the frame pointer in functions that don’t need one.
# -mabi=aapcs           Defines enums to be a variable sized type.
#
# https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
# -mtune=cortex-m0 -mcpu=cortex-m0
#
# have to pass this to prevent the symbols from being removed
# https://stackoverflow.com/questions/39236917/using-gccs-link-time-optimization-with-static-linked-libraries
#
# cmake-format: on
set(OBJECT_GEN_FLAGS
        "-mthumb \
    -fno-builtin \
    -Wall \
    -Wimplicit-fallthrough\
    -ffunction-sections \
    -fdata-sections \
    -fomit-frame-pointer \
    -mabi=aapcs\
    -mtune=${CPU_TYPE} \
    -mcpu=${CPU_TYPE}")

set(CMAKE_C_FLAGS
        "${OBJECT_GEN_FLAGS}"
        CACHE INTERNAL "C Compiler options")
set(CMAKE_CXX_FLAGS
        "${OBJECT_GEN_FLAGS}"
        CACHE INTERNAL "C++ Compiler options")
set(CMAKE_ASM_FLAGS
        "${OBJECT_GEN_FLAGS} -x assembler-with-cpp"
        CACHE INTERNAL "ASM Compiler options")

# cmake-format: off
# ---------------------------------------------------------------------------------------
# Set linker flags
# ---------------------------------------------------------------------------------------
# -Wl,--gc-sections     Perform the dead code elimination.
# --specs=nano.specs    Link with newlib-nano.
# --specs=nosys.specs   No syscalls, provide empty implementations for the POSIX system calls.
# -n                    Turn off page alignment of sections, and disable linking against shared libraries.
#                       If the output format supports Unix style magic numbers, mark the output as "NMAGIC".
#
# one should specify the map file when configuring the executable target
# with following flags
#
# -Wl,-Map=\"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map\"
#
# cmake-format: on
set(CMAKE_EXE_LINKER_FLAGS
        "-Wl,--gc-sections \
    --specs=nosys.specs \
    --specs=nano.specs \
    -mabi=aapcs \
    -mcpu=${CPU_TYPE} \
    -mthumb \
    -n"
        CACHE INTERNAL "Linker options")

# cmake-format: off
# ---------------------------------------------------------------------------------------
# line time optimization (LTO) options
# ---------------------------------------------------------------------------------------
# -flto                 Perform link time optimization.
# -ffat-lto-objects     Generate fat LTO objects.
#
# @sa https://stackoverflow.com/questions/13799452/what-is-the-difference-in-gcc-between-lto-and-fat-lto-objects
# @sa https://llvm.org/docs/FatLTO.html
# cmake-format: on
option(USE_LTO "Use Link Time Optimization" ON)
if (USE_LTO)
    set(LTO "-flto -Wl,-flto -ffat-lto-objects")
else ()
    set(LTO "")
endif ()

# ---------------------------------------------------------------------------------------
# Set debug/release build configuration Options
# ---------------------------------------------------------------------------------------

set(CMAKE_C_FLAGS_DEBUG
        "-Og -g ${LTO}"
        CACHE INTERNAL "C Compiler options for debug build type")
set(CMAKE_CXX_FLAGS_DEBUG
        "-Og -g ${LTO}"
        CACHE INTERNAL "C++ Compiler options for debug build type")
set(CMAKE_ASM_FLAGS_DEBUG
        "-g"
        CACHE INTERNAL "ASM Compiler options for debug build type")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG
        "${LTO}"
        CACHE INTERNAL "Linker options for debug build type")

set(CMAKE_C_FLAGS_RELEASE
        "-Os -g ${LTO}"
        CACHE INTERNAL "C Compiler options for release build type")
set(CMAKE_CXX_FLAGS_RELEASE
        "-Os -g ${LTO}"
        CACHE INTERNAL "C++ Compiler options for release build type")
set(CMAKE_ASM_FLAGS_RELEASE
        "-g"
        CACHE INTERNAL "ASM Compiler options for release build type")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE
        "${LTO}"
        CACHE INTERNAL "Linker options for release build type")
