cmake_minimum_required(VERSION 3.18)

# C99 standard
set(CMAKE_C_STANDARD                99)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)

set(PROJECT_PATH                    ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})

project(STM32F103X_Serial)
enable_language(C ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
#
set(CPU_PARAMETERS
    -mthumb
    -mcpu=cortex-m3
    -mfloat-abi=soft
)

# Set linker script
set(linker_script_SRC               ${PROJECT_PATH}/Application/Common/Linker/STM32F103CBTx_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})