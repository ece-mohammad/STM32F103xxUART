cmake_minimum_required(VERSION 3.18)

# Setup compiler settings
set(CMAKE_C_STANDARD                99)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})

# set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
# set(CMAKE_VERBOSE_MAKEFILE TRUE)

include_d

#
# Core project settings
#
project(SimpleOS)
enable_language(C ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
#
set(CPU_PARAMETERS
    -mthumb

    # This needs attention to properly set for used MCU
    -mcpu=cortex-m3
    -mfloat-abi=soft
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32F103CBTx_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

#
# List of source files to compile
#
set(sources_SRCS
    # Put here your source files, one in each line, relative to CMakeLists.txt file location
    ${PROJ_PATH}/simple_os/simple_os.c
    ${PROJ_PATH}/Core/Src/main.c 
    ${PROJ_PATH}/Core/Src/gpio.c 
    ${PROJ_PATH}/Core/Src/stm32f1xx_it.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c 
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c 
    ${PROJ_PATH}/Core/Src/system_stm32f1xx.c
    ${PROJ_PATH}/startup_stm32f103xb.s
)


#
# Include directories
#
set(include_path_DIRS
    # Put here your include dirs, one in each line, relative to CMakeLists.txt file location
    ${PROJ_PATH}/Core/Inc
    ${PROJ_PATH}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${PROJ_PATH}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${PROJ_PATH}/Drivers/CMSIS/Include
    ${PROJ_PATH}/simple_os
    ${PROJ_PATH}/config
    ${PROJ_PATH}/Libraries
)

#
# Symbols definition
#
set(symbols_SYMB
    # Put here your symbols (preprocessor defines), one in each line
    # Encapsulate them with double quotes for safety purpose
    "USE_FULL_LL_DRIVER"
    "USE_FULL_ASSERT=1U"
    "HSE_VALUE=8000000"
    "HSE_STARTUP_TIMEOUT=100"
    "LSE_STARTUP_TIMEOUT=5000"
    "LSE_VALUE=32768"
    "HSI_VALUE=8000000"
    "LSI_VALUE=40000"
    "VDD_VALUE=3300"
    "PREFETCH_ENABLE=1"
    "STM32F103xB"
)

# Executable files
add_executable(${EXECUTABLE} ${sources_SRCS})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    --std=c99
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)

# static library build options
set(STATIC_LIBRARY                  lib${CMAKE_PROJECT_NAME})

set(sources_STATIC_LIBRARY
    ${PROJ_PATH}/simple_os/simple_os.c
)
