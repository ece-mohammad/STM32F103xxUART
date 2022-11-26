include_guard(DIRECTORY)

# include HAL libraries, target mcu and compiler variables
include(cmake/st_ll.cmake)
include(cmake/stm32f103cbtx.cmake)
include(cmake/gcc-arm.cmake)

# application executable name
set(EXECUTABLE UART_LL_DMA_TX)

# List of source files to compile
set(APP_SOURCES

    # application source files
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/main.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/gpio.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/usart.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/stm32f1xx_it.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/syscalls.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/sysmem.c
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Src/system_stm32f1xx.c
    
    # modules source files
    ${PROJECT_PATH}/Modules/buffer/ring_buffer.c
    
    # uart source files
    ${PROJECT_PATH}/Modules/serial/uart/ll_dma/uart_ll_dma.c

    # HAL libraries source files (from st_hal.cmake)
    ${HAL_DRIVERS_C_SOURCES}
    ${HAL_DRIVERS_DMA_C_SOURCES}
    
    # Startup code
    ${STARTUP_CODE_SRC}
)
    
    
# Include directories
set(APP_INCLUDE_PATHS

    # application include paths
    ${PROJECT_PATH}/Application/uart_ll_dma_tx/Inc

    # modules include paths
    ${PROJECT_PATH}/Modules

    # HAL libraries include files (from st_ll.cmake)
    ${HAL_DRIVERS_C_INCLUDES}
)

# compiler (pre-processor) definition
set(APP_DEFINITIONS

    # HAL libraries compiler definitions (from st_ll.cmake)
    ${HAL_DRIVERS_C_DEFINITIONS}

    # common compiler definitions (from gcc-arm.cmake)
    ${COMPILER_DEFINITIONS}

    # app specific definitions
)

# Add executable file
add_executable(${EXECUTABLE} ${APP_SOURCES})

# define include paths for current executable
target_include_directories(${EXECUTABLE} PRIVATE ${APP_INCLUDE_PATHS})

# define compiler definitions for current executable
target_compile_definitions(${EXECUTABLE} PRIVATE ${APP_DEFINITIONS})

# set compiler options for current executable
target_compile_options(${EXECUTABLE} PRIVATE
${GCC_CPU_PARAMETERS}
${COMPILER_FLAGS}
)

# set linker options and flags for current executable
target_link_options(${EXECUTABLE} PRIVATE
    # 
    ${LINKER_FLAGS}

    -Wl,-Map=${EXECUTABLE}.map,--cref
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
