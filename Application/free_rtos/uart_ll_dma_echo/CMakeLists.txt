include_guard(DIRECTORY)

include(${CMAKE_HOME_DIRECTORY}/cmake/st_ll.cmake)
include(${CMAKE_HOME_DIRECTORY}/cmake/stm32f103cbtx.cmake)
include(${CMAKE_HOME_DIRECTORY}/cmake/gcc-arm.cmake)

set(EXECUTABLE FREERTOS_UART_LL_DMA_ECHO)

# List of source files to compile
set(APP_SOURCES
    # Put here your source files, one in each line, relative to CMakeLists.txt file location

    # application source files
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/main.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/gpio.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/freertos.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/stm32f1xx_it.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/syscalls.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/sysmem.c
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Src/system_stm32f1xx.c
    
    # freertos source files
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_1.c
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c

    # modules source files
    ${PROJECT_PATH}/Modules/buffer/ring_buffer.c

    # uart source files
    ${PROJECT_PATH}/Modules/serial/uart/ll_dma/uart_ll_dma.c

    # HAL libraries source files (from st_ll.cmake)
    ${HAL_DRIVERS_C_SOURCES}

    ${STARTUP_CODE_SRC}
)
    
    
# Include directories
set(APP_INCLUDE_PATHS
    # Put here your include dirs, one in each line, relative to CMakeLists.txt file location

    # application include paths
    ${PROJECT_PATH}/Application/free_rtos/uart_ll_dma_echo/Inc
    
    # freertos include paths
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${PROJECT_PATH}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3


    # modules include paths
    ${PROJECT_PATH}/Modules

    # HAL libraries include files (from st_hal.cmake)
    ${HAL_DRIVERS_C_INCLUDES}
)

# Symbols definition
set(APP_DEFINITIONS
    # Put here your symbols (preprocessor defines), one in each line
    # Encapsulate them with double quotes for safety purpose

    # HAL libraries compiler definitions (from st_hal.cmake)
    ${HAL_DRIVERS_C_DEFINITIONS}

    ${COMPILER_DEFINITIONS}
)


# Executable files
add_executable(${EXECUTABLE} ${APP_SOURCES})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${APP_INCLUDE_PATHS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${APP_DEFINITIONS})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${GCC_CPU_PARAMETERS}
    ${COMPILER_FLAGS}
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    ${LINKER_FLAGS}
    -Wl,-Map=${EXECUTABLE}.map,--cref
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Generate disassembly
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -x -s -S -d -e  $<TARGET_FILE:${EXECUTABLE}> > ${EXECUTABLE}.dump
)

# Generate hexdump
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${EXECUTABLE}> > ${EXECUTABLE}.dis
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
