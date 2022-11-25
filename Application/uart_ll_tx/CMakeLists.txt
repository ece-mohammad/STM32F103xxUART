include_guard(DIRECTORY)

include(cmake/st_ll.cmake)
include(cmake/stm32f103cbtx.cmake)
include(cmake/gcc-arm.cmake)

set(EXECUTABLE UART_LL_TX)

#
# List of source files to compile
#
set(APP_SOURCES
    # Put here your source files, one in each line, relative to CMakeLists.txt file location

    # application source files
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/main.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/gpio.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/usart.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/stm32f1xx_it.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/syscalls.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/sysmem.c
    ${PROJECT_PATH}/Application/uart_ll_tx/Src/system_stm32f1xx.c
    
    # modules source files
    ${PROJECT_PATH}/Modules/buffer/ring_buffer.c
    
    # uart source files
    ${PROJECT_PATH}/Modules/serial/uart/ll/uart_ll.c

    # HAL libraries source files (from st_hal.cmake)
    ${HAL_DRIVERS_C_SOURCES}

    ${STARTUP_CODE_SRC}
)
    
    
    #
    # Include directories
    #
set(APP_INCLUDE_PATHS
    # Put here your include dirs, one in each line, relative to CMakeLists.txt file location

    # application include paths
    ${PROJECT_PATH}/Application/uart_ll_tx/Inc

    # configurations include paths
    ${PROJECT_PATH}/Config/ll
    
    # modules include paths
    ${PROJECT_PATH}/Modules

    # HAL libraries include files (from st_hal.cmake)
    ${HAL_DRIVERS_C_INCLUDES}
)

#
# Symbols definition
#
set(APP_DEFINITIONS
    # Put here your symbols (preprocessor defines), one in each line
    # Encapsulate them with double quotes for safety purpose

    # HAL libraries compiler definitions (from st_hal.cmake)
    ${HAL_DRIVERS_C_DEFINITIONS}

    ${COMPILER_DEFINITIONS}
)

# Executable files
add_executable(${EXECUTABLE} ${APP_SOURCES})

# add startup code
# target_sources(${EXECUTABLE} PRIVATE ${STARTUP_CODE_SRC})

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

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
