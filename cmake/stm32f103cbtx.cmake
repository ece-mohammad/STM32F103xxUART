###################################################
# 
# This file defines linker script, startup code and 
# CPU compiler definitions for STM32F103CBTx
#  micro-controller
# 
###################################################

include_guard(DIRECTORY)

# Linker script
set(LINKER_SCRIPT_SRC               ${PROJECT_PATH}/Application/Common/Linker/STM32F103CBTx_FLASH.ld)

# Startup code
set(STARTUP_CODE_SRC                ${PROJECT_PATH}/Application/Common/Startup/startup_stm32f103cbtx.s)

# Core MCU flags, CPU, instruction set and FPU setup
set(GCC_CPU_PARAMETERS
    -mthumb
    -mcpu=cortex-m3
    -mfloat-abi=soft
)
