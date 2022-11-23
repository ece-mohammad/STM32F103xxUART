###################################################
# 
# This file defines common ST HAL libraries source
#  files, include paths and compiler definitions
# 
###################################################

include_guard(DIRECTORY)

# HAL libraries source files
set(HAL_DRIVERS_C_SOURCES
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

# HAL libraries include files
set(HAL_DRIVERS_C_INCLUDES
    ${PROJECT_PATH}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${PROJECT_PATH}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${PROJECT_PATH}/Drivers/CMSIS/Include
)

# HAL libraries compiler definitions
set(HAL_DRIVERS_C_DEFINITIONS
    "USE_HAL_DRIVER"
)
