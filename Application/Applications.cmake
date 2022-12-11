include_guard(GLOBAL)

# path to applications directories, relative to Applications directory
set(APPLICATIONS 
    # interrupt based
    "interrupt/uart_hal_echo"
    "interrupt/uart_hal_tx"
    "interrupt/uart_ll_echo"
    "interrupt/uart_ll_tx"
    "interrupt/uart_ll_dma_echo"
    "interrupt/uart_ll_dma_tx"
    
    # simple os
    "simple_os/uart_hal_echo"
    "simple_os/uart_hal_tx"
    "simple_os/uart_ll_echo"
    "simple_os/uart_ll_tx"
    "simple_os/uart_ll_dma_echo"
    "simple_os/uart_ll_dma_tx"
    
    # free_rtos
    "free_rtos/uart_rtos_echo"

)
