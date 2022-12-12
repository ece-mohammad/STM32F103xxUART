/******************************************************************************
 * @file      board_config.h
 * @brief
 * @version   1.0
 * @date      Apr 27, 2021
 * @copyright
 *****************************************************************************/
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/* -------------------------------------------------------------------------- */
/* -------------------------------- FreeRTOS -------------------------------- */
/* -------------------------------------------------------------------------- */

#define CONF_FREERTOS_CPU_CLOCK_HZ          64000000
#define CONF_FREERTOS_TICK_RATE_HZ          1000
#define CONF_FREERTOS_MAX_PRIORITIES        7
#define CONF_FREERTOS_MINIMAL_STACK_SIZE    32
#define CONF_FREERTOS_TOTAL_HEAP_SIZE       5120


/* -------------------------------------------------------------------------- */
/* ---------------------------------- UART ---------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * Use task notifications to signal RX/TX tasks, instead of using a dedicated
 * queue for TX/RX per channel. Which saves a lot of memory
 * (even though the queue holds only 3 bytes of data)
 * */
/* #define CONF_UART_RTOS_ENABLE_TASK_NOTIFICATIONS */

#define CONF_UART_RTOS_TX_TASK_STACK_WORDS          64
#define CONF_UART_RTOS_RX_TASK_STACK_WORDS          64
#define CONF_UART_RTOS_TX_MQUEUE_SIZE               6
#define CONF_UART_RTOS_RX_MQUEUE_SIZE               9

/**
 * UART interrupts priorities
 * */
#define CONF_UART_INTERRUPT_PREEMPTION_PRIORITY     15
#define CONF_UART_INTERRUPT_GROUPING_PRIORITY       0

/* -------------------------------------------------------------------------- */

/**
 * UART channels enable/disable
 * */
#define CONF_UART_ENABLE_CHANNEL_1
#define CONF_UART_ENABLE_CHANNEL_2 
#define CONF_UART_ENABLE_CHANNEL_3 

/* -------------------------------------------------------------------------- */

/**
 * UART channels enable/disable TX/RX
 * */
#ifdef CONF_UART_ENABLE_CHANNEL_1
#define CONF_UART_ENABLE_CHANNEL_1_TX
#define CONF_UART_ENABLE_CHANNEL_1_RX
#endif /*  CONF_UART_ENABLE_CHANNEL_1  */

#ifdef CONF_UART_ENABLE_CHANNEL_2
#define CONF_UART_ENABLE_CHANNEL_2_TX
#define CONF_UART_ENABLE_CHANNEL_2_RX
#endif /*  CONF_UART_ENABLE_CHANNEL_2  */

#ifdef CONF_UART_ENABLE_CHANNEL_3
#define CONF_UART_ENABLE_CHANNEL_3_TX
#define CONF_UART_ENABLE_CHANNEL_3_RX
#endif /*  CONF_UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */


/**
 * UART channels TX/RX buffer sizes, DMA TX size and RX buffer size
 * */
#ifdef CONF_UART_ENABLE_CHANNEL_1

#ifdef CONF_UART_ENABLE_CHANNEL_1_TX
#define CONF_UART_CHANNEL_1_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_1_TX_SIZE          188
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_1_RX
#define CONF_UART_CHANNEL_1_RX_BUFFER_SIZE       512
#define CONF_UART_DMA_CHANNEL_1_RX_BUFFER_SIZE   188
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_1  */

#ifdef CONF_UART_ENABLE_CHANNEL_2

#ifdef CONF_UART_ENABLE_CHANNEL_2_TX
#define CONF_UART_CHANNEL_2_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_2_TX_SIZE          188
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_2_RX
#define CONF_UART_CHANNEL_2_RX_BUFFER_SIZE       512
#define CONF_UART_DMA_CHANNEL_2_RX_BUFFER_SIZE   188
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_2  */

#ifdef CONF_UART_ENABLE_CHANNEL_3

#ifdef CONF_UART_ENABLE_CHANNEL_3_TX
#define CONF_UART_CHANNEL_3_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_3_TX_SIZE          188
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_3_RX
#define CONF_UART_CHANNEL_3_RX_BUFFER_SIZE       512
#define CONF_UART_DMA_CHANNEL_3_RX_BUFFER_SIZE   188
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_3  */



/* -------------------------------------------------------------------------- */

/**
 * UART configurations
 * */
#define CONF_UART_BAUDRATE          921600
#define CONF_UART_DATASIZE          LL_USART_DATAWIDTH_8B
#define CONF_UART_PARITY            LL_USART_PARITY_NONE
#define CONF_UART_STOPBITS          LL_USART_STOPBITS_1
#define CONF_UART_DIRECTION         LL_USART_DIRECTION_TX_RX
#define CONF_UART_HWCONTROL         LL_USART_HWCONTROL_NONE
#define CONF_UART_OVERSAMPLING      LL_USART_OVERSAMPLING_16

/* -------------------------------------------------------------------------- */

#endif /* __BOARD_CONFIG_H__ */
