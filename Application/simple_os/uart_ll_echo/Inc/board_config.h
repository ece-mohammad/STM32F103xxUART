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
/* -------------------------------- SimpleOS -------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * @brief SimpleOS tick rate
 */
#define CONF_OS_TICK_RATE_HZ    1000

/**
 * @brief SimpleOS task count, maximum number of tasks 
 */
#define CONF_OS_TASK_COUNT      2

/* -------------------------------------------------------------------------- */
/* ---------------------------------- UART ---------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * Switch to enable/disable handling data TX/RX inside interrupts
 * If enabled, UART_vidUpdateChannel() must be called periodically with a
 * high enough rate to handle data transmission/reception without any data loss
 * */
#define CONF_UART_MINIMAL_INTERRUPTS

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
#define CONF_UART_CHANNEL_1_TX_BUFFER_SIZE       49
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_1_RX
#define CONF_UART_CHANNEL_1_RX_BUFFER_SIZE       256
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_1  */

#ifdef CONF_UART_ENABLE_CHANNEL_2

#ifdef CONF_UART_ENABLE_CHANNEL_2_TX
#define CONF_UART_CHANNEL_2_TX_BUFFER_SIZE       49
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_2_RX
#define CONF_UART_CHANNEL_2_RX_BUFFER_SIZE       256
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_2  */

#ifdef CONF_UART_ENABLE_CHANNEL_3

#ifdef CONF_UART_ENABLE_CHANNEL_3_TX
#define CONF_UART_CHANNEL_3_TX_BUFFER_SIZE       49
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_3_RX
#define CONF_UART_CHANNEL_3_RX_BUFFER_SIZE       256
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_3  */


/* -------------------------------------------------------------------------- */

/**
 * UART configurations
 * */
#define CONF_UART_BAUDRATE          4800
#define CONF_UART_DATASIZE          LL_USART_DATAWIDTH_8B
#define CONF_UART_PARITY            LL_USART_PARITY_NONE
#define CONF_UART_STOPBITS          LL_USART_STOPBITS_1
#define CONF_UART_DIRECTION         LL_USART_DIRECTION_TX_RX
#define CONF_UART_HWCONTROL         LL_USART_HWCONTROL_NONE
#define CONF_UART_OVERSAMPLING      LL_USART_OVERSAMPLING_16

#endif /* __BOARD_CONFIG_H__ */
