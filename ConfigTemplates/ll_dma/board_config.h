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
/* ---------------------------------- UART ---------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * Switch to enable/disable handling DMA TX/RX inside interrupts
 * If disabled, UART_vidUpdateChannel() must be called periodically with a
 * high enough rate to handle data transmission/reception without any data loss
 * */
#define UART_MINIMAL_INTERRUPTS

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
 * 
 * Calculating DMA RX buffer size
 * 
 * UART baudrate = 921600 bps
 * OR 921600 / 10 = 92160 Byte / sec
 * 
 * UART_enUpdateChannel() is called every 1 milli-second
 * 92160 / 1000 = 92.160 Bytes / milli-second 
 * OR ceil(92.16) = 92 Bytes / mili-second
 * 
 * DMA half transfer and DMA transfer complete interrupts
 * DMA RX buffer = (93 * 2) = 186 bytes
 * */
#ifdef CONF_UART_ENABLE_CHANNEL_1

#ifdef CONF_UART_ENABLE_CHANNEL_1_TX
#define CONF_UART_CHANNEL_1_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_1_TX_SIZE          186
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_1_RX
#define CONF_UART_CHANNEL_1_RX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_1_RX_BUFFER_SIZE   186
#endif  /*  CONF_UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_1  */

#ifdef CONF_UART_ENABLE_CHANNEL_2

#ifdef CONF_UART_ENABLE_CHANNEL_2_TX
#define CONF_UART_CHANNEL_2_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_2_TX_SIZE          186
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_2_RX
#define CONF_UART_CHANNEL_2_RX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_2_RX_BUFFER_SIZE   186
#endif  /*  CONF_UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_2  */

#ifdef CONF_UART_ENABLE_CHANNEL_3

#ifdef CONF_UART_ENABLE_CHANNEL_3_TX
#define CONF_UART_CHANNEL_3_TX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_3_TX_SIZE          186
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_3_RX
#define CONF_UART_CHANNEL_3_RX_BUFFER_SIZE       256
#define CONF_UART_DMA_CHANNEL_3_RX_BUFFER_SIZE   186
#endif  /*  CONF_UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  CONF_UART_ENABLE_CHANNEL_3  */


/* -------------------------------------------------------------------------- */

/**
 * Debug UART configurations
 * */
#define CONF_DEBUG_UART_CHANNEL         UART_CHANNEL_1
#define CONF_DEBUG_UART_BAUDRATE        115200
#define CONF_DEBUG_UART_PARITY          LL_USART_PARITY_NONE
#define CONF_DEBUG_UART_DATASIZE        LL_USART_DATAWIDTH_8B
#define CONF_DEBUG_UART_STOPBITS        LL_USART_STOPBITS_1


#endif /* __BOARD_CONFIG_H__ */
