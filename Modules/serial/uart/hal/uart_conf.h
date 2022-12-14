/******************************************************************************
 * @file        uart_conf.h
 * @brief       UART configurations used to initialize UART channels.
 * @details     
 *              
 * @version     1.0
 * @date        Apr 25, 2022
 * @copyright   
 *****************************************************************************/
#ifndef __UART_UART_CONF_H__
#define __UART_UART_CONF_H__


#include "board_config.h"

/* -------------------------------------------------------------------------- */

/**
 * @defgroup uart_configurations UART configurations
 * @{
 */

/**
 * @brief Switch to enable/disable handling data TX/RX inside interrupts
 * If enabled, UART_vidUpdateChannel() must be called periodically with a
 * high enough rate to handle data transmission/reception without any data loss
 * */
#ifdef CONF_UART_MINIMAL_INTERRUPTS
#define UART_MINIMAL_INTERRUPTS
#endif /*  CONF_UART_MINIMAL_INTERRUPTS  */


/**
 * @brief UART interrupts priorities
 * */
#ifdef CONF_UART_INTERRUPT_PREEMPTION_PRIORITY
#define UART_INTERRUPT_PREEMPTION_PRIORITY      CONF_UART_INTERRUPT_PREEMPTION_PRIORITY
#else
#define UART_INTERRUPT_PREEMPTION_PRIORITY      15
#endif /*  CONF_UART_INTERRUPT_PREEMPTION_PRIORITY  */

#ifdef CONF_UART_INTERRUPT_GROUPING_PRIORITY
#define UART_INTERRUPT_GROUPING_PRIORITY        CONF_UART_INTERRUPT_GROUPING_PRIORITY
#else
#define UART_INTERRUPT_GROUPING_PRIORITY        0
#endif /*  CONF_UART_INTERRUPT_GROUPING_PRIORITY  */


/* -------------------------------------------------------------------------- */
/* --------------------- Enable/Disable UART Channels ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * @brief 
 */
#ifdef CONF_UART_ENABLE_CHANNEL_1
#define UART_ENABLE_CHANNEL_1
#endif /*  CONF_UART_ENABLE_CHANNEL_1  */

#ifdef CONF_UART_ENABLE_CHANNEL_2
#define UART_ENABLE_CHANNEL_2
#endif /*  CONF_UART_ENABLE_CHANNEL_2  */

#ifdef CONF_UART_ENABLE_CHANNEL_3
#define UART_ENABLE_CHANNEL_3
#endif /*  CONF_UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */
/* ----------------------- Enable/Disable UART TX/RX ------------------------ */
/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_1

#ifdef CONF_UART_ENABLE_CHANNEL_1_TX
#define UART_ENABLE_CHANNEL_1_TX
#else
#warning UART channel 1 TX is in blocking mode
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_1_RX
#define UART_ENABLE_CHANNEL_1_RX
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  UART_ENABLE_CHANNEL_1  */


#ifdef UART_ENABLE_CHANNEL_2

#ifdef CONF_UART_ENABLE_CHANNEL_2_TX
#define UART_ENABLE_CHANNEL_2_TX
#else
#warning UART channel 2 TX is in blocking mode
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_2_RX
#define UART_ENABLE_CHANNEL_2_RX
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  UART_ENABLE_CHANNEL_2  */


#ifdef UART_ENABLE_CHANNEL_3

#ifdef CONF_UART_ENABLE_CHANNEL_3_TX
#define UART_ENABLE_CHANNEL_3_TX
#else
#warning UART channel 3 TX is in blocking mode
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef CONF_UART_ENABLE_CHANNEL_3_RX
#define UART_ENABLE_CHANNEL_3_RX
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */
/* ---------------------- UART TX/RX Buffers' sizes  ------------------------ */
/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_1

#ifdef UART_ENABLE_CHANNEL_1_TX

#ifdef CONF_UART_CHANNEL_1_TX_BUFFER_SIZE
#define UART_CHANNEL_1_TX_BUFFER_SIZE       CONF_UART_CHANNEL_1_TX_BUFFER_SIZE
#else
#define UART_CHANNEL_1_TX_BUFFER_SIZE       255
#endif /*  CONF_UART_CHANNEL_1_TX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef UART_ENABLE_CHANNEL_1_RX

#ifdef CONF_UART_CHANNEL_1_RX_BUFFER_SIZE
#define UART_CHANNEL_1_RX_BUFFER_SIZE       CONF_UART_CHANNEL_1_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_1_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_1_RX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  UART_ENABLE_CHANNEL_1  */

#ifdef UART_ENABLE_CHANNEL_2

#ifdef UART_ENABLE_CHANNEL_2_TX

#ifdef CONF_UART_CHANNEL_2_TX_BUFFER_SIZE
#define UART_CHANNEL_2_TX_BUFFER_SIZE       CONF_UART_CHANNEL_2_TX_BUFFER_SIZE
#else
#define UART_CHANNEL_2_TX_BUFFER_SIZE       255
#endif /*  CONF_UART_CHANNEL_2_TX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef UART_ENABLE_CHANNEL_2_RX

#ifdef CONF_UART_CHANNEL_2_RX_BUFFER_SIZE
#define UART_CHANNEL_2_RX_BUFFER_SIZE       CONF_UART_CHANNEL_2_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_2_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_2_RX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  UART_ENABLE_CHANNEL_2  */

#ifdef UART_ENABLE_CHANNEL_3

#ifdef UART_ENABLE_CHANNEL_3_TX

#ifdef CONF_UART_CHANNEL_3_TX_BUFFER_SIZE
#define UART_CHANNEL_3_TX_BUFFER_SIZE       CONF_UART_CHANNEL_3_TX_BUFFER_SIZE
#else
#define UART_CHANNEL_3_TX_BUFFER_SIZE       255
#endif /*  CONF_UART_CHANNEL_3_TX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef UART_ENABLE_CHANNEL_3_RX

#ifdef CONF_UART_CHANNEL_3_RX_BUFFER_SIZE
#define UART_CHANNEL_3_RX_BUFFER_SIZE       CONF_UART_CHANNEL_3_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_3_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_3_RX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  UART_ENABLE_CHANNEL_3  */

/* ------------------------------------------------------------------------- */
/* ------------------------ Configuration Checks --------------------------- */
/* ------------------------------------------------------------------------- */


#ifdef UART_ENABLE_CHANNEL_1

#ifdef UART_ENABLE_CHANNEL_1_TX
#if (UART_CHANNEL_1_TX_BUFFER_SIZE < 2)
#error Channel 1 TX buffer size must be > 1
#endif  /*  (UART_CHANNEL_1_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef UART_ENABLE_CHANNEL_1_RX
#if (UART_CHANNEL_1_RX_BUFFER_SIZE < 2)
#error Channel 1 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_1_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  UART_ENABLE_CHANNEL_1  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_2

#ifdef UART_ENABLE_CHANNEL_2_TX
#if (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)
#error Channel 2 TX buffer size must be > 1
#endif  /*  (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef UART_ENABLE_CHANNEL_2_RX
#if (UART_CHANNEL_2_RX_BUFFER_SIZE < 2)
#error Channel 2 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  UART_ENABLE_CHANNEL_2  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_3

#ifdef UART_ENABLE_CHANNEL_3_TX
#if (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)
#error Channel 3 TX buffer size must be > 1
#endif  /*  (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef UART_ENABLE_CHANNEL_3_RX
#if (UART_CHANNEL_3_RX_BUFFER_SIZE < 2)
#error Channel 3 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)  */

#endif  /*  UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */

/**@}*/

#endif /* __UART_UART_CONF_H__ */
