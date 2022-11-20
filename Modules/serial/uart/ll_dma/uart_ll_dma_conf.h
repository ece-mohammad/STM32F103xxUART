/******************************************************************************
 * @file      uart_ll_dma_conf.h
 * @brief
 * @version   1.0
 * @date      Apr 25, 2022
 * @copyright
 *****************************************************************************/
#ifndef __UART_LL_DMA_CONF_H__
#define __UART_LL_DMA_CONF_H__

/* -------------------------------------------------------------------------- */

#include "board_config.h"

/* -------------------------------------------------------------------------- */

/**
 * Switch to enable/disable handling DMA TX/RX inside interrupts
 * If disabled, UART_vidUpdateChannel() must be called periodically with a
 * high enough rate to handle data transmission/reception without any data loss
 * */
#ifdef CONF_UART_DMA_MINIMAL_INTERRUPTS
#define UART_DMA_MINIMAL_INTERRUPTS
#endif /*  CONF_UART_DMA_MINIMAL_INTERRUPTS  */


/**
 * UART interrupts priorities
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


/* -------------------------------------------------------------------------- */
/* --------------------- Enable/Disable UART Channels ----------------------- */
/* -------------------------------------------------------------------------- */

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

#ifdef CONF_UART_DMA_CHANNEL_1_TX_SIZE
#define UART_DMA_CHANNEL_1_TX_SIZE          CONF_UART_DMA_CHANNEL_1_TX_SIZE
#else
#define UART_DMA_CHANNEL_1_TX_SIZE          20
#endif /*  CONF_UART_DMA_CHANNEL_1_TX_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef UART_ENABLE_CHANNEL_1_RX

#ifdef CONF_UART_CHANNEL_1_RX_BUFFER_SIZE
#define UART_CHANNEL_1_RX_BUFFER_SIZE       CONF_UART_CHANNEL_1_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_1_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_1_RX_BUFFER_SIZE  */

#ifdef CONF_UART_DMA_CHANNEL_1_RX_BUFFER_SIZE
#define UART_DMA_CHANNEL_1_RX_BUFFER_SIZE   CONF_UART_DMA_CHANNEL_1_RX_BUFFER_SIZE
#else
#define UART_DMA_CHANNEL_1_RX_BUFFER_SIZE   20
#endif /*  UART_DMA_CHANNEL_1_RX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  UART_ENABLE_CHANNEL_1  */

#ifdef UART_ENABLE_CHANNEL_2

#ifdef UART_ENABLE_CHANNEL_2_TX

#ifdef CONF_UART_CHANNEL_2_TX_BUFFER_SIZE
#define UART_CHANNEL_2_TX_BUFFER_SIZE       CONF_UART_CHANNEL_2_TX_BUFFER_SIZE
#else
#define UART_CHANNEL_2_TX_BUFFER_SIZE       255
#endif /*  CONF_UART_CHANNEL_2_TX_BUFFER_SIZE  */

#ifdef CONF_UART_DMA_CHANNEL_2_TX_SIZE
#define UART_DMA_CHANNEL_2_TX_SIZE          CONF_UART_DMA_CHANNEL_2_TX_SIZE
#else
#define UART_DMA_CHANNEL_2_TX_SIZE          20
#endif /*  CONF_UART_DMA_CHANNEL_2_TX_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef UART_ENABLE_CHANNEL_2_RX

#ifdef CONF_UART_CHANNEL_2_RX_BUFFER_SIZE
#define UART_CHANNEL_2_RX_BUFFER_SIZE       CONF_UART_CHANNEL_2_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_2_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_2_RX_BUFFER_SIZE  */

#ifdef CONF_UART_DMA_CHANNEL_2_RX_BUFFER_SIZE
#define UART_DMA_CHANNEL_2_RX_BUFFER_SIZE   CONF_UART_DMA_CHANNEL_2_RX_BUFFER_SIZE
#else
#define UART_DMA_CHANNEL_2_RX_BUFFER_SIZE   20
#endif /*  UART_DMA_CHANNEL_2_RX_BUFFER_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  UART_ENABLE_CHANNEL_2  */

#ifdef UART_ENABLE_CHANNEL_3

#ifdef UART_ENABLE_CHANNEL_3_TX

#ifdef CONF_UART_CHANNEL_3_TX_BUFFER_SIZE
#define UART_CHANNEL_3_TX_BUFFER_SIZE       CONF_UART_CHANNEL_3_TX_BUFFER_SIZE
#else
#define UART_CHANNEL_3_TX_BUFFER_SIZE       255
#endif /*  CONF_UART_CHANNEL_3_TX_BUFFER_SIZE  */

#ifdef CONF_UART_DMA_CHANNEL_3_TX_SIZE
#define UART_DMA_CHANNEL_3_TX_SIZE          CONF_UART_DMA_CHANNEL_3_TX_SIZE
#else
#define UART_DMA_CHANNEL_3_TX_SIZE          20
#endif /*  CONF_UART_DMA_CHANNEL_3_TX_SIZE  */

#endif  /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef UART_ENABLE_CHANNEL_3_RX

#ifdef CONF_UART_CHANNEL_3_RX_BUFFER_SIZE
#define UART_CHANNEL_3_RX_BUFFER_SIZE       CONF_UART_CHANNEL_3_RX_BUFFER_SIZE
#else
#define UART_CHANNEL_3_RX_BUFFER_SIZE       255
#endif /*  UART_CHANNEL_3_RX_BUFFER_SIZE  */

#ifdef CONF_UART_DMA_CHANNEL_3_RX_BUFFER_SIZE
#define UART_DMA_CHANNEL_3_RX_BUFFER_SIZE   CONF_UART_DMA_CHANNEL_3_RX_BUFFER_SIZE
#else
#define UART_DMA_CHANNEL_3_RX_BUFFER_SIZE   20
#endif /*  UART_DMA_CHANNEL_3_RX_BUFFER_SIZE  */

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

#if (UART_DMA_CHANNEL_1_TX_SIZE < 1)
#error Channel 1 DMA TX size must be > 1
#endif /*  (UART_DMA_CHANNEL_1_TX_SIZE < 1)  */
#endif  /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef UART_ENABLE_CHANNEL_1_RX
#if (UART_CHANNEL_1_RX_BUFFER_SIZE < 2)
#error Channel 1 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_1_TX_BUFFER_SIZE < 2)  */

#if (UART_DMA_CHANNEL_1_RX_BUFFER_SIZE < 2)
#error Channel 1 DMA RX buffer must be > 1
#endif /*  (UART_DMA_CHANNEL_1_RX_BUFFER_SIZE < 2)  */
#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#endif  /*  UART_ENABLE_CHANNEL_1  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_2

#ifdef UART_ENABLE_CHANNEL_2_TX
#if (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)
#error Channel 2 TX buffer size must be > 1
#endif  /*  (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)  */

#if (UART_DMA_CHANNEL_2_TX_SIZE < 1)
#error Channel 2 DMA TX size must be > 1
#endif /*  (UART_DMA_CHANNEL_2_TX_SIZE < 1)  */
#endif  /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef UART_ENABLE_CHANNEL_2_RX
#if (UART_CHANNEL_2_RX_BUFFER_SIZE < 2)
#error Channel 2 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_2_TX_BUFFER_SIZE < 2)  */

#if (UART_DMA_CHANNEL_2_RX_BUFFER_SIZE < 2)
#error Channel 2 DMA RX buffer must be > 1
#endif /*  (UART_DMA_CHANNEL_2_RX_BUFFER_SIZE < 2)  */
#endif  /*  UART_ENABLE_CHANNEL_2_RX  */

#endif  /*  UART_ENABLE_CHANNEL_2  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_3

#ifdef UART_ENABLE_CHANNEL_3_TX
#if (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)
#error Channel 3 TX buffer size must be > 1
#endif  /*  (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)  */

#if (UART_DMA_CHANNEL_3_TX_SIZE < 1)
#error Channel 3 DMA TX size must be > 1
#endif /*  (UART_DMA_CHANNEL_3_TX_SIZE < 1)  */
#endif  /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef UART_ENABLE_CHANNEL_3_RX
#if (UART_CHANNEL_3_RX_BUFFER_SIZE < 2)
#error Channel 3 RX buffer size must be > 1
#endif  /*  (UART_CHANNEL_3_TX_BUFFER_SIZE < 2)  */

#if (UART_DMA_CHANNEL_3_RX_BUFFER_SIZE < 2)
#error Channel 3 DMA RX buffer must be > 1
#endif /*  (UART_DMA_CHANNEL_3_RX_BUFFER_SIZE < 2)  */
#endif  /*  UART_ENABLE_CHANNEL_3_RX  */

#endif  /*  UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */

#endif /* __UART_LL_DMA_CONF_H__ */
