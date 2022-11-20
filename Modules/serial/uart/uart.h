/******************************************************************************
 * @file      uart.h
 * @brief     UART module for handling data transmission and reception for STM32F103x.
 * @details   The module offers blocking and non blocking UART data transmission,
 *            and non blocking UART data reception
 *
 *            To implement non blocking mode, the module relies on UART interrupts
 *            and/or DMA.
 *
 *            The module offers the following APIs for data transmission and
 *            reception :
 *
 *              UART data transmission is available in either non-blocking or
 *              blocking mode for each channel.
 *                - UART_enWrite()            : queue data bytes for transmission
 *                                              in UAR TX buffer
 *                - UART_enWriteBlocking()    : sends data bytes immediately
 *                - UART_enFlushTX()          : wait till all pending data bytes are
 *                                              transmitted
 *
 *              UART data reception is available in non-blocking mode only,
 *              received data bytes are stored in UART channel's RX buffer
 *              and read through functions:
 *                - UART_enRead()         : read number of bytes from UART RX buffer
 *                - UART_enReadUntil()    : read bytes from UART RX buffer until a
 *                                          certain byte value, or the requested
 *                                          number of bytes
 *                - UART_enReadLine()     : read bytes from UART RX buffer until
 *                                          new line character, or the requested
 *                                          number of bytes
 *
 *            The module contains 3 source file, each implementing the APIs
 *            from 'uart.h':
 *              - uart.c        : uses STM32 HAL library to transmit/receive
 *                                UART data using UART interrupts.
 *              - uart_ll.c     : uses STM32 LL library to transmit/receive
 *                                UART data using UART interrupts.
 *              - uart_ll_dma.c : uses STM32 LL library to transmit/receive
 *                                UART data using UART and DMA interrupts.
 *
 *             Check each file's README for more detailed information
 *
 *             The module can be built using <i>only</i> one of the source files,
 *             and can be configured from associated header file:
 *              - uart_conf.h           : contains UART channels configurations
 *                                        used with uart.c
 *              - uart_ll_conf.h        : contains UART channels configurations
 *                                        used with uart_ll.c
 *              - uart_ll_dma_conf.h    : contains UART channels configurations
 *                                        used with uart_ll_dma.c
 *
 * @version   1.0
 * @date      Apr 3, 2022
 * @copyright
 *****************************************************************************/
#ifndef __UART_UART_H__
#define __UART_UART_H__

#include "main.h"

/**
 * @defgroup serial_uart UART module to handle transmission and reception asynchronously
 * @brief UART module to handle transmission and reception asynchronously
 * @{
 * */

/**
 * UART channel number, used by UART functions like UART_enInitialize(),
 * uART_enRead(), etc
 *
 * UART channels can be enabled/disabled by macros
 * #UART_ENABLE_CHANNEL_1, #UART_ENABLE_CHANNEL_2 and #UART_ENABLE_CHANNEL_3
 * */
typedef enum uart_channel_t {
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2 = 1,
    UART_CHANNEL_3 = 2,
    UART_CHANNEL_COUNT,
} UART_Channel_t;

/**
 * UART error, returned by UART functions to indicate if an error occurred
 * during execution
 * */
typedef enum uart_error_t {
    UART_ERROR_NONE = 0,            /**<  None Error: no error occurred during function execution  >*/
    UART_ERROR_NULLPTR,             /**<  Null pointer error: an unexpected null pointer  >*/
    UART_ERROR_INVALID_PARAM,       /**<  Invalid parameter error: a parameter was passed with an invalid value (like len=0)  >*/
    UART_ERROR_INVALID_CHANNEL,     /**<  Invalid channel error: Passed UART channel is invalid  >*/
    UART_ERROR_NOT_INIT,            /**<  Not Initialized error: attempting to use a UART channel that was not initialized  >*/
    UART_ERROR_BUFFER_EMPTY,        /**<  Buffer empty error: read failed because UART RX buffer is empty  >*/
    UART_ERROR_BUFFER_FULL,         /**<  Buffer full error: write failed because UART TX buffer is full  >*/
} UART_Error_t;

/**
 * UART configuration structure used to initialize UART channel
 * */
#if defined(USE_FULL_LL_DRIVER)
typedef LL_USART_InitTypeDef UART_Conf_t;
#elif defined(USE_HAL_DRIVER)
typedef UART_InitTypeDef UART_Conf_t;
#endif /*  defined(USE_FULL_LL_DRIVER)  */

/**
 * Initialize UART channel using given configurations
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [in]  psConf      : pointer to UART channels' configuration structure
 *
 * @pre UART channel is enabled in UART configuration header file
 *
 * @post UART channel is ready to receive and transmit data
 * @post UART channel can be used with other module functions to read/write data
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (psConf is NULL)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *     UART_ERROR_NOT_INIT          : Not Initialized error, error during UART initialization
 *
 * */
UART_Error_t UART_enInitialize(const UART_Channel_t enChannel, UART_Conf_t const * const psConf);

/**
 * De-initialize UART channel
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post UART channel is disabled, can't be used to read/write data
 * @post Buffered data in UART channel's RX buffer are dropped
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *
 * */
UART_Error_t UART_enDeInitialize(const UART_Channel_t enChannel);

/**
 * Read bytes from UART RX buffer
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [out] pu8Data     : pointer to an array of bytes to write read bytes from UART channel's RX buffer
 * @param [in]  u32Len      : number of data bytes to read from UART channels' RX buffer (must be > 0)
 * @param [out] pu32Count   : pointer to an unsigned 32-bit variable, where number of read data bytes will be written
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post If UART Channel's RX buffer is empty, pu32Count contains zero and the function returns UART_ERROR_BUFFER_EMPTY
 *
 * @post If UART Channel's RX buffer is not empty but contains fewer bytes than requested \f( available bytes < u32Len \f),
 *       data bytes are copied from the RX buffer and pu32Count contains the number of copied bytes \f( (*pu32Count) < u32Len \f).
 *       As a result, the RX buffer is now empty. The function returns UART_ERROR_NONE.
 *
 * @post If UART channel's RX buffer is not empty and contains enough data, the requested data bytes are copied
 *       and \f((*pu32Count) == u32Len\f). The function returns UART_ERROR_NONE.
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (pu8Data or pu32Count is NULL)
 *     UART_ERROR_INVALID_PARAM     : Null pointer error, an unexpected null pointer (u32Len is zero)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *     UART_ERROR_BUFFER_EMPTY      : Buffer empty error, UART channels RX buffer is empty
 *
 * */
UART_Error_t UART_enRead(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count);

/**
 * Read bytes from UART RX buffer up to a certain byte value, or a number of bytes
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [out] pu8Data     : pointer to an array of bytes to write read bytes from UART channel's RX buffer
 * @param [in]  u32Len      : number of data bytes to read from UART channels' RX buffer (must be > 0)
 * @param [in]  u8Until     : byte value at which to stop reading from RX buffer
 * @param [out] pu32Count   : pointer to an unsigned 32-bit variable, where number of read data bytes will be written
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post If UART Channel's RX buffer is empty, @arg pu32Count contains zero and the function returns #UART_ERROR_BUFFER_EMPTY
 *
 * @post If UART Channel's RX buffer is not empty, contains the byte @arg u8Until within the first @arg u32Len bytes,
 * data bytes are read from RX buffer and @arg pu32Cout contains number of read bytes \f((*pu32Count) <= u32Len \f)
 *
 * @post If UART channel's RX buffer is not empty, doesn't contain the byte value @arg u8Until within the first @arg u32Len bytes,
 * data bytes are read from RX buffer and @arg pu32Count contains number of read bytes \f((*pu32Count) == u32Len\f)
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (pu8Data or pu32Count is NULL)
 *     UART_ERROR_INVALID_PARAM     : Null pointer error, an unexpected null pointer (u32Len is zero)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *     UART_ERROR_BUFFER_EMPTY      : Buffer empty error, UART channels RX buffer is empty
 *
 * */
UART_Error_t UART_enReadUntil(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint8_t u8Until, uint32_t * const pu32Count);

/**
 * Read bytes from UART RX buffer up to new line character <CR>, or a number of bytes
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [out] pu8Data     : pointer to an array of bytes to write read bytes from UART channel's RX buffer
 * @param [in]  u32Len      : number of data bytes to read from UART channels' RX buffer (must be > 0)
 * @param [in]  u8Until     : byte value at which to stop reading from RX buffer
 * @param [out] pu32Count   : pointer to an unsigned 32-bit variable, where number of read data bytes will be written
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post If UART Channel's RX buffer is empty, @arg pu32Count contains zero and the function returns UART_ERROR_BUFFER_EMPTY
 *
 * @post If UART Channel's RX buffer is not empty, contains the byte <CR> within the first @arg u32Len bytes,
 * data bytes are read from RX buffer and @arg pu32Cout contains number of read bytes \f((*pu32Count) <= u32Len \f)
 *
 * @post If UART channel's RX buffer is not empty, doesn't contain the byte value <CR> within the first @arg u32Len bytes,
 * data bytes are read from RX buffer and @arg pu32Count contains number of read bytes \f((*pu32Count) == u32Len\f)
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (pu8Data or pu32Count is NULL)
 *     UART_ERROR_INVALID_PARAM     : Null pointer error, an unexpected null pointer (u32Len is zero)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *     UART_ERROR_BUFFER_EMPTY      : Buffer empty error, UART channels RX buffer is empty
 *
 * */
UART_Error_t UART_enReadLine(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count);

/**
 * Write bytes to UART channel's TX buffer to be transmitted
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [out] pu8Data     : pointer to an array of bytes to write read bytes from UART channel's RX buffer
 * @param [in]  u32Len      : number of data bytes to read from UART channels' RX buffer (must be > 0)
 * @param [out] pu32Count   : pointer to an unsigned 32-bit variable, where number of read data bytes will be written
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post Bytes will be added to UART TX buffer, and @arg pu32Count will contain number of bytes added to UART TX buffer
 *
 * @post If there is no ongoing transmission, UART will start transmitting the added bytes immediately
 *
 * @post If there is an ongoing transmission, UART will start transmitting the added bytes as soon as
 * previously added bytes are transmitted
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (pu8Data or pu32Count is NULL)
 *     UART_ERROR_INVALID_PARAM     : Null pointer error, an unexpected null pointer (u32Len is zero)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *     UART_ERROR_BUFFER_FULL       : Buffer full error, UART channels TX buffer is full
 *
 * */
UART_Error_t UART_enWrite(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count);

/**
 * Transmit data bytes on UART channel
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 * @param [out] pu8Data     : pointer to an array of bytes to write read bytes from UART channel's RX buffer
 * @param [in]  u32Len      : number of data bytes to read from UART channels' RX buffer (must be > 0)
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post Bytes will be transmitted on UART channel's TX
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_NULLPTR           : Null pointer error, an unexpected null pointer (pu8Data or pu32Count is NULL)
 *     UART_ERROR_INVALID_PARAM     : Null pointer error, an unexpected null pointer (u32Len is zero)
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *
 * */
UART_Error_t UART_enWriteBlocking(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len);

/**
 * Wait until UART completes sending all bytes in its TX buffer
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post All data bytes in UART channel's TX buffer are transmitted (UART channel's TX buffer is empty)
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *
 * */
UART_Error_t UART_enFlushTx(const UART_Channel_t enChannel);

/**
 * Reset UART RX buffer
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post All bytes in UART RX buffer are dropped (UART channel's RX buffer is empty)
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *
 * */
UART_Error_t UART_enFlushRx(const UART_Channel_t enChannel);

/**
 * @brief Update UART channel's TX/RX buffer
 *
 * @note Only useful when using uart_ll_dma.c with #UART_DMA_MINIMAL_INTERRUPTS enabled
 *
 * @note Must be called regularly with a frequency to match UART baudrate and DMA TX/RX buffer size
 *
 * @pre UART channel is initialized using UART_enInitialize()
 *
 * @post Received data bytes are copied from DMA RX buffer into UART channel's RX buffer
 *
 * @post Transmitted bytes are removed from UART channel's TX buffer
 *
 * @param [in]  enChannel   : UART channel number, UART_CHANNEL_X where X is {1, 2, 3}, @see UART_Channel_t
 *
 * @return #UART_Error_t
 *     UART_ERROR_NONE              : None Error, UART channel was initialized successfully
 *     UART_ERROR_INVALID_CHANNEL   : Invalid channel error, UART channel is not available \f( >= #UART_CHANNEL_COUNT \f) or not enabled in UART configuration file
 *
 * */
UART_Error_t UART_enUpdateChannel(UART_Channel_t enChannel);

/**@}*/

#endif /* __UART_UART_H__ */


