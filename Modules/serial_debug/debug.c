/******************************************************************************
 * @file      debug.c
 * @brief
 * @version   1.0
 * @date      Apr 17, 2022
 * @copyright
 *****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <main.h>

#include "board_config.h"

#include "serial/uart/uart.h"
#include "debug.h"

#if defined(DEBUG)

#ifdef USE_HAL_DRIVER

// static const UART_Conf_t Debug_sUartConf = {
//     .BaudRate               = CONF_DEBUG_UART_BAUDRATE,
//     .Parity                 = CONF_DEBUG_UART_PARITY,
//     .StopBits               = CONF_DEBUG_UART_STOPBITS,
//     .WordLength             = CONF_DEBUG_UART_DATASIZE,
//     .Mode                   = UART_MODE_TX,
//     .HwFlowCtl              = UART_HWCONTROL_NONE,
//     .OverSampling           = UART_OVERSAMPLING_16,
// };

#else

// static const UART_Conf_t Debug_sUartConf = {
//     .BaudRate               = CONF_DEBUG_UART_BAUDRATE,
//     .Parity                 = CONF_DEBUG_UART_PARITY,
//     .StopBits               = CONF_DEBUG_UART_STOPBITS,
//     .DataWidth              = CONF_DEBUG_UART_DATASIZE,
//     .TransferDirection      = LL_USART_DIRECTION_TX,
//     .HardwareFlowControl    = LL_USART_HWCONTROL_NONE,
//     .OverSampling           = LL_USART_OVERSAMPLING_16,
// };

#endif /*  USE_HAL_DRIVER  */

/* ------------------------------------------------------------------------- */

void
Debug_vidInitialize(void)
{
    UART_enInitialize(CONF_DEBUG_UART_CHANNEL, (UART_Conf_t *)&Debug_sUartConf);
}

/* ------------------------------------------------------------------------- */

void
Debug_vidFlushOutput(void)
{
    UART_enFlushTx(CONF_DEBUG_UART_CHANNEL);
}

/* ------------------------------------------------------------------------- */

void
Debug_vidDeInitialize(void)
{
    UART_Error_t Local_enError;

    UART_enFlushTx(CONF_DEBUG_UART_CHANNEL);

    Local_enError = UART_enDeInitialize(CONF_DEBUG_UART_CHANNEL);

    if(Local_enError != UART_ERROR_NONE)
    {
        return;
    }
}

/* ------------------------------------------------------------------------- */

extern
int
__io_putchar(int ch)
{
    uint32_t Local_u32Count = 0;

    UART_enWrite(CONF_DEBUG_UART_CHANNEL, (uint8_t *)&ch, (uint32_t)1, &Local_u32Count);

    return (int)Local_u32Count;
}

/* ------------------------------------------------------------------------- */

int
_write(int file, char * ptr, int len)
{
    uint32_t Local_u32Count = 0;

    UART_enWrite(CONF_DEBUG_UART_CHANNEL, (uint8_t *)ptr, (uint32_t)len, &Local_u32Count);

    (void)file;

    return (int)Local_u32Count;
}

/* ------------------------------------------------------------------------- */

int
_read(int file, char * ptr, int len)
{
    uint32_t Local_u32Count = 0;

    UART_enRead(CONF_DEBUG_UART_CHANNEL, (uint8_t *)ptr, (uint32_t)len, &Local_u32Count);

    (void)file;

    return (int)Local_u32Count;
}

/* ------------------------------------------------------------------------- */

#endif /*   DEBUG  */
