/******************************************************************************
 * @file        uart.c
 * @brief       UART asynchronous read/write implementation using ST HAL 
 *              drivers for STM32F103CBTx
 * @version     1.0
 * @date        Apr 3, 2022
 * @copyright   
 *****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "buffer/ring_buffer.h"
#include "utils/utils.h"

#include "uart_conf.h"
#include "serial/uart/uart.h"

/* -------------------------------------------------------------------------- */

/**
 * @brief UART structure
 * */
typedef struct {
        UART_HandleTypeDef *    handle;         /**<  HAL UART handle, used by ST HAL drivers to interface with UART  */

        GPIO_TypeDef *          gpio_port;      /**<  pointer to UART TX/RX pins GPIO port, to initialize UART TX/RX gpio pins  */
        uint32_t                tx_pin;         /**<  UART TX gpio pin  */
        uint32_t                rx_pin;         /**<  UART RX gpio pin  */

        RingBuffer_t *          rx_buffer;      /**<  pointer to UART RX buffer  */
        uint8_t *               rx;             /**<  pointer to UART RX buffer array  */
        uint32_t                rx_size;        /**<  size of UART RX buffer  */

        RingBuffer_t *          tx_buffer;      /**<  pointer to UART TX buffer  */
        uint8_t *               tx;             /**<  pointer to UART TX buffer array  */
        uint32_t                tx_size;        /**<  size of UART TX buffer  */

        IRQn_Type               uart_irq;       /**<  UART IRQn (IRQ number), used to enable/disable UART interrupts in NVIC  */
} UART_t;

/* -------------------------------------------------------------------------- */

/**
 * @brief Handle UART TX, RX and errors (PE : Parity Error, NE : Noise Error,
 *        FE :  Frame Error and ORE : Over-Run Error).
 * 
 * @param [in,out] psUart UART_t pointer of UART channel (UART_1, UART_2, UART_3, ...)
 * 
 */
static void UART_vidIrqCallback(UART_t const * const psUart);

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_1)

static UART_HandleTypeDef UART_Channel_1_Handle = {
        .Instance = USART1
};

#ifdef UART_ENABLE_CHANNEL_1_RX
static RingBuffer_t UART_ChanneL_1_RxBuffer;
static uint8_t UART_Channel_1_RX_Data [UART_CHANNEL_1_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
static RingBuffer_t UART_ChanneL_1_TxBuffer;
static uint8_t UART_Channel_1_TX_Data [UART_CHANNEL_1_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

static const UART_t UART_1 = {

        .handle = &UART_Channel_1_Handle,

        .gpio_port = GPIOA,
        .tx_pin = GPIO_PIN_9,
        .rx_pin = GPIO_PIN_10,

#ifdef UART_ENABLE_CHANNEL_1_RX
        .rx_buffer = &UART_ChanneL_1_RxBuffer,
        .rx = UART_Channel_1_RX_Data,
        .rx_size = UART_CHANNEL_1_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
        .tx_buffer = &UART_ChanneL_1_TxBuffer,
        .tx = UART_Channel_1_TX_Data,
        .tx_size = UART_CHANNEL_1_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

        .uart_irq = USART1_IRQn
};

#endif  /*  UART_ENABLE_CHANNEL_1   */

/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_2)

static UART_HandleTypeDef UART_Channel_2_Handle = {
        .Instance = USART2
};

#ifdef UART_ENABLE_CHANNEL_2_RX
static RingBuffer_t UART_ChanneL_2_RxBuffer;
static uint8_t UART_Channel_2_RX_Data [UART_CHANNEL_2_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
static RingBuffer_t UART_ChanneL_2_TxBuffer;
static uint8_t UART_Channel_2_TX_Data [UART_CHANNEL_2_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

static const UART_t UART_2 = {

        .handle = &UART_Channel_2_Handle,

        .gpio_port = GPIOA,
        .tx_pin = GPIO_PIN_2,
        .rx_pin = GPIO_PIN_3,

        #ifdef UART_ENABLE_CHANNEL_2_RX
        .rx_buffer = &UART_ChanneL_2_RxBuffer,
        .rx = UART_Channel_2_RX_Data,
        .rx_size = UART_CHANNEL_2_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
        .tx_buffer = &UART_ChanneL_2_TxBuffer,
        .tx = UART_Channel_2_TX_Data,
        .tx_size = UART_CHANNEL_2_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

        .uart_irq = USART2_IRQn
};

#endif  /*  UART_ENABLE_CHANNEL_2   */

/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_3)

static UART_HandleTypeDef UART_Channel_3_Handle = {
        .Instance = USART3
};

#ifdef UART_ENABLE_CHANNEL_3_RX
static RingBuffer_t UART_ChanneL_3_RxBuffer;
static uint8_t UART_Channel_3_RX_Data [UART_CHANNEL_3_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
static RingBuffer_t UART_ChanneL_3_TxBuffer;
static uint8_t UART_Channel_3_TX_Data [UART_CHANNEL_3_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

static const UART_t UART_3 = {

        .handle = &UART_Channel_3_Handle,

        .gpio_port = GPIOB,
        .tx_pin = GPIO_PIN_10,
        .rx_pin = GPIO_PIN_11,

#ifdef UART_ENABLE_CHANNEL_3_RX
        .rx_buffer = &UART_ChanneL_3_RxBuffer,
        .rx = UART_Channel_3_RX_Data,
        .rx_size = UART_CHANNEL_3_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
        .tx_buffer = &UART_ChanneL_3_TxBuffer,
        .tx = UART_Channel_3_TX_Data,
        .tx_size = UART_CHANNEL_3_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

        .uart_irq = USART3_IRQn
};

#endif  /*  UART_ENABLE_CHANNEL_3   */

/* ------------------------------------------------------------------------- */

/**
 * Array to UART channels' handles
 * */
static const UART_t * const UART_asHandles[UART_CHANNEL_COUNT] = {

#if defined(UART_ENABLE_CHANNEL_1)
        &UART_1,
#else
        NULL,
#endif  /*  UART_ENABLE_CHANNEL_1   */

#if defined(UART_ENABLE_CHANNEL_2)
        &UART_2,
#else
        NULL,
#endif  /*  UART_ENABLE_CHANNEL_2   */

#if defined(UART_ENABLE_CHANNEL_3)
        &UART_3,
#else
        NULL,
#endif  /*  UART_ENABLE_CHANNEL_3   */

};

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enInitialize(const UART_Channel_t enChannel, UART_Conf_t const * const psConf)
{
    GPIO_InitTypeDef Local_sGpioInit = {0};
    UART_Conf_t Local_sUartConf = {0};
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if(IS_NULLPTR(psConf))
    {
        return UART_ERROR_NULLPTR;
    }

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t *)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /* enable UART clock and its pin's GPIO port clock */
    switch(enChannel)
    {
        case UART_CHANNEL_1:
        {
            __HAL_RCC_USART1_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        break;

        case UART_CHANNEL_2:
        {
            __HAL_RCC_USART2_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        break;

        case UART_CHANNEL_3:
        {
            __HAL_RCC_USART3_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        break;

        case UART_CHANNEL_COUNT:
        default:
        {
            return UART_ERROR_INVALID_CHANNEL;
        }
    }

    /* copy UART configurations */
    memcpy(&Local_sUartConf, psConf, sizeof Local_sUartConf);
    Local_sUartConf.Mode = 0;

    /*  initialize UART RX buffer  */
    if(Local_psUart->rx_size)
    {
        Local_sUartConf.Mode |= UART_MODE_RX;

        Local_enBufferError = RingBuffer_enInit(Local_psUart->rx_buffer, Local_psUart->rx, Local_psUart->rx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioInit.Pin = Local_psUart->rx_pin;
        Local_sGpioInit.Mode = GPIO_MODE_INPUT;
        Local_sGpioInit.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioInit);
    }

    /*  initialize UART TX buffer  */
    if(Local_psUart->tx_size)
    {
        Local_sUartConf.Mode |= UART_MODE_TX;

        Local_enBufferError = RingBuffer_enInit(Local_psUart->tx_buffer, Local_psUart->tx, Local_psUart->tx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioInit.Pin = Local_psUart->tx_pin;
        Local_sGpioInit.Mode = GPIO_MODE_AF_PP;
        Local_sGpioInit.Pull = GPIO_PULLUP;
        Local_sGpioInit.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioInit);
    }

    /*  configure UART channel  */
    memcpy(&Local_psUart->handle->Init, &Local_sUartConf, sizeof(Local_sUartConf));

    if(HAL_UART_Init(Local_psUart->handle) != HAL_OK)
    {
        return UART_ERROR_NOT_INIT;
    }

#if !defined(UART_MINIMAL_INTERRUPTS)

    /*  enable UART RX/Error interrupts if RX mode is enabled  */
    if((Local_sUartConf.Mode & UART_MODE_RX) == UART_MODE_RX)
    {
        __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_RXNE);
    }

    /*  enable UART interrupts  */
    HAL_NVIC_SetPriority(Local_psUart->uart_irq, UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY);
    HAL_NVIC_EnableIRQ(Local_psUart->uart_irq);

#endif /* !defined(UART_MINIMAL_INTERRUPTS) */

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enDeInitialize(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t *)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

#if !defined(UART_MINIMAL_INTERRUPTS)

    /*  disable UART NVIC interrupts  */
    HAL_NVIC_DisableIRQ(Local_psUart->uart_irq);

    /*  disable UART RX interrupt  */
    if(Local_psUart->handle->Init.Mode & UART_MODE_RX)
    {
        __HAL_UART_DISABLE_IT(Local_psUart->handle, UART_IT_RXNE);
    }

    /*  disable UART TX interrupt  */
    if(Local_psUart->handle->Init.Mode & UART_MODE_TX)
    {
        __HAL_UART_DISABLE_IT(Local_psUart->handle, UART_IT_TXE);
    }
    
#endif /* !defined(UART_MINIMAL_INTERRUPTS) */

    /*  reset RX/TX buffers */
    RingBuffer_enReset(Local_psUart->tx_buffer);
    RingBuffer_enReset(Local_psUart->rx_buffer);

    /*  de-initialize UART */
    if(HAL_UART_DeInit(Local_psUart->handle) != HAL_OK)
    {
        return UART_ERROR_NOT_INIT;
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enRead(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count)
{
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

    if(IS_NULLPTR(pu8Data) || IS_NULLPTR(pu32Count))
    {
        return UART_ERROR_NULLPTR;
    }

    if(IS_ZERO(u32Len))
    {
        return UART_ERROR_INVALID_PARAM;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /* read byte from UART RX buffer */
    Local_enBufferError = RingBuffer_enGetItems(Local_psUart->rx_buffer, pu8Data, u32Len, pu32Count);

    /**
     * @note ring buffer return NULPTR error if ring buffer is not initialized,
     * that means UART_enRead() was called before initializing UART channel
     */
    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }

    /* If RX buffer is empty, return buffer empty error */
    if(Local_enBufferError == RING_BUFFER_ERROR_EMPTY)
    {
        /**
         * @note RingBuffer_enGetItems() sets pu32Count to 0 when ring buffer is empty
         */
        return UART_ERROR_BUFFER_EMPTY;
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enReadUntil(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint8_t u8Until, uint32_t * const pu32Count)
{
    uint32_t Local_u32Count = 0;
    UART_t * Local_psUart = NULL;
    uint8_t Local_u8Byte = 0;
    RingBuffer_Error_t Local_enBufferError = RING_BUFFER_ERROR_NONE;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

    if(IS_NULLPTR(pu8Data) || IS_NULLPTR(pu32Count))
    {
        return UART_ERROR_NULLPTR;
    }

    if(IS_ZERO(u32Len))
    {
        return UART_ERROR_INVALID_PARAM;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /**
     *  read bytes from UART RX buffer until one of the following conditions is true:
     *  - UART RX buffer is empty 
     *  - a byte is read that has the value == u8Until
     *  - u32Len bytes were read from UART RX buffer
     * */
    while((Local_u32Count < u32Len) && ((Local_enBufferError = RingBuffer_enGetItem(Local_psUart->rx_buffer, &Local_u8Byte)) == RING_BUFFER_ERROR_NONE))
    {
        pu8Data[Local_u32Count++] = Local_u8Byte;

        if(Local_u8Byte == u8Until)
        {
            break;
        }
        else
        {
            (void)0;
        }
    }

    (*pu32Count) = Local_u32Count;

    /**
     * @note ring buffer return NULPTR error if ring buffer is not initialized,
     * that means UART_enReadUntil() was called before initializing UART channel
     */
    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }

    /**
     * if UARTs empty, return buffer empty error
     * 
     * @note RingBuffer_enGetItems() sets pu32Count to 0 when ring buffer is empty
     */
    if(Local_enBufferError == RING_BUFFER_ERROR_EMPTY)
    {
        return UART_ERROR_BUFFER_EMPTY;
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enReadLine(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint32_t * const pu16Count)
{
    return UART_enReadUntil(enChannel, pu8Data, u32Len, '\n', pu16Count);
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWrite(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count)
{
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

    if(IS_NULLPTR(pu8Data) || IS_NULLPTR(pu32Count))
    {
        return UART_ERROR_NULLPTR;
    }

    if(IS_ZERO(u32Len))
    {
        return UART_ERROR_INVALID_PARAM;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /* write bytes to UART TX buffer */
    Local_enBufferError = RingBuffer_enPutItems(Local_psUart->tx_buffer, pu8Data, u32Len, pu32Count);

    /**
     * @note ring buffer return NULPTR error if ring buffer is not initialized,
     * that means UART_enWrite() was called before initializing UART channel
     */
    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }

    /**
     *  If UART TX buffer is full, return buffer full error,
     *  else enable UART TXE interrupt (if not enabled)
     * 
     * @note RingBuffer_enPutItems() sets pu32Count to 0 when ring buffer is full
     * */
    if(Local_enBufferError == RING_BUFFER_ERROR_FULL)
    {
        return UART_ERROR_BUFFER_FULL;
    }
    else
    {

#if !defined(UART_MINIMAL_INTERRUPTS)
        /**
         * @note enabling UART TXE interrupt while UART_IRQn is no enabled in NVIC will not trigger UART interrupts.
         * However, it's better to not enable them at all when they are not needed
         */
        if(!__HAL_UART_GET_IT_SOURCE(Local_psUart->handle, UART_IT_TXE))
        {
            __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_TXE);
        }
#endif /* defined(UART_MINIMAL_INTERRUPTS) */

    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWriteBlocking(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len)
{
    UART_t * Local_psUart = NULL;
    uint32_t Local_u32TransmitCount = 0;
    uint32_t Local_u32WriteCount;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

    if(IS_NULLPTR(pu8Data))
    {
        return UART_ERROR_NULLPTR;
    }

    if(IS_ZERO(u32Len))
    {
        return UART_ERROR_INVALID_PARAM;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */


    while(Local_u32TransmitCount < u32Len)
    {
        /* write data to UART TX buffer  */
        UART_enWrite(
            enChannel, 
            &pu8Data[Local_u32TransmitCount], 
            u32Len - Local_u32TransmitCount, 
            &Local_u32WriteCount
        );

        Local_u32TransmitCount += Local_u32WriteCount;

        UART_enFlushTx(enChannel);
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enFlushTx(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;
    uint32_t Local_u32RemainingBytes = 0;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /* wait until all bytes in UART TX buffer are transmitted (UART TX buffer is empty) */
    do
    {
        UART_enUpdateChannel(enChannel);
        RingBuffer_enItemCount(Local_psUart->tx_buffer, &Local_u32RemainingBytes);
    }
    while(Local_u32RemainingBytes > 0);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enFlushRx(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /* reset UART RX buffer (UART RX buffer becomes empty) */
    RingBuffer_enReset(Local_psUart->rx_buffer);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enUpdateChannel(UART_Channel_t enChannel)
{

#if defined(UART_MINIMAL_INTERRUPTS)

    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enError;
    uint8_t Local_u8Byte;
    uint32_t Local_u32Status;

#ifdef DEBUG

    if ((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t *)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }
    else
    {
        (void)0;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /**
     * Handle UART error handling:
     * 
     * - Error flags (in USARTx->SR) are cleared by: 
     *   1- reading UARTx->SR register, followed by
     *   2- reading USARTx->DR register
     * */
    Local_u32Status = Local_psUart->handle->Instance->SR;

    /**
     * Handle UART RX
     * 
     * If UART->RXNE flag is set (when a byte is received on UART):
     *   - read byte from UART->DR 
     *   - If no parity error:
     *     - put byte into UART->rx_buffer
     *   - Else:
     *     - Ignore read byte with parity error
     * */
    if((Local_u32Status & UART_FLAG_RXNE) == UART_FLAG_RXNE)
    {
        Local_u8Byte = (uint8_t)Local_psUart->handle->Instance->DR;
        if((Local_u32Status & (UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_PE)) == FALSE)
        {
            Local_enError = RingBuffer_enPutItem(Local_psUart->rx_buffer, &Local_u8Byte);
        }
    }

    /**
     * Handle UART TX
     * 
     * If UART->TXE flag is set (TXE flag is set when UART TX is enabled, and after a byte is transmitted from UART->DR):
     *   - If UART->tx_buffer has data bytes (not empty):
     *     - ready byte from UART->tx_buffer and send it (writing to DR clears TXE flag)
     * */
    if((Local_u32Status & UART_FLAG_TXE) == UART_FLAG_TXE)
    {
        Local_enError = RingBuffer_enGetItem(Local_psUart->tx_buffer, &Local_u8Byte);
        if(Local_enError == RING_BUFFER_ERROR_NONE)
        {
            Local_psUart->handle->Instance->DR = Local_u8Byte;
        }
    }

#else

    UNUSED(enChannel);

#endif /* defined(UART_MINIMAL_INTERRUPTS) */

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

static void UART_vidIrqCallback(UART_t const * const psUart)
{
    RingBuffer_Error_t Local_enError;
    uint8_t Local_u8Byte;
    uint32_t Local_u32Status;

    /**
     * Handle UART error handling:
     * 
     * - Error flags (in USARTx->SR) are cleared by: 
     *   1- reading UARTx->SR register, followed by
     *   2- reading USARTx->DR register
     * */
    Local_u32Status = psUart->handle->Instance->SR;

    /**
     * Handle UART RX
     * 
     * If UART->RXNE flag is set (when a byte is received on UART) and
     * If UART->RXNEI (RX not empty interrupt):
     *   - read byte from UART->DR 
     *   - If no parity error:
     *     - put byte into UART->rx_buffer
     *   - Else:
     *     - Ignore read byte with parity error
     * */
    if(__HAL_UART_GET_IT_SOURCE(psUart->handle, UART_IT_RXNE) && ((Local_u32Status & UART_FLAG_RXNE) == UART_FLAG_RXNE))
    {
        Local_u8Byte = (uint8_t)psUart->handle->Instance->DR;
        if((Local_u32Status & (UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_PE)) == FALSE)
        {
            Local_enError = RingBuffer_enPutItem(psUart->rx_buffer, &Local_u8Byte);
        }
    }

    /**
     * Handle UART TX
     * 
     * If UART->TXE flag is set (TXE flag is set when UART TX is enabled, and after a byte is transmitted from UART->DR) and
     * If UART->TXEI (TX empty interrupt) is enabled (enabled in UART_enWrite()):
     *   - If UART->tx_buffer has data bytes (not empty):
     *     - ready byte from UART->tx_buffer and send it
     * 
     *   - else (UART->tx_buffer is empty):
     *     - disable UART->TXEI (TX empty interrupt)
     * */
    if(__HAL_UART_GET_IT_SOURCE(psUart->handle, UART_IT_TXE) && ((Local_u32Status & UART_FLAG_TXE) == UART_FLAG_TXE))
    {
        Local_enError = RingBuffer_enGetItem(psUart->tx_buffer, &Local_u8Byte);
        if(Local_enError == RING_BUFFER_ERROR_NONE)
        {
            psUart->handle->Instance->DR = Local_u8Byte;
        }
        else
        {
            /**
             * @note disable TXE interrupt when buffer is empty, 
             * because TXE flag will always be asserted (until a new byte is written to UART->DR)
             * and that keeps triggerring UART interrupt
             * */
            __HAL_UART_DISABLE_IT(psUart->handle, UART_IT_TXE);
        }
    }
}

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_1)

void USART1_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_1);
}

#endif /*   UART_ENABLE_CHANNEL_1  */

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_2)

void USART2_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_2);
}

#endif /*   UART_ENABLE_CHANNEL_2  */

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_3)

void USART3_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_3);
}

#endif /*   UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */
