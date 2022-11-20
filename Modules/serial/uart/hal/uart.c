/******************************************************************************
 * @file      port_uart.c
 * @brief
 * @version   1.0
 * @date      Apr 3, 2022
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
 * UART structure
 * */
typedef struct {
        UART_HandleTypeDef *    handle;         /**<    */

        GPIO_TypeDef *          gpio_port;      /**<    */
        uint32_t                tx_pin;         /**<    */
        uint32_t                rx_pin;         /**<    */

        RingBuffer_t *          rx_buffer;      /**<    */
        uint8_t *               rx;             /**<    */
        uint32_t                rx_size;        /**<    */

        RingBuffer_t *          tx_buffer;      /**<    */
        uint8_t *               tx;             /**<    */
        uint32_t                tx_size;        /**<    */

        IRQn_Type               uart_irq;       /**<    */
} UART_t;

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
 * Array to UART handles
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
    RingBuffer_Error_t Local_enBufferError;
    UART_t * Local_psUart = NULL;
    GPIO_InitTypeDef Local_sGpioInit = {0};

#ifdef DEBUG

    if(IS_NULLPTR(psConf))
    {
        return UART_ERROR_NULLPTR;
    }

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

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

    /*  initialize UART RX buffers  */
    if(Local_psUart->rx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->rx_buffer, Local_psUart->rx, Local_psUart->rx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioInit.Pin = Local_psUart->rx_pin;
        Local_sGpioInit.Mode = GPIO_MODE_INPUT;
        Local_sGpioInit.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioInit);
    }

    /*  initialize UART TX buffers  */
    if(Local_psUart->tx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->tx_buffer, Local_psUart->tx, Local_psUart->tx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioInit.Pin = Local_psUart->tx_pin;
        Local_sGpioInit.Mode = GPIO_MODE_AF_PP;
        Local_sGpioInit.Pull = GPIO_PULLUP;
        Local_sGpioInit.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioInit);
    }

    /*  configure UART channel  */
    memcpy(&Local_psUart->handle->Init, psConf, sizeof(Local_psUart->handle->Init));

    if(HAL_UART_Init(Local_psUart->handle) != HAL_OK)
    {
        return UART_ERROR_NOT_INIT;
    }

    /*  enable UART RX/Error interrupts if RX mode is enabled  */
    if((psConf->Mode & UART_MODE_RX) == UART_MODE_RX)
    {
        __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_PE);
        __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_ERR);
    }

    /*  enable UART interrupts  */
    HAL_NVIC_SetPriority(Local_psUart->uart_irq, UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY);
    HAL_NVIC_EnableIRQ(Local_psUart->uart_irq);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enDeInitialize(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /*  disable UART NVIC interrupts  */
    HAL_NVIC_DisableIRQ(Local_psUart->uart_irq);

    /*  disable UART RX interrupts  */
    if(Local_psUart->handle->Init.Mode & UART_MODE_RX)
    {
        __HAL_UART_DISABLE_IT(Local_psUart->handle, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(Local_psUart->handle, UART_IT_PE);
        __HAL_UART_DISABLE_IT(Local_psUart->handle, UART_IT_ERR);
    }

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
    RingBuffer_Counter_t Local_u32Count = 0;
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


    // CRITICAL_SECTION_BEGIN();

    RingBuffer_enGetItems(Local_psUart->rx_buffer, pu8Data, u32Len, &Local_u32Count);

    // CRITICAL_SECTION_END();

    (*pu32Count) = Local_u32Count;

    if(Local_u32Count == 0)
    {
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

    // CRITICAL_SECTION_BEGIN();

    while((Local_u32Count < u32Len) && (RingBuffer_enGetItem(Local_psUart->rx_buffer, &Local_u8Byte) == RING_BUFFER_ERROR_NONE))
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

    // CRITICAL_SECTION_END();

    (*pu32Count) = Local_u32Count;

    if(Local_u32Count == 0)
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
    RingBuffer_Counter_t Local_u32Count = 0;
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

    // CRITICAL_SECTION_BEGIN();

    RingBuffer_enPutItems(Local_psUart->tx_buffer, pu8Data, u32Len, &Local_u32Count);

    // CRITICAL_SECTION_END();

    (*pu32Count) = Local_u32Count;

    if(Local_u32Count == 0)
    {
        return UART_ERROR_BUFFER_FULL;
    }
    else
    {
        __HAL_UART_ENABLE_IT(Local_psUart->handle, UART_IT_TXE);
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWriteBlocking(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len)
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


    HAL_UART_Transmit(Local_psUart->handle, pu8Data, (uint16_t)u32Len, HAL_MAX_DELAY);

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

    do
    {
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

    RingBuffer_enReset(Local_psUart->rx_buffer);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enUpdateChannel(UART_Channel_t enChannel)
{
    (void)enChannel;
    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

static void UART_vidIrqCallback(UART_t const * const psUart)
{
    RingBuffer_Error_t Local_enError;
    uint8_t Local_u8Byte;

    /*  check error flags   */
    if(__HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_PE)
            || __HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_NE)
            || __HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_FE)
            || __HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(psUart->handle);
        return;
    }

    /*  check TX empty flag   */
    if(__HAL_UART_GET_IT_SOURCE(psUart->handle, UART_IT_TXE) && __HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_TXE))
    {
        Local_enError = RingBuffer_enGetItem(psUart->tx_buffer, &Local_u8Byte);
        if(Local_enError == RING_BUFFER_ERROR_EMPTY)
        {
            __HAL_UART_DISABLE_IT(psUart->handle, UART_IT_TXE);
        }
        else
        {
            psUart->handle->Instance->DR = Local_u8Byte;
        }
    }

    /*  check RX not empty flag   */
    if(__HAL_UART_GET_IT_SOURCE(psUart->handle, UART_IT_RXNE) && __HAL_UART_GET_FLAG(psUart->handle, UART_FLAG_RXNE))
    {
        Local_u8Byte = (uint8_t)psUart->handle->Instance->DR;
        Local_enError = RingBuffer_enPutItem(psUart->rx_buffer, &Local_u8Byte);
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
