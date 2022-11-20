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

#include "utils/utils.h"
#include "buffer/ring_buffer.h"

#include "uart_ll_conf.h"
#include "serial/uart/uart.h"


/* -------------------------------------------------------------------------- */

/**
 * UART structure
 * */
typedef struct {
        USART_TypeDef * uart_handle;    /**<    */
        GPIO_TypeDef * gpio_port;       /**<    */
        uint32_t rx_pin;                /**<    */
        uint32_t tx_pin;                /**<    */

        RingBuffer_t * rx_buffer;       /**<    */
        RingBuffer_t * tx_buffer;       /**<    */

        uint8_t * rx;                   /**<    */
        uint8_t * tx;                   /**<    */

        uint32_t rx_size;               /**<    */
        uint32_t tx_size;               /**<    */
                                        /**<    */
        IRQn_Type uart_irq;             /**<    */
} UART_t;

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_1)

#ifdef UART_ENABLE_CHANNEL_1_RX
static RingBuffer_t UART_Channel_1_RxBuffer;
static uint8_t UART_Channel_1_RX_Data [UART_CHANNEL_1_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
static RingBuffer_t UART_Channel_1_TxBuffer;
static uint8_t UART_Channel_1_TX_Data [UART_CHANNEL_1_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

static const UART_t UART_1 = {

        .uart_handle = USART1,

        .gpio_port = GPIOA,
        .tx_pin = LL_GPIO_PIN_9,
        .rx_pin = LL_GPIO_PIN_10,

#ifdef UART_ENABLE_CHANNEL_1_RX
        .rx_buffer = &UART_Channel_1_RxBuffer,
        .rx = UART_Channel_1_RX_Data,
        .rx_size = UART_CHANNEL_1_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
        .tx_buffer = &UART_Channel_1_TxBuffer,
        .tx = UART_Channel_1_TX_Data,
        .tx_size = UART_CHANNEL_1_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

        .uart_irq = USART1_IRQn,

};

#endif  /*  UART_ENABLE_CHANNEL_1   */

/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_2)

#ifdef UART_ENABLE_CHANNEL_2_RX
static RingBuffer_t UART_Channel_2_RxBuffer;
static uint8_t UART_Channel_2_RX_Data [UART_CHANNEL_2_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
static RingBuffer_t UART_Channel_2_TxBuffer;
static uint8_t UART_Channel_2_TX_Data [UART_CHANNEL_2_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

static const UART_t UART_2 = {

        .uart_handle = USART2,

        .gpio_port = GPIOA,
        .tx_pin = LL_GPIO_PIN_2,
        .rx_pin = LL_GPIO_PIN_3,

#ifdef UART_ENABLE_CHANNEL_2_RX
        .rx_buffer = &UART_Channel_2_RxBuffer,
        .rx = UART_Channel_2_RX_Data,
        .rx_size = UART_CHANNEL_2_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
        .tx_buffer = &UART_Channel_2_TxBuffer,
        .tx = UART_Channel_2_TX_Data,
        .tx_size = UART_CHANNEL_2_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

        .uart_irq = USART2_IRQn
};

#endif  /*  UART_ENABLE_CHANNEL_2   */

/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_3)

#ifdef UART_ENABLE_CHANNEL_3_RX
static RingBuffer_t UART_Channel_3_RxBuffer;
static uint8_t UART_Channel_3_RX_Data [UART_CHANNEL_3_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
static RingBuffer_t UART_Channel_3_TxBuffer;
static uint8_t UART_Channel_3_TX_Data [UART_CHANNEL_3_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

static const UART_t UART_3 = {

        .uart_handle = USART3,

        .gpio_port = GPIOB,
        .tx_pin = LL_GPIO_PIN_10,
        .rx_pin = LL_GPIO_PIN_11,

#ifdef UART_ENABLE_CHANNEL_3_RX
        .rx_buffer = &UART_Channel_3_RxBuffer,
        .rx = UART_Channel_3_RX_Data,
        .rx_size = UART_CHANNEL_3_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
        .tx_buffer = &UART_Channel_3_TxBuffer,
        .tx = UART_Channel_3_TX_Data,
        .tx_size = UART_CHANNEL_3_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

        .uart_irq = USART3_IRQn,

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
    LL_GPIO_InitTypeDef Local_sGpioConf = {0};
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if(IS_NULLPTR(psConf))
    {
        return UART_ERROR_NULLPTR;
    }

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR((Local_psUart = UART_asHandles[enChannel])))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /*  initialize channel's UART_t structure & enable its NVIC interrupt   */
#if defined(UART_ENABLE_CHANNEL_1)
    if(enChannel == UART_CHANNEL_1)
    {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    }
#endif  /*  UART_ENABLE_CHANNEL_1   */

#if defined(UART_ENABLE_CHANNEL_2)
    if(enChannel == UART_CHANNEL_2)
    {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    }
#endif  /*  UART_ENABLE_CHANNEL_2   */

#if defined(UART_ENABLE_CHANNEL_3)
    if(enChannel == UART_CHANNEL_3)
    {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    }
#endif  /*  UART_ENABLE_CHANNEL_3   */

    /*  initialize UART GPIO pins and RX/TX buffers  */
    if(Local_psUart->rx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->rx_buffer, Local_psUart->rx, Local_psUart->rx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioConf.Pin = Local_psUart->rx_pin;
        Local_sGpioConf.Mode = LL_GPIO_MODE_INPUT;
        Local_sGpioConf.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        LL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioConf);
    }

    if(Local_psUart->tx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->tx_buffer, Local_psUart->tx, Local_psUart->tx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        Local_sGpioConf.Pin = Local_psUart->tx_pin;
        Local_sGpioConf.Mode = LL_GPIO_MODE_ALTERNATE;
        Local_sGpioConf.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        Local_sGpioConf.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        LL_GPIO_Init(Local_psUart->gpio_port, &Local_sGpioConf);
    }

    /*  Initialize UART channel  */
    if(!LL_USART_Init(Local_psUart->uart_handle, (LL_USART_InitTypeDef *)psConf) == SUCCESS)
    {
        LL_USART_DeInit(Local_psUart->uart_handle);
        return UART_ERROR_NOT_INIT;
    }

    LL_USART_ConfigAsyncMode(Local_psUart->uart_handle);
    LL_USART_Enable(Local_psUart->uart_handle);

    /*  enable UART RX/Error interrupts if RX mode is enabled  */
    if((psConf->TransferDirection & LL_USART_DIRECTION_RX) == LL_USART_DIRECTION_RX)
    {
        LL_USART_EnableIT_RXNE(Local_psUart->uart_handle);
        LL_USART_EnableIT_PE(Local_psUart->uart_handle);
        LL_USART_EnableIT_ERROR(Local_psUart->uart_handle);
    }

    NVIC_SetPriority(Local_psUart->uart_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY));
    NVIC_EnableIRQ(Local_psUart->uart_irq);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enDeInitialize(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR((Local_psUart = UART_asHandles[enChannel])))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    NVIC_DisableIRQ(Local_psUart->uart_irq);

    /*  disable UART RX interrupts  */
    if((LL_USART_GetTransferDirection(Local_psUart->uart_handle) & LL_USART_DIRECTION_RX) == LL_USART_DIRECTION_RX)
    {
        LL_USART_DisableIT_RXNE(Local_psUart->uart_handle);
        LL_USART_DisableIT_PE(Local_psUart->uart_handle);
        LL_USART_DisableIT_ERROR(Local_psUart->uart_handle);
    }

    /*  reset RX/TX buffers */
    if(Local_psUart->tx_size)
    {
        RingBuffer_enReset(Local_psUart->tx_buffer);
    }

    if(Local_psUart->rx_size)
    {
        RingBuffer_enReset(Local_psUart->rx_buffer);
    }

    /*  de-initialize UART */
    if(LL_USART_DeInit(Local_psUart->uart_handle) != SUCCESS)
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
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
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

    /*  get bytes from rx buffer  */
    Local_enBufferError = RingBuffer_enGetItems(Local_psUart->rx_buffer, pu8Data, u32Len, &Local_u32Count);

    (*pu32Count) = Local_u32Count;

    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }
    else if(Local_enBufferError == RING_BUFFER_ERROR_EMPTY)
    {
        return UART_ERROR_BUFFER_EMPTY;
    }
    else
    {
        return UART_ERROR_NONE;
    }
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enReadUntil(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint8_t u8Until, uint32_t * const pu32Count)
{
    uint32_t Local_u32Count = 0;
    UART_t * Local_psUart = NULL;
    uint8_t Local_u8Byte = 0;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
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

    /*  read bytes from rx buffer  */
    while((Local_u32Count < u32Len) && ((Local_enBufferError = RingBuffer_enGetItem(Local_psUart->rx_buffer, &Local_u8Byte)) == RING_BUFFER_ERROR_NONE))
    {
        pu8Data[Local_u32Count++] = Local_u8Byte;

        if(Local_u8Byte == u8Until)
        {
            break;
        }
    }

    // CRITICAL_SECTION_END();

    (*pu32Count) = Local_u32Count;

    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }
    else if(Local_u32Count == 0)
    {
        return UART_ERROR_BUFFER_EMPTY;
    }
    else
    {
        return UART_ERROR_NONE;
    }
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enReadLine(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u16Len, uint32_t * const pu16Count)
{
    return UART_enReadUntil(enChannel, pu8Data, u16Len, '\n', pu16Count);
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWrite(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count)
{
    RingBuffer_Counter_t Local_u32Count = 0;
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
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

    /*  put bytes into tx queue  */
    Local_enBufferError = RingBuffer_enPutItems(Local_psUart->tx_buffer, pu8Data, u32Len, &Local_u32Count);

    (*pu32Count) = Local_u32Count;

    if(Local_enBufferError == RING_BUFFER_ERROR_NULLPTR)
    {
        return UART_ERROR_NOT_INIT;
    }
    else if(Local_enBufferError == RING_BUFFER_ERROR_FULL)
    {
        return UART_ERROR_BUFFER_FULL;
    }

    if(!LL_USART_IsEnabledIT_TXE(Local_psUart->uart_handle))
    {
        LL_USART_EnableIT_TXE(Local_psUart->uart_handle);
    }

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWriteBlocking(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len)
{
    UART_t * Local_psUart = NULL;
    uint8_t * Local_pu8DataPtr = pu8Data;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
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


    while(u32Len--)
    {
        LL_USART_TransmitData8(Local_psUart->uart_handle, *Local_pu8DataPtr++);

        while(LL_USART_IsActiveFlag_TXE(Local_psUart->uart_handle) == 0)
        {
        }
    }

    while(LL_USART_IsActiveFlag_TC(Local_psUart->uart_handle) == 0)
    {
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

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */


    /*  check TXE interrupt is enabled  */
    if(!LL_USART_IsEnabledIT_TXE(Local_psUart->uart_handle))
    {
        LL_USART_EnableIT_TXE(Local_psUart->uart_handle);
    }

    do
    {
        /*  wait TX buffer to be empty  */
        RingBuffer_enItemCount(Local_psUart->tx_buffer, &Local_u32RemainingBytes);
    }
    while(Local_u32RemainingBytes > 0);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enFlushRx(const UART_Channel_t enChannel)
{
    (void)enChannel;
    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enUpdateChannel(UART_Channel_t enChannel)
{
    (void)enChannel;
    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

static void UART_vidIrqCallback(const UART_t * const psUart)
{
    RingBuffer_Error_t Local_enBufferError;
    uint8_t Local_u8Byte;

    /*  check error flags   */
    if ((LL_USART_IsEnabledIT_ERROR(psUart->uart_handle) && LL_USART_IsActiveFlag_ORE(psUart->uart_handle))
            || (LL_USART_IsEnabledIT_PE(psUart->uart_handle) && LL_USART_IsActiveFlag_PE(psUart->uart_handle)))
    {
        LL_USART_ClearFlag_ORE(psUart->uart_handle);
        return;
    }

    /*  check TX empty flag   */
    if(LL_USART_IsEnabledIT_TXE(psUart->uart_handle) && LL_USART_IsActiveFlag_TXE(psUart->uart_handle))
    {
        Local_enBufferError = RingBuffer_enGetItem(psUart->tx_buffer, &Local_u8Byte);
        if(Local_enBufferError == RING_BUFFER_ERROR_NONE)
        {
            LL_USART_TransmitData8(psUart->uart_handle, Local_u8Byte);
        }
        else
        {
            LL_USART_DisableIT_TXE(psUart->uart_handle);
        }
    }

    /*  check RX not empty flag   */
    if(LL_USART_IsEnabledIT_RXNE(psUart->uart_handle) && LL_USART_IsActiveFlag_RXNE(psUart->uart_handle))
    {
        Local_u8Byte = LL_USART_ReceiveData8(psUart->uart_handle);
        RingBuffer_enPutItem(psUart->rx_buffer, &Local_u8Byte);
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

