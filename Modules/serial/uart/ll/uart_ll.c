/******************************************************************************
 * @file      uart_ll.c
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
        USART_TypeDef * uart_handle;    /**<  UART handle, used by ST LL drivers to interface with UART  */

        GPIO_TypeDef * gpio_port;       /**<  pointer to UART TX/RX pins GPIO port, to initialize UART TX/RX gpio pins  */
        uint32_t rx_pin;                /**<  UART RX gpio pin  */
        uint32_t tx_pin;                /**<  UART TX gpio pin  */

        RingBuffer_t * rx_buffer;       /**<  pointer to UART RX buffer  */
        RingBuffer_t * tx_buffer;       /**<  pointer to UART TX buffer  */

        uint8_t * rx;                   /**<  pointer to UART RX buffer array  */
        uint8_t * tx;                   /**<  pointer to UART TX buffer array  */

        uint32_t rx_size;               /**<  size of UART RX buffer  */
        uint32_t tx_size;               /**<  size of UART TX buffer  */

        IRQn_Type uart_irq;             /**<  UART IRQn (IRQ number), used to enable/disable UART interrupts in NVIC  */
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

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR((Local_psUart = (UART_t *)UART_asHandles[enChannel])))
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

#if !defined(UART_MINIMAL_INTERRUPTS)

    /*  enable UART RX/Error interrupts if RX mode is enabled  */
    if((psConf->TransferDirection & LL_USART_DIRECTION_RX) == LL_USART_DIRECTION_RX)
    {
        LL_USART_EnableIT_RXNE(Local_psUart->uart_handle);
        LL_USART_EnableIT_PE(Local_psUart->uart_handle);
        LL_USART_EnableIT_ERROR(Local_psUart->uart_handle);
    }

    NVIC_SetPriority(Local_psUart->uart_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY));
    NVIC_EnableIRQ(Local_psUart->uart_irq);

#endif /* !defined(UART_MINIMAL_INTERRUPTS) */

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enDeInitialize(const UART_Channel_t enChannel)
{
    UART_t * Local_psUart = NULL;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR((Local_psUart = (UART_t *)UART_asHandles[enChannel])))
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
    RingBuffer_Error_t Local_enBufferError = RING_BUFFER_ERROR_NONE;

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

    /*  read bytes from rx buffer  */
    while((Local_u32Count < u32Len) && ((Local_enBufferError = RingBuffer_enGetItem(Local_psUart->rx_buffer, &Local_u8Byte)) == RING_BUFFER_ERROR_NONE))
    {
        pu8Data[Local_u32Count++] = Local_u8Byte;

        if(Local_u8Byte == u8Until)
        {
            break;
        }
    }

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
    uint8_t * Local_pu8DataPtr = (uint8_t *)pu8Data;

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

#if defined(UART_MINIMAL_INTERRUPTS)

    UART_t * Local_psUart = NULL;
    uint32_t Local_u32Status;
    uint8_t Local_u8Byte;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if((enChannel >= UART_CHANNEL_COUNT) || IS_NULLPTR(Local_psUart = (UART_t*)UART_asHandles[enChannel]))
    {
        return UART_ERROR_INVALID_CHANNEL;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    /**
     * Handle UART error handling:
     * - Error flags (in USARTx->SR) are cleared by: 
     *   1- reading UARTx->SR register, followed by
     *   2- reading USARTx->DR register
     * */
    Local_u32Status = Local_psUart->uart_handle->SR;

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
    if((Local_u32Status & USART_SR_RXNE) == USART_SR_RXNE)
    {
        Local_u8Byte = LL_USART_ReceiveData8(Local_psUart->uart_handle);
        if((Local_u32Status & USART_SR_PE) != USART_SR_PE)
        {
            RingBuffer_enPutItem(Local_psUart->rx_buffer, &Local_u8Byte);
        }
    }

    /**
     * Handle UART TX
     * 
     * If UART->TXE flag is set (TXE flag is set when UART TX is enabled, and after a byte is transmitted from UART->DR):
     *   - If UART->tx_buffer has data bytes (not empty):
     *     - ready byte from UART->tx_buffer and send it (writing to DR clears TXE flag)
     * */
    if((Local_u32Status & USART_SR_TXE) == USART_SR_TXE)
    {
        Local_enBufferError = RingBuffer_enGetItem(Local_psUart->tx_buffer, &Local_u8Byte);
        if(Local_enBufferError == RING_BUFFER_ERROR_NONE)
        {
            LL_USART_TransmitData8(Local_psUart->uart_handle, Local_u8Byte);
        }
    }

#else

    (void)enChannel;

#endif /* defined(UART_MINIMAL_INTERRUPTS) */

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

static void UART_vidIrqCallback(const UART_t * const psUart)
{
    uint32_t Local_u32Status;
    uint8_t Local_u8Byte;
    RingBuffer_Error_t Local_enBufferError;

    /**
     * Handle UART error handling:
     * - Error flags (in USARTx->SR) are cleared by: 
     *   1- reading UARTx->SR register, followed by
     *   2- reading USARTx->DR register
     * */
    Local_u32Status = psUart->uart_handle->SR;

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
    if(LL_USART_IsEnabledIT_RXNE(psUart->uart_handle) && ((Local_u32Status & USART_SR_RXNE) == USART_SR_RXNE))
    {
        Local_u8Byte = LL_USART_ReceiveData8(psUart->uart_handle);
        if((Local_u32Status & USART_SR_PE) != USART_SR_PE)
        {
            RingBuffer_enPutItem(psUart->rx_buffer, &Local_u8Byte);
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
    if(LL_USART_IsEnabledIT_TXE(psUart->uart_handle) && ((Local_u32Status & USART_SR_TXE) == USART_SR_TXE))
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

