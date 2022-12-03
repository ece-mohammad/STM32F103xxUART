/******************************************************************************
 * @file      uart_ll_dma.c
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

#include "uart_ll_dma_conf.h"
#include "serial/uart/uart.h"


/* ------------------------------------------------------------------------- */
/* --------------------- Public Data Types Definitions --------------------- */
/* ------------------------------------------------------------------------- */

/**
 * UART DMA X/TX context
 * */
typedef struct uart_dma_context_t
{
        volatile uint32_t  dma_rx_read_pos;         /**<  UART DMA RX buffer read position  */
        volatile uint32_t dma_tx_curent_transfer;   /**<  number of bytes DMA is currently sending on UART TX pin  */

#ifdef UART_MINIMAL_INTERRUPTS
        volatile uint32_t dma_tx_complete_flag;
        volatile uint32_t dma_rx_pending_flag;
#endif /*  UART_MINIMAL_INTERRUPTS  */

} UART_DMA_Context_t;

/* -------------------------------------------------------------------------- */

/**
 * UART structure
 * */
typedef struct uart_t {
        /* UART RX buffer */
        RingBuffer_t * rx_buffer;   /**<  UART RX ring buffer, to store received bytes.
                                          Bytes are read using UART_enRead(), UART_enReadUntil(), UART_enReadLine() functions  */
        uint8_t * rx_data;          /**<  RX ring buffer data  */
        uint32_t rx_size;           /**<  RX ring buffer data size  */

        /* UART TX buffer */
        RingBuffer_t * tx_buffer;   /**<  UART TX ring buffer, stores bytes to be transmitted over UART  */
        uint8_t * tx_data;          /**<  TX ring buffer data  */
        uint32_t tx_size;           /**<  TX ring buffer data size  */

        USART_TypeDef * uart_handle;    /**<  UART handle, used to interface with UART peripheral (USART1, USART2, USART3, ...)  */

        /* UART GPIO */
        GPIO_TypeDef * gpio_port;   /**<  UART Pins' GPIO port handle, used to configure UART TX/RX pins (GPIOA< GPIOP, ...)  */
        uint32_t rx_pin;            /**<  UART RX pin number (LL_GPIO_PIN_X)  */
        uint32_t tx_pin;            /**<  UART TX pin number (LL_GPIO_PIN_x)  */

        /* UART DMA */
        DMA_TypeDef * dma_handle;   /**<  UART DMA handle, used to interface with DMA peripheral used with UART  */
        uint32_t dma_rx_channel;    /**<  UART DMA channel number associated with UART RX request  */
        uint32_t dma_tx_channel;    /**<  UART DMA channel number associated with UART TX request  */

        UART_DMA_Context_t * dma_context;   /**<  UART DMA TX/RX context  */

        /* UART DMA RX context */
        uint8_t * dma_rx_buffer;        /**<  UART DMA RX temporary buffer to store received data by DMA  */
        uint32_t  dma_rx_buffer_size;   /**<  UART DMA temporary buffer size  */

        /* UART DMA TX context */
        uint32_t dma_tx_size;       /**<  maximum number of bytes DMA sends in each transfer  */

        /* DMA RX flags (LL_DMA_IsActiveFlag_XX) */
        uint32_t (*dma_rx_is_active_flag_tc)(DMA_TypeDef *);    /**<  DMA RX Get Transmit Complete (TC) flag function (LL_DMA_IsActiveFlag_TCx())  */
        uint32_t (*dma_rx_is_active_flag_ht)(DMA_TypeDef *);    /**<  DMA RX Get Transmit Half-complete (HT) flag function (LL_DMA_IsActiveFlag_HTx())  */

        /* DMA TX flags (LL_DMA_IsActiveFlag_XX) */
        uint32_t (*dma_tx_is_active_flag_tc)(DMA_TypeDef *);    /**<  DMA TX Get Transmit Complete (TC) flag function (LL_DMA_IsActiveFlag_TCx())  */

        /* UART DMA RX clear flag (LL_DMA_ClearFLag_XX) */
        void (*dma_rx_clearflag_tc)(DMA_TypeDef *);     /**<  DMA RX clear Transmit Complete (TC) flag (LL_DMA_ClearFlag_TCx())  */
        void (*dma_rx_clearflag_ht)(DMA_TypeDef *);     /**<  DMA RX clear Transmit Half-complete (HT) flag (LL_DMA_ClearFlag_HTx())  */
        void (*dma_rx_clearflag_te)(DMA_TypeDef *);     /**<  DMA RX clear Transmit Error (TE) flag (LL_DMA_ClearFlag_TEx())  */

        /* DMA TX clear flag (LL_DMA_ClearFLag_XX) */
        void (*dma_tx_clearflag_tc)(DMA_TypeDef *);     /**<  DMA TX clear Transmit Complete (TC) flag (LL_DMA_ClearFlag_TCx())  */

        /* UART & DMA interrupts */
        IRQn_Type uart_irqn;        /**<  UART channel's IRQn (USARTx_IRQn)  */
        IRQn_Type dma_rx_irqn;      /**<  DMA RX channel IRQn (DMA_Channelx_IRQn)  */
        IRQn_Type dma_tx_irqn;      /**<  DMA TX channel IRQn (DMA_Channelx_IRQn)  */


#if 0
        volatile uint32_t  dma_rx_read_pos;      /**<  UART DMA RX buffer read position  */
        volatile uint32_t dma_tx_curent_transfer;       /**<  number of bytes DMA is currently sending on UART TX pin  */

#ifdef UART_MINIMAL_INTERRUPTS
        volatile uint32_t dma_tx_complete_flag;
        volatile uint32_t dma_rx_pending_flag;
#endif /*  UART_MINIMAL_INTERRUPTS  */

#endif

} UART_t;

/* ------------------------------------------------------------------------- */
/* --------------------- Private Variables Definitions --------------------- */
/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_1)

#ifdef UART_ENABLE_CHANNEL_1_RX
static RingBuffer_t UART_1_RxBuffer;
static uint8_t UART_Channel_1_RX_Data [UART_CHANNEL_1_RX_BUFFER_SIZE];
static uint8_t UART_DMA_Channel_1_RX_Data [UART_DMA_CHANNEL_1_RX_BUFFER_SIZE];
#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
static RingBuffer_t UART_1_TxBuffer;
static uint8_t UART_Channel_1_TX_Data [UART_CHANNEL_1_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

static UART_DMA_Context_t UART_1_DMA_Context;

static const UART_t UART_1 = {

#ifdef UART_ENABLE_CHANNEL_1_RX
        .rx_buffer = &UART_1_RxBuffer,
        .rx_data = UART_Channel_1_RX_Data,
        .rx_size = UART_CHANNEL_1_RX_BUFFER_SIZE,
#endif  /*  UART_ENABLE_CHANNEL_1_RX  */

#ifdef UART_ENABLE_CHANNEL_1_TX
        .tx_buffer = &UART_1_TxBuffer,
        .tx_data = UART_Channel_1_TX_Data,
        .tx_size = UART_CHANNEL_1_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

        .uart_handle = USART1,

        /*
         *    USART1 GPIO
         * ----+-------+-----
         * Pin | PORTx | PINx
         * ----+-------+------
         * TX  | GPIOA | Pin 9
         * RX  | GPIOA | Pin 10
         */
        .gpio_port = GPIOA,
        .tx_pin = LL_GPIO_PIN_9,
        .rx_pin = LL_GPIO_PIN_10,

        /*
         *         USART1 DMA
         * ---------+------+--------------
         * Function | DMAx | DMA_Channel_x
         * ---------+-------+--------------
         * USART TX | DMA1 | DMA_CHANNEL_4
         * USART RX | DMA1 | DMA_CHANNEL_5
         */
        .dma_handle = DMA1,
        .dma_tx_channel = LL_DMA_CHANNEL_4,
        .dma_rx_channel = LL_DMA_CHANNEL_5,

        .dma_context = &UART_1_DMA_Context,

#ifdef UART_ENABLE_CHANNEL_1_TX
        .dma_tx_size = UART_DMA_CHANNEL_1_TX_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_TX  */

#ifdef UART_ENABLE_CHANNEL_1_RX
        .dma_rx_buffer = UART_DMA_Channel_1_RX_Data,
        .dma_rx_buffer_size = UART_DMA_CHANNEL_1_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_1_RX  */

        .dma_tx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC4,

        .dma_rx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC5,
        .dma_rx_is_active_flag_ht = LL_DMA_IsActiveFlag_HT5,

        .dma_tx_clearflag_tc = LL_DMA_ClearFlag_TC4,

        .dma_rx_clearflag_tc = LL_DMA_ClearFlag_TC5,
        .dma_rx_clearflag_ht = LL_DMA_ClearFlag_HT5,
        .dma_rx_clearflag_te = LL_DMA_ClearFlag_TE5,

        .uart_irqn = USART1_IRQn,
        .dma_tx_irqn = DMA1_Channel4_IRQn,
        .dma_rx_irqn = DMA1_Channel5_IRQn,

};
#endif  /*  UART_ENABLE_CHANNEL_1   */

/* ------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_2)

#ifdef UART_ENABLE_CHANNEL_2_RX
static RingBuffer_t UART_2_RxBuffer;
static uint8_t UART_Channel_2_RX_Data [UART_CHANNEL_2_RX_BUFFER_SIZE];
static uint8_t UART_DMA_Channel_2_RX_Data [UART_DMA_CHANNEL_2_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
static RingBuffer_t UART_2_TxBuffer;
static uint8_t UART_Channel_2_TX_Data [UART_CHANNEL_2_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_T_RX  */

static UART_DMA_Context_t UART_2_DMA_Context;

static const UART_t UART_2 = {

#ifdef UART_ENABLE_CHANNEL_2_RX
        .rx_buffer = &UART_2_RxBuffer,
        .rx_data = UART_Channel_2_RX_Data,
        .rx_size = UART_CHANNEL_2_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

#ifdef UART_ENABLE_CHANNEL_2_TX
        .tx_buffer = &UART_2_TxBuffer,
        .tx_data = UART_Channel_2_TX_Data,
        .tx_size = UART_CHANNEL_2_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

        .uart_handle = USART2,

        /*
         *    USART2 GPIO
         * ----+-------+-----
         * Pin | PORTx | PINx
         * ----+-------+------
         * TX  | GPIOA | Pin 2
         * RX  | GPIOA | Pin 3
         */
        .gpio_port = GPIOA,
        .tx_pin = LL_GPIO_PIN_2,
        .rx_pin = LL_GPIO_PIN_3,

        /*
         *         USART2 DMA
         * ---------+------+--------------
         * Function | DMAx | DMA_Channel_x
         * ---------+-------+--------------
         * USART TX | DMA1 | DMA_CHANNEL_7
         * USART RX | DMA1 | DMA_CHANNEL_6
         */
        .dma_handle = DMA1,
        .dma_tx_channel = LL_DMA_CHANNEL_7,
        .dma_rx_channel = LL_DMA_CHANNEL_6,

        .dma_context = &UART_2_DMA_Context,

#ifdef UART_ENABLE_CHANNEL_2_TX
        .dma_tx_size = UART_DMA_CHANNEL_2_TX_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_TX  */

#ifdef UART_ENABLE_CHANNEL_2_RX
        .dma_rx_buffer = UART_DMA_Channel_2_RX_Data,
        .dma_rx_buffer_size = UART_DMA_CHANNEL_2_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_2_RX  */

        .dma_tx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC7,

        .dma_rx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC6,
        .dma_rx_is_active_flag_ht = LL_DMA_IsActiveFlag_HT6,

        .dma_tx_clearflag_tc = LL_DMA_ClearFlag_TC7,

        .dma_rx_clearflag_tc = LL_DMA_ClearFlag_TC6,
        .dma_rx_clearflag_ht = LL_DMA_ClearFlag_HT6,
        .dma_rx_clearflag_te = LL_DMA_ClearFlag_TE6,

        .dma_tx_irqn = DMA1_Channel7_IRQn,
        .dma_rx_irqn = DMA1_Channel6_IRQn,
        .uart_irqn = USART2_IRQn,
};
#endif  /*  UART_ENABLE_CHANNEL_2   */

#if defined(UART_ENABLE_CHANNEL_3)

#ifdef UART_ENABLE_CHANNEL_3_RX
static RingBuffer_t UART_3_RxBuffer;
static uint8_t UART_Channel_3_RX_Data [UART_CHANNEL_3_RX_BUFFER_SIZE];
static uint8_t UART_DMA_Channel_3_RX_Data [UART_DMA_CHANNEL_2_RX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
static RingBuffer_t UART_3_TxBuffer;
static uint8_t UART_Channel_3_TX_Data [UART_CHANNEL_3_TX_BUFFER_SIZE];
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

static UART_DMA_Context_t UART_3_DMA_Context;

static const UART_t UART_3 = {

#ifdef UART_ENABLE_CHANNEL_3_RX
        .rx_buffer = &UART_3_RxBuffer,
        .rx_data = UART_Channel_3_RX_Data,
        .rx_size = UART_CHANNEL_3_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

#ifdef UART_ENABLE_CHANNEL_3_TX
        .tx_buffer = &UART_3_TxBuffer,
        .tx_data = UART_Channel_3_TX_Data,
        .tx_size = UART_CHANNEL_3_TX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

        .uart_handle = USART3,

        /*
         *    USART3 GPIO
         * ----+-------+-----
         * Pin | PORTx | PINx
         * ----+-------+------
         * TX  | GPIOB | Pin 10
         * RX  | GPIOB | Pin 11
         */
        .gpio_port = GPIOB,
        .tx_pin = LL_GPIO_PIN_10,
        .rx_pin = LL_GPIO_PIN_11,

        /*
         *         USART3 DMA
         * ---------+------+--------------
         * Function | DMAx | DMA_Channel_x
         * ---------+-------+--------------
         * USART TX | DMA1 | DMA_CHANNEL_2
         * USART RX | DMA1 | DMA_CHANNEL_3
         */
        .dma_handle = DMA1,
        .dma_tx_channel = LL_DMA_CHANNEL_2,
        .dma_rx_channel = LL_DMA_CHANNEL_3,

        .dma_context = &UART_3_DMA_Context,

#ifdef UART_ENABLE_CHANNEL_3_TX
        .dma_tx_size = UART_DMA_CHANNEL_3_TX_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_TX  */

#ifdef UART_ENABLE_CHANNEL_3_RX
        .dma_rx_buffer = UART_DMA_Channel_3_RX_Data,
        .dma_rx_buffer_size = UART_DMA_CHANNEL_3_RX_BUFFER_SIZE,
#endif /*  UART_ENABLE_CHANNEL_3_RX  */

        .dma_tx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC2,

        .dma_rx_is_active_flag_tc = LL_DMA_IsActiveFlag_TC3,
        .dma_rx_is_active_flag_ht = LL_DMA_IsActiveFlag_HT3,

        .dma_tx_clearflag_tc = LL_DMA_ClearFlag_TC2,

        .dma_rx_clearflag_tc = LL_DMA_ClearFlag_TC3,
        .dma_rx_clearflag_ht = LL_DMA_ClearFlag_HT3,
        .dma_rx_clearflag_te = LL_DMA_ClearFlag_TE3,

        .uart_irqn = USART3_IRQn,
        .dma_tx_irqn = DMA1_Channel2_IRQn,
        .dma_rx_irqn = DMA1_Channel3_IRQn,
};
#endif  /*  UART_ENABLE_CHANNEL_3   */

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

/* ------------------------------------------------------------------------- */
/* --------------------- Private Functions Declarations -------------------- */
/* ------------------------------------------------------------------------- */

/**
 *
 * @brief Copy received data from UART channel's DMA RX buffer to UART channel RX buffer
 *
 * @callergraph
 *
 * @param [in] psUart  : pointer to UART chanel's UART_t
 *
 * @return void
 *
 * */
static inline void UART_DMA_vidReceiveData(const UART_t * const psUart);

/**
 * @brief Start UART channel's DMA TX transmission
 *
 * @callergraph
 *
 * @param [in] psUart  : pointer to UART chanel's UART_t
 *
 * @return void
 *
 * */
static inline void UART_DMA_vidTransmitData(const UART_t * const psUart);

/* ------------------------------------------------------------------------- */
/* --------------------- Private Functions Definitions --------------------- */
/* ------------------------------------------------------------------------- */

static inline void UART_DMA_vidReceiveData(const UART_t * const psUart)
{
    uint32_t old_read_pos = psUart->dma_context->dma_rx_read_pos;
    uint32_t current_read_pos;
    RingBuffer_Counter_t put_count;

    /*  current read position in DMA RX buffer  */
    current_read_pos = psUart->dma_rx_buffer_size - LL_DMA_GetDataLength(psUart->dma_handle, psUart->dma_rx_channel);

    /*  check current read position != old read position */
    if(current_read_pos == old_read_pos)
    {
        return;
    }

    if(current_read_pos > old_read_pos)
    {
        /**
         * if current_read_position > old_read_position :
         * - some bytes were received and current position didn't warp around in the buffer
         * - Add items between old an new read positions to UART RX ring buffer
         * */
        RingBuffer_enPutItems(
                psUart->rx_buffer,
                &psUart->dma_rx_buffer[old_read_pos],
                (current_read_pos - old_read_pos),
                &put_count
        );
    }
    else
    {
        /**
         * if current_read_position < old_read_position :
         * - some bytes were received and current position warped around in the buffer
         * - Add items between old read positions and end of DMA RX buffer
         * - Add items between current position and start of DMA RX buffer (if any exist)
         * */
        RingBuffer_enPutItems(
                psUart->rx_buffer,
                &psUart->dma_rx_buffer[old_read_pos],
                (psUart->dma_rx_buffer_size - old_read_pos),
                &put_count
        );

        if(current_read_pos > 0)
        {
            RingBuffer_enPutItems(
                    psUart->rx_buffer,
                    &psUart->dma_rx_buffer[0],
                    current_read_pos,
                    &put_count
            );
        }
    }

    /* Update DMA RX read position with current read position */
    psUart->dma_context->dma_rx_read_pos = current_read_pos;
}

/* -------------------------------------------------------------------------- */

static inline void UART_DMA_vidTransmitData(const UART_t * const psUart)
{
    RingBuffer_Counter_t Local_u32BlockSize;
    RingBuffer_Item_t * Local_pu8ReadAddress;

    /*  check that no DMA transmission is ongoing (remaining transfers == 0)  */
    if(psUart->dma_context->dma_tx_curent_transfer > 0)
    {
        /*  DMA transfer is ongoing, UART_enWrite() was called before DMA finishes its current transfer */
        return;
    }

    /*  get number of available bytes in TX buffer */
    RingBuffer_enBlockReadCount(psUart->tx_buffer, &Local_u32BlockSize);

    /**
     * - When called from UART_enWrite() (and DMA has no ongoing transfers) this condition will always be true
     * - When called from DMA_IRQHandler() (DMA just finished a transfer) this condition will always be true until TX buffer is empty
     * */
    if(Local_u32BlockSize == 0)
    {
        return;
    }

    /**
     * limit block size to a small-ish size so that the TX buffer
     * can have free space as soon as possible.
     * Since DMA TX C interrupt is responsible for skipping transfered bytes from TX buffer,
     * a large block size will take more time and TX buffer may fill up.
     * */
    Local_u32BlockSize = MIN(psUart->dma_tx_size, Local_u32BlockSize);

    /*  set DMA TX size */
    psUart->dma_context->dma_tx_curent_transfer = Local_u32BlockSize;

    /*  Disable DMA channel  */
    LL_DMA_DisableChannel(psUart->dma_handle, psUart->dma_tx_channel);

    /*  Get address from TX buffer  */
    RingBuffer_enBlockReadAddress(psUart->tx_buffer, &Local_pu8ReadAddress);

    /*  Set DMA TX number of transfers  */
    LL_DMA_SetDataLength(psUart->dma_handle, psUart->dma_tx_channel, Local_u32BlockSize);

    /*  Configure DMA memory address (source address)  */
    LL_DMA_SetMemoryAddress(psUart->dma_handle, psUart->dma_tx_channel, (uint32_t)Local_pu8ReadAddress);

    /* Enable DMA channel */
    LL_DMA_EnableChannel(psUart->dma_handle, psUart->dma_tx_channel);
}


/* ------------------------------------------------------------------------- */
/* ---------------------- Public Functions Definitions --------------------- */
/* ------------------------------------------------------------------------- */

UART_Error_t UART_enInitialize(const UART_Channel_t enChannel, UART_Conf_t const * const psConf)
{
    UART_t * Local_psUart = NULL;
    RingBuffer_Error_t Local_enBufferError;

#ifdef DEBUG

    if(IS_NULLPTR(psConf))
    {
        return UART_ERROR_NULLPTR;
    }
    else
    {
        (void)0;
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
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }
#endif  /*  UART_ENABLE_CHANNEL_1   */

#if defined(UART_ENABLE_CHANNEL_2)
    if(enChannel == UART_CHANNEL_2)
    {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }
#endif  /*  UART_ENABLE_CHANNEL_2   */

#if defined(UART_ENABLE_CHANNEL_3)
    if(enChannel == UART_CHANNEL_3)
    {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }
#endif  /*  UART_ENABLE_CHANNEL_3   */

    /*  initialize UART TX GPIO pins, TX buffer and DMA TX channel  */
    if(Local_psUart->tx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->tx_buffer, Local_psUart->tx_data, Local_psUart->tx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        /*
         * initialize DMA for UART TX
         * DMA direction = memory to peripheral
         * mode = normal
         * priority = normal
         *
         * memory address   (src) = 0
         * memory data size (src) = byte
         * memory inc mode  (src) = increment
         *
         * peripheral address   (dst) = USARTx->DR
         * peripheral data size (dst) = byte
         * peripheral inc mode  (dst) = no increment
         *
         * enable DMA TC (transfer complete) interrupt
         */
        LL_DMA_ConfigTransfer(
                /* DMAx */              Local_psUart->dma_handle,
                /* DMA channel */       Local_psUart->dma_tx_channel,
                /* direction */         LL_DMA_DIRECTION_MEMORY_TO_PERIPH   | \
                /* mode */              LL_DMA_MODE_NORMAL                  | \
                /* priority */          LL_DMA_PRIORITY_MEDIUM              | \
                /* memory size */       LL_DMA_MDATAALIGN_BYTE              | \
                /* memory increment */  LL_DMA_MEMORY_INCREMENT             | \
                /* per. size */         LL_DMA_PDATAALIGN_BYTE              | \
                /* per. increment */    LL_DMA_PERIPH_NOINCREMENT
        );

        LL_DMA_SetPeriphAddress(Local_psUart->dma_handle, Local_psUart->dma_tx_channel, LL_USART_DMA_GetRegAddr(Local_psUart->uart_handle));

        /*  enable DMA TC interrupt  */
        LL_DMA_EnableIT_TC(Local_psUart->dma_handle, Local_psUart->dma_tx_channel);

        NVIC_SetPriority(Local_psUart->dma_tx_irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY));
        NVIC_EnableIRQ(Local_psUart->dma_tx_irqn);
    }

    /*  initialize UART RX GPIO pins, RX buffer and DMA RX channel  */
    if(Local_psUart->rx_size)
    {
        Local_enBufferError = RingBuffer_enInit(Local_psUart->rx_buffer, Local_psUart->rx_data, Local_psUart->rx_size);
        assert_param(Local_enBufferError == RING_BUFFER_ERROR_NONE);

        LL_GPIO_SetPinMode(Local_psUart->gpio_port, Local_psUart->rx_pin, LL_GPIO_MODE_INPUT);
        LL_GPIO_SetPinPull(Local_psUart->gpio_port, Local_psUart->rx_pin, LL_GPIO_PULL_UP);

        Local_psUart->dma_context->dma_rx_read_pos = 0;

        /*
         * initialize DMA for UART RX
         * DMA direction = peripheral to memory
         * mode = circular
         * priority = normal
         *
         * memory address   (src) = psUart->dma_rx_buffer
         * memory data size (src) = byte
         * memory inc mode  (src) = increment
         *
         * peripheral address   (dst) = USARTx->DR
         * peripheral data size (dst) = byte
         * peripheral inc mode  (dst) = no increment
         *
         * enable DMA TC (transfer complete) interrupt
         * enable DMA TC (half transfer) interrupt
         */
        LL_DMA_ConfigTransfer(
                /* DMAx */              Local_psUart->dma_handle,
                /* DMA channel */       Local_psUart->dma_rx_channel,
                /* direction */         LL_DMA_DIRECTION_PERIPH_TO_MEMORY   | \
                /* mode */              LL_DMA_MODE_CIRCULAR                | \
                /* priority */          LL_DMA_PRIORITY_MEDIUM              | \
                /* memory size */       LL_DMA_MDATAALIGN_BYTE              | \
                /* memory increment */  LL_DMA_MEMORY_INCREMENT             | \
                /* per. size */         LL_DMA_PDATAALIGN_BYTE              | \
                /* per. increment */    LL_DMA_PERIPH_NOINCREMENT
        );

        LL_DMA_SetMemoryAddress(Local_psUart->dma_handle, Local_psUart->dma_rx_channel, (uint32_t)Local_psUart->dma_rx_buffer);
        LL_DMA_SetPeriphAddress(Local_psUart->dma_handle, Local_psUart->dma_rx_channel, LL_USART_DMA_GetRegAddr(Local_psUart->uart_handle));
        LL_DMA_SetDataLength(Local_psUart->dma_handle, Local_psUart->dma_rx_channel, Local_psUart->dma_rx_buffer_size);

        LL_DMA_EnableIT_HT(Local_psUart->dma_handle, Local_psUart->dma_rx_channel);
        LL_DMA_EnableIT_TC(Local_psUart->dma_handle, Local_psUart->dma_rx_channel);

        NVIC_SetPriority(Local_psUart->dma_rx_irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY));
        NVIC_EnableIRQ(Local_psUart->dma_rx_irqn);

        LL_DMA_EnableChannel(Local_psUart->dma_handle, Local_psUart->dma_rx_channel);
    }

    if((psConf->TransferDirection & LL_USART_DIRECTION_TX) == LL_USART_DIRECTION_TX)
    {
        LL_GPIO_SetPinMode(Local_psUart->gpio_port, Local_psUart->tx_pin, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinOutputType(Local_psUart->gpio_port, Local_psUart->tx_pin, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinSpeed(Local_psUart->gpio_port, Local_psUart->tx_pin, LL_GPIO_SPEED_FREQ_HIGH);
    }

    /*  Initialize UART channel  */
    if(LL_USART_Init(Local_psUart->uart_handle, (LL_USART_InitTypeDef *)psConf) != SUCCESS)
    {
        LL_USART_DeInit(Local_psUart->uart_handle);
        return UART_ERROR_NOT_INIT;
    }

    LL_USART_ConfigAsyncMode(Local_psUart->uart_handle);
    LL_USART_Enable(Local_psUart->uart_handle);

    /*  enable USART DMA TX request  */
    if((psConf->TransferDirection & LL_USART_DIRECTION_TX) == LL_USART_DIRECTION_TX)
    {
        LL_USART_EnableDMAReq_TX(Local_psUart->uart_handle);
    }

    /*  enable USART DMA RX request and IDLE interrupt  */
    if((psConf->TransferDirection & LL_USART_DIRECTION_RX) == LL_USART_DIRECTION_RX)
    {
        LL_USART_EnableIT_IDLE(Local_psUart->uart_handle);
        LL_USART_EnableDMAReq_RX(Local_psUart->uart_handle);
    }

    NVIC_SetPriority(Local_psUart->uart_irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PREEMPTION_PRIORITY, UART_INTERRUPT_GROUPING_PRIORITY));
    NVIC_EnableIRQ(Local_psUart->uart_irqn);

    (void)Local_enBufferError;

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

    NVIC_DisableIRQ(Local_psUart->uart_irqn);

    /*  disable UART RX interrupts  */
    if((LL_USART_GetTransferDirection(Local_psUart->uart_handle) & LL_USART_DIRECTION_RX) == LL_USART_DIRECTION_RX)
    {
        LL_USART_DisableIT_RXNE(Local_psUart->uart_handle);
        LL_USART_DisableIT_PE(Local_psUart->uart_handle);
        LL_USART_DisableIT_ERROR(Local_psUart->uart_handle);
    }

    /*  reset RX/TX buffers & DMA TX/RX channels */
    if(Local_psUart->tx_size)
    {
        RingBuffer_enReset(Local_psUart->tx_buffer);
        LL_DMA_DeInit(Local_psUart->dma_handle, Local_psUart->dma_tx_channel);
    }

    if(Local_psUart->rx_size)
    {
        RingBuffer_enReset(Local_psUart->rx_buffer);
        LL_DMA_DeInit(Local_psUart->dma_handle, Local_psUart->dma_rx_channel);
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

    /*  read bytes from rx buffer  */
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

UART_Error_t UART_enReadLine(const UART_Channel_t enChannel, uint8_t * const pu8Data, uint32_t u32Len, uint32_t * const pu32Count)
{
    return UART_enReadUntil(enChannel, pu8Data, u32Len, '\n', pu32Count);
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

    Local_psUart = (UART_t*)UART_asHandles[enChannel];

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


    /*  start UART TX  */
    UART_DMA_vidTransmitData(Local_psUart);

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

UART_Error_t UART_enWriteBlocking(const UART_Channel_t enChannel, uint8_t const * const pu8Data, uint32_t u32Len)
{
    UART_t * Local_psUart = NULL;
    uint8_t * Local_pu8Data = (uint8_t *)pu8Data;

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

    Local_psUart = (UART_t*)UART_asHandles[enChannel];

#endif /*  DEBUG  */

    while(u32Len--)
    {
        LL_USART_TransmitData8(Local_psUart->uart_handle, *Local_pu8Data++);

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
    else
    {
        (void)0;
    }

#else

    Local_psUart = UART_asHandles[enChannel];

#endif /*  DEBUG  */

    do
    {
        UART_enUpdateChannel(enChannel);

        /*  wait TX buffer to be empty  */
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

#ifdef UART_MINIMAL_INTERRUPTS

    UART_t * Local_psUart = NULL;
    RingBuffer_Counter_t Local_u32SkipCount;

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

    Local_psUart = (UART_t*)UART_asHandles[enChannel];

#endif /*  DEBUG  */


    if(Local_psUart->tx_size && Local_psUart->dma_context->dma_tx_complete_flag)
    {
        Local_psUart->dma_context->dma_tx_complete_flag = 0;

        /*  Skip DMA TX size from TX buffer (those items were transfered to UART DR already)  */
        RingBuffer_enSkipItems(Local_psUart->tx_buffer, Local_psUart->dma_context->dma_tx_curent_transfer, &Local_u32SkipCount);

        /*  Reset DMA TX size  */
        Local_psUart->dma_context->dma_tx_curent_transfer = 0;

        UART_DMA_vidTransmitData(Local_psUart);
    }

    if(Local_psUart->rx_size && Local_psUart->dma_context->dma_rx_pending_flag)
    {
        Local_psUart->dma_context->dma_rx_pending_flag = 0;
        UART_DMA_vidReceiveData(Local_psUart);
    }

#else

    (void)enChannel;

#endif

    return UART_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */

static void UART_vidIrqCallback(const UART_t * const psUart)
{
    RingBuffer_Error_t Local_enBufferError;
    uint8_t Local_u8Byte;
    uint32_t Local_u32Satus = psUart->uart_handle->SR;

#if 0

    if ((LL_USART_IsEnabledIT_ERROR(psUart->uart_handle) && ((Local_u32Satus & USART_SR_ORE) == USART_SR_ORE)) ||
        (LL_USART_IsEnabledIT_PE(psUart->uart_handle) && ((Local_u32Satus & USART_SR_PE) == USART_SR_PE)))
    {

    }

#endif /* 0 */

    /*  check error flags   */
    if((LL_USART_IsEnabledIT_ERROR(psUart->uart_handle) ||
            LL_USART_IsEnabledIT_PE(psUart->uart_handle)) &&
            (LL_USART_IsActiveFlag_ORE(psUart->uart_handle) ||
                    LL_USART_IsActiveFlag_PE(psUart->uart_handle))
    )
    {
        LL_USART_ClearFlag_ORE(psUart->uart_handle);
        return;
    }

    /*  check IDLE flag  */
    if(LL_USART_IsEnabledIT_IDLE(psUart->uart_handle) && ((Local_u32Satus & USART_SR_IDLE) == USART_SR_IDLE))
    {
        LL_USART_ClearFlag_IDLE(psUart->uart_handle);

#ifdef UART_MINIMAL_INTERRUPTS
        psUart->dma_context->dma_rx_pending_flag = 1;
#else
        UART_DMA_vidReceiveData(psUart);
#endif /*  UART_MINIMAL_INTERRUPTS  */
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

static void UART_DMA_TX_IRQHandler(const UART_t * const psUart)
{
    /**
     * Check if TC interrupt is enabled (it should be enabled in UART_vidInitialize() right after DMA TX channel initialization)
     * and DMA TC flag is active
     * */
    if(LL_DMA_IsEnabledIT_TC(psUart->dma_handle, psUart->dma_tx_channel) &&
            psUart->dma_tx_is_active_flag_tc(psUart->dma_handle))
    {
        /* clear DMA TC flag  */
        psUart->dma_tx_clearflag_tc(psUart->dma_handle);

#ifdef UART_MINIMAL_INTERRUPTS
        psUart->dma_context->dma_tx_complete_flag = 1;
#else
        RingBuffer_Counter_t Local_u32SkipCount;

        /*  Skip DMA TX size from TX buffer (those items were transfered to UART DR already)  */
        RingBuffer_enSkipItems(psUart->tx_buffer, psUart->dma_context->dma_tx_curent_transfer, &Local_u32SkipCount);

        /*  Reset DMA TX size  */
        psUart->dma_context->dma_tx_curent_transfer = 0;

        /*  Send more data from TX buffer (if there are any)  */
        UART_DMA_vidTransmitData(psUart);
#endif /* UART_MINIMAL_INTERRUPTS  */
    }
}

/* -------------------------------------------------------------------------- */

static void UART_DMA_RX_IRQHandler(const UART_t * const psUart)
{
    if(LL_DMA_IsEnabledIT_HT(psUart->dma_handle, psUart->dma_rx_channel) &&
            psUart->dma_rx_is_active_flag_ht(psUart->dma_handle))
    {
        psUart->dma_rx_clearflag_ht(psUart->dma_handle);

#ifdef UART_MINIMAL_INTERRUPTS
        psUart->dma_context->dma_rx_pending_flag = 1;
#else
        UART_DMA_vidReceiveData(psUart);
#endif /*  UART_MINIMAL_INTERRUPTS  */
    }

    if(LL_DMA_IsEnabledIT_TC(psUart->dma_handle, psUart->dma_rx_channel) &&
            psUart->dma_rx_is_active_flag_tc(psUart->dma_handle))
    {
        psUart->dma_rx_clearflag_tc(psUart->dma_handle);

#ifdef UART_MINIMAL_INTERRUPTS
        psUart->dma_context->dma_rx_pending_flag = 1;
#else
        UART_DMA_vidReceiveData(psUart);
#endif /*  UART_MINIMAL_INTERRUPTS  */
    }
}

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_1)

void USART1_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_1);
}

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_1_TX

void DMA1_Channel4_IRQHandler(void)
{
    UART_DMA_TX_IRQHandler(&UART_1);
}

#endif  /* UART_ENABLE_CHANNEL_1_TX  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_1_RX

void DMA1_Channel5_IRQHandler(void)
{
    UART_DMA_RX_IRQHandler(&UART_1);
}

#endif  /* UART_ENABLE_CHANNEL_1_RX  */

/* -------------------------------------------------------------------------- */

#endif /*  UART_ENABLE_CHANNEL_1  */

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_2)

void USART2_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_2);
}

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_2_RX

void DMA1_Channel6_IRQHandler(void)
{
    UART_DMA_RX_IRQHandler(&UART_2);
}
#endif  /*  UART_ENABLE_CHANNEL_2_RX  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_2_TX

void DMA1_Channel7_IRQHandler(void)
{
    UART_DMA_TX_IRQHandler(&UART_2);
}

#endif  /*  UART_ENABLE_CHANNEL_2_TX  */

/* -------------------------------------------------------------------------- */

#endif  /*   UART_ENABLE_CHANNEL_2  */

/* -------------------------------------------------------------------------- */

#if defined(UART_ENABLE_CHANNEL_3)

void USART3_IRQHandler(void)
{
    UART_vidIrqCallback(&UART_3);
}

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_3_TX

void DMA1_Channel2_IRQHandler(void)
{
    UART_DMA_TX_IRQHandler(&UART_3);
}

#endif  /*  UART_ENABLE_CHANNEL_3_TX  */

/* -------------------------------------------------------------------------- */

#ifdef UART_ENABLE_CHANNEL_3_RX

void DMA1_Channel3_IRQHandler(void)
{
    UART_DMA_RX_IRQHandler(&UART_3);
}

#endif  /*  UART_ENABLE_CHANNEL_3_RX  */

/* -------------------------------------------------------------------------- */

#endif /*   UART_ENABLE_CHANNEL_3  */

/* -------------------------------------------------------------------------- */

