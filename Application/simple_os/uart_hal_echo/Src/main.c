/******************************************************************************
 * @file        main.c 
 * @brief       Echo received data on UART channels, using 
 *              UART HAL (ST HAL drivers) implementation 
 *              (Modules/serial/uart/hal/uart.c)
 * @details     Echo back received bytes on UART1, UART2 and UART3.
 *              MCU configurations:
 *              <ul>
 * 
 *                <li>System Clokc: 36 MHz</li>
 * 
 *                <li>UARTx (1, 2, 3) baudrate: up to 4800 bps, configurable 
 *                    using @ref UART_BAUDRATE</li>
 * 
 *                <li>UART_MINIMAL_INTERRUPTS is enabled</li>
 * 
 *                <li>SimpleOS is configured with 1000 Hz ticxk rate, 
 *                    and task count = 2</li>
 * 
 *                <li>TASK_vidUartUpdate() is configured with priority 0 (highest)
 *                    and period of 1 milli-seconds (1 ticks)</li>
 * 
 *                <li>TASK_vidUartLoopback() is configured with priority 1 (lowest)
 *                    and period of 100 milli-seconds (100 ticks)</li>
 * 
 *                <li>Enabled interrupts: 
 *                    <ul>
 *                      <li>SysTick interrupt, every 1 milli-second to update
 *                          simple os tick.</li>
 *                    </ul>
 *                </li>
 *   
 *                <li>TASK_vidUartLoopback() is called with period of 100 ms, so data is 
 *                queued to be sent from UART TX buffer every 100 milli-seconds</li>
 * 
 *              </ul>
 * 
 *              To maximize data transmission throughput, we should maximize the number 
 *              of bytes queued to UART TX buffer between each call of TASK_vidUartLoopback(). 
 *              To achieve that, number of bytes transmitted between each call of 
 *              TASK_vidUartLoopback() is calculated. And CONF_UART_CHANNEL_3_TX_BUFFER_SIZE
 *              must be at least equal to that number.
 *              For baudrate 4800 bps:
 *                - bytes per sec = baudrate / 10 
 *                                = 4800 / 10 
 *                                = 480 bytes / sec
 *                
 *                - bytes per milli-second = (bytes per sec) / 1000
 *                                         = 480 / 1000
 *                                         = 0.48 bytes / sec
 *                
 *                - CONF_UART_CHANNEL_3_TX_BUFFER_SIZE = ((bytes per milli-second * period) + 1)
 *                                                     = (0.48 * 100) + 1
 *                                                     = 49 bytes
 *  
 *              So every call of TASK_vidUartLoopback() will queue atmost 48 bytes into
 *              UART TX buffer (buffer capacity = buufer size - 1), which matches UART baudrate 
 *              to transmit 48 bytes every 100 milli-seconds. A larger buffer size will work,
 *              but:
 *                - it will cost more RAM, and 
 *                - a larger TX buffer size means that more data cxan be queued at the same time, 
 *                  so transmitting large messages is easy. However, uart_loopback() keeps track
 *                of how many bytes were added to UART TX buffer, and queues large messages in
 *                chunks
 * 
*               TASK_vidUartUpdate() (and in turn UART_enUpdateChannel()) is called 
 *              with period of 1 ms (smallest tick rate), that means a new byte can be 
 *              received/transmitted every 1 milli-second, so a maximum baudrate of 
 *              1000 bytes / sec (or 10 Kbps)
 * 
 *              In theory, with this set up, it should be possible to transmit and receive
 *              UART data at rate 10 Kpbs (1 KB/s). UART RXNE flag must be checked at a rate
 *              at least equal to UART (buadrate / frame size). And since simple os is 
 *              configured with 1000 ticks/sec, that's the maximum rate that UART RXNE flag
 *              can be checked. So it's possible to receive UART data for any baudare < 10 Kbps.
 *              
 *              In practice, I found that's not totally true. It's possible to receive data at that rate,
 *              but there will be some errors and lost data bytes at baudrates > 4800 bps. 
 *              
 *              Since UART_enUpdateChannel() is called at rate (1000 calles/sec)
 *              there will be data loss when using baudrates > 5 Kbps (500 bytes /sec)
 *              If RXNE value is plotted against time, and assuming:
 *              - the flag is cleared shortly after it's set, and
 *              - bytes are received continuosly
 *              the plot will look like a square wave with:
 *              - frequency = baudrate / frame size (10 bits)
 *              - on time   = time it taskes to process received byte & clear RXNE flag
 *              - off time  = time after last received byte was processed and the reception of a new byte
 * 
 *              As to sample a signal correctly, we must sample it as double its 
 *              maximum frequency (Nyquist rate). So to check RXNE flag, 
 *              it must be at a rate at least 2x the frequency RXNE is set.
 *              And maximum baudrate is 5000 pbs (or 4800 pbs) where this application
 *              will operate reliably with no data loss.
 *              
 * @author      Mohammad Mohsen <kuro.ece@gmail.com>
 * @brief 
 * @version     1.0
 * @date        2022-11-23
 * 
 * @copyright   Copyright (c) 2022
 * 
 *****************************************************************************/

/* Includes ---------------------------------------------------------------- */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "gpio.h"

#include "board_config.h"
#include "serial/uart/uart.h"
#include "simple_os.h"

/* Private typedef --------------------------------------------------------- */

/**
 * @brief UART received data
 */
typedef struct line_t
{
    uint8_t *msg;
    uint32_t max_len;
    uint32_t len;
} Line_t;

/* Private define ---------------------------------------------------------- */


/**
 * @brief Max baudrate for all UART channels @ clock speed Fclk = 36 MHz 
 */
#define MAX_BAUDRATE            921600

#if CONF_UART_BAUDRATE > MAX_BAUDRATE
#error Maximum alowed baudrate is 921600
#endif

/**
 * @brief UART loop back buffer size
 */
#define LOOPBACK_RX_BUFFER_SIZE 256

/* Private macro ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/**
 * @brief UART configurations
 */
const UART_Conf_t uart_conf = {
    .BaudRate       = CONF_UART_BAUDRATE,
    .WordLength     = CONF_UART_DATASIZE,
    .Parity         = CONF_UART_PARITY,
    .StopBits       = CONF_UART_STOPBITS,
    .Mode           = CONF_UART_MODE,
    .HwFlowCtl      = CONF_UART_HWCONTROL,
    .OverSampling   = CONF_UART_OVERSAMPLING,
};

/**
 * @brief UART channel's received data buffers
 */
uint8_t rx_buffer [UART_CHANNEL_COUNT][LOOPBACK_RX_BUFFER_SIZE] = {0};

/**
 * @brief UART channel's received adata
 */
Line_t  rx_line[UART_CHANNEL_COUNT] = {0};


/* Private function prototypes --------------------------------------------- */

/**
 * @brief Initialize system and bus clocks
 */
static void SystemClock_Config(void);

/**
 * @brief initialize UART channel using configuration @ref uart_conf
 * 
 * @param [in] ch number of uart channel to initialize
 * 
 * @return @ref UART_Error_t 
 */
static inline UART_Error_t uart_init(UART_Channel_t ch);

/**
 * @brief loopback (echo) received lines on uart channel
 * 
 * @param [in] ch number of uart channel to loopback
 * 
 * @return @ref UART_Error_t 
 */
static inline UART_Error_t uart_loopback(UART_Channel_t ch);

/**
 * @brief   SimpleOS task that updates UART channels (UART1, UART2 and UART3)
 * 
 * @param [in] pvArgs   pointer to task arguments, passed to task when it's called by 
 *                      SimpleOS's task dispatcher. NULL for this task
 */
static void TASK_vidUartUpdate(void * pvArgs);

/**
 * @brief   SimpleOS task that loops back received data bytes 
 *          on UART channels (UART1, UART2 and UART3)
 * 
 * @param [in] pvArgs   pointer to task arguments, passed to task when it's called by 
 *                      SimpleOS's task dispatcher. NULL for this task
 */
static void TASK_vidUartLoopback(void * pvArgs);

/* Private functions implementations --------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** 
     * Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure:
     *   - Enable HSI
     *   - Enable  PLL
     *   - PLL_CLK_SRC  = HSI_CLK / 2
     *   - PLL_MUL      = PLL_MUL_9
     * 
     * PLL_CLK  = PLL_CLK_SRC * PLL_MUL 
     *          = HSI_CLK / 2 * 9
     * PLL_CLK  = 36 MHz
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** 
     * Initializes the CPU, AHB and APB buses clocks:
     *   - SYS_CLK    = PLL_CLK
     *   - AHB_CLK    = SYS_CLK
     *   - APB1_CLK   = AHB_CLK / 1
     *   - APB2_CLK   = AHB_CLK / 1
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }

    /* initialize systick with updated clock */
    HAL_InitTick(TICK_INT_PRIORITY);
}

static inline UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;

    uart_error = UART_enInitialize(ch, &uart_conf);
    assert_param(uart_error == UART_ERROR_NONE);

    return uart_error;
}

static inline UART_Error_t uart_loopback(UART_Channel_t ch)
{
    Line_t *uart_rx_line = &rx_line[ch];
    uint32_t byte_count = 0;
    UART_Error_t uart_error;

    /* read line fropm UART RX buffer */
    uart_error = UART_enReadLine(
        ch,
        &uart_rx_line->msg[uart_rx_line->len],
        uart_rx_line->max_len - uart_rx_line->len - 1,
        &byte_count
    );

    /* loopback (echo) data  */
    uart_rx_line->len += byte_count;
    
    if (uart_rx_line->len > 0)
    {
        uart_error = UART_enWrite(
            ch,
            uart_rx_line->msg,
            uart_rx_line->len,
            &byte_count
        );

        uart_rx_line->len -= byte_count;
    }

    return uart_error;
}

static void TASK_vidUartUpdate(void * pvArgs)
{
    UART_enUpdateChannel(UART_CHANNEL_1);
    UART_enUpdateChannel(UART_CHANNEL_2);
    UART_enUpdateChannel(UART_CHANNEL_3);
}

static void TASK_vidUartLoopback(void * pvArgs)
{
    uart_loopback(UART_CHANNEL_1);
    uart_loopback(UART_CHANNEL_2);
    uart_loopback(UART_CHANNEL_3);
}

/* main --------------------------------------------------------------------- */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    OS_Error_t os_error;
    OS_TaskHandle_t uart_update_task_handle;
    OS_TaskHandle_t uart_loopback_task_handle;
    
    /* MCU Configuration------------------------------------------------------ */

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    /*  initialize UART channels receive buffer  */
    for(UART_Channel_t ch = 0; ch < UART_CHANNEL_COUNT; ch++)
    {
        rx_line[ch].msg = rx_buffer[ch];
        rx_line[ch].max_len = LOOPBACK_RX_BUFFER_SIZE - 1;
        rx_line[ch].len = 0;
    }

    /* initialize UART channels */
    uart_init(UART_CHANNEL_1);
    uart_init(UART_CHANNEL_2);
    uart_init(UART_CHANNEL_3);

    OS_vidInitialize();

    os_error = OS_enAddTask(
        TASK_vidUartUpdate,
        NULL,
        0,
        OS_MS_TO_TICKS(1),
        0,
        &uart_update_task_handle
    );
    assert_param(os_error == OS_ERROR_NONE);

    os_error = OS_enAddTask(
        TASK_vidUartLoopback,
        NULL,
        1,
        OS_MS_TO_TICKS(100),
        OS_MS_TO_TICKS(1000),
        &uart_loopback_task_handle
    );
    assert_param(os_error == OS_ERROR_NONE);
    
    /* Infinite loop */
    while (1)
    {
        OS_vidDispatchTasks();
    }
}

/**
 * @brief Update UART channels when @ref UART_MINIMAL_INTERRUPTS is enabled
 */
void HAL_SYSTICK_Callback(void)
{
    OS_vidUpdateTasks();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    Error_Handler();
    (void)file;
    (void)line;
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
