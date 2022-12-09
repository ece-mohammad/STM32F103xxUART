/******************************************************************************
 * @file        main.c 
 * @brief       Transmit data continuously on UART channels, using 
 *              UART LL (ST LL drivers) implementation 
 *              (Modules/serial/uart/ll/uart.c)
 * @details     Echo back received bytes on UART1, UART2 and UART3.
 *              MCU configurations:
 *              <ul>
 * 
 *                <li>System Clokc: 16 MHz</li>
 * 
 *                <li>UARTx (1, 2, 3) baudrate: 4800 bps, configurable 
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
 *              1000 bytes / sec (or 10 Kbps). So configuring UART with a highre baudrate 
 *              is not efficient. It may work, but it will depend on the receiving side.
 *              Data reception however, will not work.
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

/* Private includes -------------------------------------------------------- */
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
typedef struct tx_data_t
{
    uint8_t *data;
    uint32_t len;
    uint32_t tx_len;
} TX_Data_t;

/* Private define ---------------------------------------------------------- */

/**
 * @brief Max baudrate for all UART channels @ clock speed Fclk = 16 MHz (BRR = 1)
 * */
#define MAX_BAUDRATE            921600

#if CONF_UART_BAUDRATE > MAX_BAUDRATE
#error Maximum alowed baudrate is 921600
#endif

/* Private macro ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/**
 * @brief UAT configurations
 */
const UART_Conf_t uart_conf = {
    .BaudRate               = CONF_UART_BAUDRATE,
    .DataWidth              = CONF_UART_DATASIZE,
    .Parity                 = CONF_UART_PARITY,
    .StopBits               = CONF_UART_STOPBITS,
    .TransferDirection      = CONF_UART_DIRECTION,
    .HardwareFlowControl    = CONF_UART_HWCONTROL,
    .OverSampling           = CONF_UART_OVERSAMPLING,
};

static TX_Data_t uart_data [UART_CHANNEL_COUNT] = {0};

/* Private function prototypes --------------------------------------------- */

/**
 * @brief   System Clock Configuration
 * 
 * @details Set System clock to 16 MHz (from PLL),
 *          and bus clocks (AHB, APB1, APB2) prescalers to 1
 */
static void SystemClock_Config(void);

/**
 * @brief Initialize uart channel @ref ch
 * 
 * @param [in] ch channel number #UART_Channel_t 
 * 
 * @return @ref UART_Error_t 
 * 
 */
static UART_Error_t uart_init(UART_Channel_t ch);

/**
 * @brief SimpleOS task that updates UART channels (UART1, UART2, UART3)
 * 
 * @param pvArgs NULL
 */
static void TASK_vidUartUpdate(void * pvArgs);

/**
 * @brief SimpleOS task to transmit data on UART channels (UART1, UART2, UART3)
 * 
 * @param pvArgs pointer to TX_Data_t array of UART channels' transmit data
 */
static void TASK_vidUartTransmit(void * pvArgs);

/* Private function definitions -------------------------------------------- */

static void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
    {
    }

    /** 
     * Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure:
     *   - Enable HSI
     *   - Enable  PLL
     *   - PLL_CLK_SRC  = HSI_CLK / 2
     *   - PLL_MUL      = PLL_MUL_4
     * 
     * PLL_CLK  = PLL_CLK_SRC * PLL_MUL 
     *          = HSI_CLK / 2 * 4 
     *          = HSI_CLK * 2 
     * PLL_CLK  = 16 MHz
     */
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1)
    {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_4);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1)
    {

    }

    /** 
     * Initializes the CPU, AHB and APB buses clocks:
     *   - SYS_CLK    = PLL_CLK
     *   - AHB_CLK    = SYS_CLK
     *   - APB1_CLK   = AHB_CLK / 1
     *   - APB2_CLK   = AHB_CLK / 1
     */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }
    LL_Init1msTick(16000000);
    LL_SetSystemCoreClock(16000000);

    /*  Initialize SysTick to generate interrupts   */
    SysTick->LOAD  = (uint32_t)((16000000 / 1000) - 1UL);       /* set reload register */
    SysTick->VAL   = 0UL;                                       /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
            SysTick_CTRL_TICKINT_Msk |                          /* Enable the Systick interrupt */
            SysTick_CTRL_ENABLE_Msk;                            /* Enable the Systick Timer */

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
}

static UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;

    uart_error = UART_enInitialize(ch, &uart_conf);
    assert_param(uart_error == UART_ERROR_NONE);

    return uart_error;
}

static void TASK_vidUartUpdate(void * pvArgs)
{
    UART_enUpdateChannel(UART_CHANNEL_1);
    UART_enUpdateChannel(UART_CHANNEL_2);
    UART_enUpdateChannel(UART_CHANNEL_3);
}

static void TASK_vidUartTransmit(void * pvArgs)
{
    uint32_t write_count;
    TX_Data_t * tx_data = (TX_Data_t *)pvArgs;

    for(UART_Channel_t ch = 0; ch < UART_CHANNEL_COUNT; ch++)
    {
        UART_enWrite(
            ch, 
            &tx_data[ch].data[tx_data[ch].tx_len], 
            tx_data[ch].len - tx_data[ch].tx_len, 
            &write_count
        );

        tx_data[ch].tx_len += write_count;

        if(tx_data[ch].tx_len >= tx_data[ch].len)
        {
            tx_data[ch].tx_len = 0;
        }
    }
}

/* main -------------------------------------------------------------------- */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    OS_Error_t os_error;
    OS_TaskHandle_t uart_update_task_handle;
    OS_TaskHandle_t uart_transmit_task_handle;
    static const uint8_t str[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, "
                                    "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. "
                                    "Ut enim ad minim veniam, "
                                    "quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. "
                                    "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. "
                                    "Excepteur sint occaecat cupidatat non proident, "
                                    "sunt in culpa qui officia deserunt mollit anim id est laborum.\r\n";

    /* MCU Configuration--------------------------------------------------------*/

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

    /** 
     * NOJTAG : JTAG-DP Disabled and SW-DP Enabled
     */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    for(UART_Channel_t ch = 0; ch < UART_CHANNEL_COUNT; ch++)
    {
        uart_data[ch].data = (uint8_t *)str;
        uart_data[ch].len = strlen((const char *)str);
        uart_data[ch].tx_len = 0;
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
        OS_MS_TO_TICKS(1),
        &uart_update_task_handle
    );
    assert_param(os_error == OS_ERROR_NONE);

    os_error = OS_enAddTask(
        TASK_vidUartTransmit,
        uart_data,
        1,
        OS_MS_TO_TICKS(100),
        0,
        &uart_transmit_task_handle
    );
    assert_param(os_error == OS_ERROR_NONE);

    while (1)
    {
        OS_vidDispatchTasks();
    }
}

#if defined(CONF_UART_MINIMAL_INTERRUPTS)

/**
 * @brief Update UART channels when @ref UART_MINIMAL_INTERRUPTS is enabled
 */
void SysTick_Handler(void)
{
    OS_vidUpdateTasks();
}

#endif /* defined(CONF_UART_MINIMAL_INTERRUPTS) */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    // __disable_irq();
    // while (1)
    // {
    // }
    /* USER CODE END Error_Handler_Debug */
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
    /* USER CODE BEGIN 6 */
    Error_Handler();
    (void)file;
    (void)line;
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
