/******************************************************************************
 * @file        main.c 
 * @brief       Echo received data on UART channels, using 
 *              UART HAL (ST HAL drivers) implementation 
 *              (Modules/serial/uart/hal/uart.c)
 * @details     Echo back received bytes on UART1, UART2 and UART3. 
 *              MCU configurations:
 *              <ul>
 *                <li>System Clokc: 32 MHz</li>
 *                <li>UARTx (1, 2, 3) baudrate: 9600 up to 921600 bps,
 *                  configurable using @ref UART_BAUDRATE</li>
 *              </ul>
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
static UART_Error_t uart_init(UART_Channel_t ch);

/**
 * @brief loopback (echo) received lines on uart channel
 * 
 * @param [in] ch number of uart channel to loopback
 * 
 * @return @ref UART_Error_t 
 */
static UART_Error_t uart_loopback(UART_Channel_t ch);

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
     *   - PLL_MUL      = PLL_MUL_8
     * 
     * PLL_CLK  = PLL_CLK_SRC * PLL_MUL 
     *          = HSI_CLK / 2 * 8
     * PLL_CLK  = 32 MHz
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

    HAL_InitTick(TICK_INT_PRIORITY);
}

UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;

    uart_error = UART_enInitialize(ch, &uart_conf);
    assert_param(uart_error == UART_ERROR_NONE);

    return uart_error;
}

UART_Error_t uart_loopback(UART_Channel_t ch)
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


/* main --------------------------------------------------------------------- */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
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


    /* Infinite loop */
    while (1)
    {
        /* loopback received data on UART channels */
        uart_loopback(UART_CHANNEL_1);
        uart_loopback(UART_CHANNEL_2);
        uart_loopback(UART_CHANNEL_3);
    }
}

#if defined(UART_MINIMAL_INTERRUPTS)

/**
 * @brief Update UART channels when @ref UART_MINIMAL_INTERRUPTS is enabled
 */
void HAL_SYSTICK_Callback(void)
{
    UART_enUpdateChannel(UART_CHANNEL_1);
    UART_enUpdateChannel(UART_CHANNEL_2);
    UART_enUpdateChannel(UART_CHANNEL_3);
}

#endif /*  UART_MINIMAL_INTERRUPTS  */

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
