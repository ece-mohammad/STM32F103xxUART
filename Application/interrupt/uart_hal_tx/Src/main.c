/******************************************************************************
 * @file        main.c 
 * @brief       Transmit data on UART channels, using 
 *              UART HAL (ST HAL drivers) implementation 
 *              (Modules/serial/uart/hal/uart.c)
 * @details     Continuously sends a long text on UART channels UART1, UART2 
 *              and UART3.
 *              MCU configurations:
 *              <ul>
 *                <li>System Clokc: 16 MHz</li>
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

/* Private includes -------------------------------------------------------- */
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
    .BaudRate       = CONF_UART_BAUDRATE,
    .WordLength     = CONF_UART_DATASIZE,
    .Parity         = CONF_UART_PARITY,
    .StopBits       = CONF_UART_STOPBITS,
    .Mode           = CONF_UART_MODE,
    .HwFlowCtl      = CONF_UART_HWCONTROL,
    .OverSampling   = CONF_UART_OVERSAMPLING,
};

static TX_Data_t tx_data [UART_CHANNEL_COUNT] = {0};

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

/* Private function definitions -------------------------------------------- */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

static UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;

    uart_error = UART_enInitialize(ch, &uart_conf);
    assert_param(uart_error == UART_ERROR_NONE);

    return uart_error;
}

/* main -------------------------------------------------------------------- */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    static const uint8_t str[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, "
                                    "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. "
                                    "Ut enim ad minim veniam, "
                                    "quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. "
                                    "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. "
                                    "Excepteur sint occaecat cupidatat non proident, "
                                    "sunt in culpa qui officia deserunt mollit anim id est laborum.\r\n";

    uint32_t write_count;

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    for(UART_Channel_t ch = 0; ch < UART_CHANNEL_COUNT; ch++)
    {
        tx_data[ch].data = (uint8_t *)str;
        tx_data[ch].len = strlen((const char *)str);
        tx_data[ch].tx_len = 0;
    }

    /* initialize UART channels */
    uart_init(UART_CHANNEL_1);
    uart_init(UART_CHANNEL_2);
    uart_init(UART_CHANNEL_3);

    while (1)
    {
        
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
