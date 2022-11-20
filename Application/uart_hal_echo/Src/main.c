/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board_config.h"

#include "serial/uart/uart.h"
#include "serial_debug/debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct line_t
{
    uint8_t *msg;
    uint32_t max_len;
    uint32_t len;
} Line_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_UART_BAUDRATE      115200
#define LOOPBACK_RX_BUFFER_SIZE 256

/*
 * Max baudrate for all UART channels @ clock speed Fclk = 16 MHz (BRR = 1)
 * */
#define MAX_BAUDRATE            921600

#if TEST_UART_BAUDRATE > MAX_BAUDRATE
#error Maximum alowed baudrate is 921600
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const UART_Conf_t uart_conf = {
    .BaudRate       = TEST_UART_BAUDRATE,
    .WordLength     = CONF_DEBUG_UART_DATASIZE,
    .Parity         = CONF_DEBUG_UART_PARITY,
    .StopBits       = CONF_DEBUG_UART_STOPBITS,
    .Mode           = UART_MODE_TX_RX,
    .HwFlowCtl      = UART_HWCONTROL_NONE,
    .OverSampling   = UART_OVERSAMPLING_16,
};


uint8_t rx_msg [UART_CHANNEL_COUNT][LOOPBACK_RX_BUFFER_SIZE] = {0};
Line_t  rx_line[UART_CHANNEL_COUNT] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief initialize UART channel using configuration @ref uart_conf
 * 
 * @param ch number of uart channel to initialize
 * @return UART_Error_t 
 */
static UART_Error_t uart_init(UART_Channel_t ch);

/**
 * @brief loopback (echo) received lines on uart channel
 * 
 * @param ch number of uart channel to loopback
 * @return UART_Error_t 
 */
static UART_Error_t uart_loopback(UART_Channel_t ch);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    /*  initialize UART channels receive buffer  */
    for(UART_Channel_t ch = 0; ch < UART_CHANNEL_COUNT; ch++)
    {
        rx_line[ch].msg = rx_msg[ch];
        rx_line[ch].max_len = LOOPBACK_RX_BUFFER_SIZE - 1;
        rx_line[ch].len = 0;
    }

    /* initialize UART channels */
    uart_init(UART_CHANNEL_1);
    uart_init(UART_CHANNEL_2);
    uart_init(UART_CHANNEL_3);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* loopback UART channels */
        uart_loopback(UART_CHANNEL_1);
        uart_loopback(UART_CHANNEL_2);
        uart_loopback(UART_CHANNEL_3);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
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

    /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */
UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;
    uart_error = UART_enInitialize(ch, &uart_conf);

#ifdef DEBUG
    assert_param(uart_error == UART_ERROR_NONE);
#endif /* DEBUG */

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

    /* update  */
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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
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
