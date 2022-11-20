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
        uint8_t* msg;
        uint32_t max_len;
        uint32_t len;
} Line_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_UART_DAISY_CHAIN
#define TEST_UART_CHANNEL   UART_CHANNEL_1
#define TEST_UART_BAUDRATE  115200
/*
 * Max baudrate for all UART channels @ clock speed Fclk = 16 MHz (BRR = 1)
 * */
#define MAX_BAUDRATE        921600

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
        .BaudRate               = TEST_UART_BAUDRATE,
        .DataWidth              = CONF_DEBUG_UART_DATASIZE,
        .Parity                 = CONF_DEBUG_UART_PARITY,
        .StopBits               = CONF_DEBUG_UART_STOPBITS,
        .TransferDirection      = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl    = LL_USART_HWCONTROL_NONE,
        .OverSampling           = LL_USART_OVERSAMPLING_16,
};

uint8_t rx_msg [3][1024] = {0};
Line_t rx_line [3] = {
        {.msg = rx_msg[0], .max_len = sizeof(rx_msg[0]) - 1, .len = 0},
        {.msg = rx_msg[1], .max_len = sizeof(rx_msg[1]) - 1, .len = 0},
        {.msg = rx_msg[2], .max_len = sizeof(rx_msg[2]) - 1, .len = 0},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

UART_Error_t uart_init(UART_Channel_t ch);
UART_Error_t uart_loopback(UART_Channel_t ch);

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
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

    /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
     */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /*  Initialize SysTick to generate interrupts   */
    SysTick->LOAD  = (uint32_t)((16000000 / 1000) - 1UL);       /* set reload register */
    SysTick->VAL   = 0UL;                                       /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
            SysTick_CTRL_TICKINT_Msk |                   /* Enable the Systick interrupt */
            SysTick_CTRL_ENABLE_Msk;                   /* Enable the Systick Timer */


    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    uart_init(UART_CHANNEL_1);
    uart_init(UART_CHANNEL_2);
    uart_init(UART_CHANNEL_3);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

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
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
    {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1)
    {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_8);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1)
    {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }
    LL_Init1msTick(32000000);
    LL_SetSystemCoreClock(32000000);
}

/* USER CODE BEGIN 4 */
UART_Error_t uart_init(UART_Channel_t ch)
{
    UART_Error_t uart_error;
    
    uart_error =  UART_enInitialize(ch, &uart_conf);
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

void SysTick_Handler(void)
{
    UART_enUpdateChannel(UART_CHANNEL_1);
    UART_enUpdateChannel(UART_CHANNEL_2);
    UART_enUpdateChannel(UART_CHANNEL_3);
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
    // __disable_irq();
    // while (1)
    // {
    // }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
