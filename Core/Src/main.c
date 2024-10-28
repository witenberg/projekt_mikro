/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_HandleTypeDef huart2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 256


#define FRAME_START 0x3A
#define FRAME_END 0x3B

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* === FLAGS === */

/* === UART Receive === */
volatile uint8_t buf_RX[UART_BUFFER_SIZE];
volatile uint16_t IDX_RX_EMPTY = 0;
volatile uint16_t IDX_RX_BUSY = 0;


/* === UART Transmit === */
volatile uint8_t buf_TX[UART_BUFFER_SIZE];
volatile uint16_t IDX_TX_EMPTY = 0;
volatile uint16_t IDX_TX_BUSY = 0;




typedef struct {
    uint8_t start;
    char sender[2];
    char receiver[2];
    char length[3];
    uint8_t data[128];
    uint16_t crc;
    uint8_t end;
} UART_Frame;

UART_Frame frame;

int frameLength = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* === INDEX INCREMENTING FUNCTIONS === */
void increase_tx_busy() {
    IDX_TX_BUSY = (IDX_TX_BUSY + 1) % UART_BUFFER_SIZE;
}

void increase_tx_empty() {
    IDX_TX_EMPTY = (IDX_TX_EMPTY + 1) % UART_BUFFER_SIZE;
}

void increase_rx_busy() {
    IDX_RX_BUSY = (IDX_RX_BUSY + 1) % UART_BUFFER_SIZE;
}

void increase_rx_empty() {
    IDX_RX_EMPTY = (IDX_RX_EMPTY + 1) % UART_BUFFER_SIZE;
}


/* === BUFFER EMPTY CHECK FUNCTIONS === */
int tx_has_data() {
    return IDX_TX_EMPTY != IDX_TX_BUSY;
}

int rx_has_data() {
    return IDX_RX_EMPTY != IDX_RX_BUSY;
}



/* === USART CALLBACK RECEIVE === */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		// Wyłączenie przerwań, aby uniknąć zakłóceń w trakcie ustawiania danych do wysłania
		__disable_irq();

		increase_rx_empty();

		// Odbieranie
		HAL_UART_Receive_IT(&huart2, &buf_RX[IDX_RX_EMPTY], 1);

		// Włączenie przerwań
		__enable_irq();
	}
}

/* === USART TRANSMIT CALLBACK === */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // Sprawdzenie, czy są kolejne dane do wysłania
        if (tx_has_data()) {
        	// Wyłączenie przerwań, aby uniknąć zakłóceń w trakcie ustawiania danych do wysłania
        	__disable_irq();

            HAL_UART_Transmit_IT(huart, (uint8_t *)&buf_TX[IDX_TX_BUSY], 1);
            increase_tx_busy();

            // Włączenie przerwań
            __enable_irq();
        }
    }
}


void send(char* msg) {

	// Ustawienie wskaźnika na koniec bufora
	uint16_t idx = IDX_TX_EMPTY;

	// Kopiowanie danych do bufora
	for (int i=0; i<strlen(msg); i++) {
		buf_TX[idx] = msg[i];
		idx = (idx + 1) % UART_BUFFER_SIZE;
	}

	IDX_TX_EMPTY = idx;

	// Sprawdzenie, czy nadawanie jest gotowe i czy nie ma zaległych danych do wysłania
	if (!tx_has_data() && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {

		// Wyłączenie przerwań, aby uniknąć zakłóceń w trakcie ustawiania danych do wysłania
		__disable_irq();

		// Wysyłanie
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf_TX[IDX_TX_BUSY], 1);
		increase_tx_busy();

		// Włączenie przerwań
		__enable_irq();
	}
}



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
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &buf_RX[IDX_RX_EMPTY], 1);

  TIM2_Init();
  DHT11_GPIO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
