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
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//UART_HandleTypeDef huart2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_TXBUF_SIZE 1024 // rozmiar bufora nadawczego
#define USART_RXBUF_SIZE 128 // rozmiar bufora odbiorczego

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
uint8_t USART_BUF_RX[USART_TXBUF_SIZE];
volatile int USART_RX_EMPTY = 0;
volatile int USART_RX_BUSY = 0;


/* === UART Transmit === */
uint8_t USART_BUF_TX[USART_TXBUF_SIZE];
volatile int USART_TX_EMPTY = 0;
volatile int USART_TX_BUSY = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void increase_usart_tx_busy();
void increase_usart_tx_empty();
void increase_usart_rx_busy();
void increase_usart_rx_empty();

int usart_tx_has_data();
int usart_rx_has_data();

uint16_t calculate_crc(const char *data, size_t length);
char* get_crc_hex(const char *input);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* === CRC === */
uint16_t calculate_crc(const char *data, size_t length) {
	uint16_t crc = 0x0000; // początkowa wartość
	uint16_t polynomial = 0xA001; // polinom (odwrócony 0x8005)

	for (size_t i = 0; i < length; i++) {
		crc ^= (uint8_t)data[i]; // XOR z bieżącym bajtem
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x0001) crc = (crc >> 1) ^ polynomial;
			else crc >>= 1;
		}
	}
	return crc;
}

char* get_crc_hex(const char *input) {
	char tmp[257]; // tymczasowa tablica
	strncpy(tmp, input, 256);
	tmp[256] = '\0';
	size_t length = strnlen(tmp, 256);

	uint16_t crc = calculate_crc(tmp, length); // obliczanie crc
	char *crc_string = (char *)malloc(5);
	if (!crc_string) return NULL;

	snprintf(crc_string, 5, "%04X", crc); // formatowanie

	return crc_string;
}


/* === INDEX INCREMENTING FUNCTIONS === */
void increase_usart_tx_busy() {
    USART_TX_BUSY = (USART_TX_BUSY + 1) % USART_TXBUF_SIZE;
}

void increase_usart_tx_empty() {
    USART_TX_EMPTY = (USART_TX_EMPTY + 1) % USART_TXBUF_SIZE;
}

void increase_usart_rx_busy() {
	USART_RX_BUSY = (USART_RX_BUSY + 1) % USART_RXBUF_SIZE;
}

void increase_usart_rx_empty() {
    USART_RX_EMPTY = (USART_RX_EMPTY + 1) % USART_RXBUF_SIZE;
}


/* === BUFFER EMPTY CHECK FUNCTIONS === */
int usart_tx_has_data() {
    return USART_TX_EMPTY != USART_TX_BUSY;
}

int usart_rx_has_data() {
    return USART_RX_EMPTY != USART_RX_BUSY;
}



/* === USART CALLBACK RECEIVE === */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		increase_usart_rx_empty();
		HAL_UART_Receive_IT(&huart2, &USART_BUF_RX[USART_RX_EMPTY], 1);
	}
}

/* === USART TRANSMIT CALLBACK === */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // sprawdzenie, czy są dane do wysłania
        if (usart_tx_has_data()) {
        	uint8_t tmp = USART_BUF_TX[USART_TX_BUSY];
        	increase_usart_tx_busy();
            HAL_UART_Transmit_IT(&huart2, &tmp, 1); // bajt wysłany, można wysłać następny
        }
    }
}

int16_t USART_getchar(){
	int16_t tmp;
	if (usart_rx_has_data()) {
		 tmp = USART_BUF_RX[USART_RX_BUSY];
		 increase_usart_rx_busy();
		 return tmp;
	} else return -1;
}

uint8_t USART_getline(char *buf) {
	static uint8_t bf[128];
	static uint8_t idx=0;
	int i;
	uint8_t ret;
	while(usart_rx_has_data()) {
		bf[idx] = USART_getchar();
		if (bf[idx] == 10 || bf[idx] == 13) { // znak \n lub \r
			bf[idx] = 0;
			for(i = 0; i <= idx; i++) {
				buf[i]=bf[i]; // kopiowanie do bufora
			}
			ret = idx;
			idx = 0;
			return ret; // odebrano linie
		} else {
			idx++;
			if (idx >= 128) idx=0; // powrót na początek bufora
		}
	}
	return 0;
}


void send(char* format, ...) {
	char tmp[256];
	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp, format, arglist);
	va_end(arglist);
	volatile int idx = USART_TX_EMPTY;
	for (int i = 0; i < strlen(tmp); i++) {
		USART_BUF_TX[idx] = tmp[i];
		idx = (idx + 1) % USART_TXBUF_SIZE;
	}

	__disable_irq();
	USART_TX_EMPTY = idx;
	if (!usart_tx_has_data() && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET) {
		uint8_t tmp = USART_BUF_TX[USART_TX_BUSY];
		increase_usart_tx_busy();
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}
	__enable_irq();
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

  HAL_UART_Receive_IT(&huart2, &USART_BUF_RX[USART_RX_EMPTY], 1);

  //TIM2_Init();
  //DHT11_GPIO_Init();

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
