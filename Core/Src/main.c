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
#include <ctype.h>
#include <errno.h>

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

#define MASK 0x2F
#define MASKED_START 0x2E
#define MASKED_END 0x2C

#define SENDER "PC"
#define RECEIVER "MC"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* === UART Receive === */
uint8_t USART_BUF_RX[USART_RXBUF_SIZE];
volatile uint16_t USART_RX_EMPTY = 0;
volatile uint16_t USART_RX_BUSY = 0;


/* === UART Transmit === */
uint8_t USART_BUF_TX[USART_TXBUF_SIZE];
volatile uint16_t USART_TX_EMPTY = 0;
volatile uint16_t USART_TX_BUSY = 0;

/* Flagi */
volatile uint8_t DHT11_READ_FLAG = 0;
uint32_t measurement_interval = 5000;
//volatile uint8_t is_handling = 0; // flaga sprawdzająca, czy jest aktualnie obsługiwany jakiś znak


/* Stany */
typedef enum {
	IDLE,
	FIND_SENDER,
	FIND_RECEIVER,
	FIND_LENGTH,
	FIND_DATA,
	FIND_CRC,
	FIND_END,
	FIND_MASKED
} FrameState;

/* Struktura ramki */
typedef struct {
	uint8_t sender[3];
	uint8_t receiver[3];
	uint8_t length[4];
	uint8_t data[257];
	uint8_t crc_frame[5];

	uint16_t crc_calculated;
	uint8_t sender_id;
	uint8_t receiver_id;
	uint8_t length_id;
	uint16_t data_id;
	uint8_t crc_id;

	uint16_t length_int;
	uint16_t masked_counter;
	FrameState state;
	bool complete;
} Frame;

Frame frame;

dht11_t dht;
dht11_t *pDHT = &dht;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* === CRC === */
uint16_t calculate_crc_byte(uint16_t crc, uint8_t data) {
	crc ^= (data << 8);
	for (uint8_t i = 0; i < 8; i++) {
		if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
		else crc <<= 1;
	}
	return crc;
}

uint8_t USART_kbhit(){
	if (USART_RX_EMPTY == USART_RX_BUSY){
		return 0;
	} else {
		return 1;
	}
}

void USART_fsend(char* format, ...) {
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
	if (USART_TX_EMPTY == USART_TX_BUSY && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET) {
		USART_TX_EMPTY = idx;
		uint8_t tmp = USART_BUF_TX[USART_TX_BUSY];
		USART_TX_BUSY++;
		if (USART_TX_BUSY >= USART_TXBUF_SIZE) USART_TX_BUSY = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}
	else USART_TX_EMPTY = idx;
	__enable_irq();
}

/* === USART CALLBACK RECEIVE === */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		USART_RX_EMPTY++;
		if (USART_RX_EMPTY >= USART_RXBUF_SIZE) USART_RX_EMPTY = 0;
		HAL_UART_Receive_IT(&huart2, &USART_BUF_RX[USART_RX_EMPTY], 1);
	}
}

/* === USART TRANSMIT CALLBACK === */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // sprawdzenie, czy są dane do wysłania
        if (USART_TX_EMPTY != USART_TX_BUSY) {
        	uint8_t tmp = USART_BUF_TX[USART_TX_BUSY];
        	USART_TX_BUSY++;
        	if (USART_TX_BUSY >= USART_TXBUF_SIZE) USART_TX_BUSY = 0;
            HAL_UART_Transmit_IT(&huart2, &tmp, 1); // bajt wysłany, można wysłać następny
        }
    }
}

int16_t USART_getchar(){
	int16_t tmp;
	if (USART_RX_EMPTY != USART_RX_BUSY) {
		 tmp = USART_BUF_RX[USART_RX_BUSY];
		 USART_RX_BUSY++;
		 if (USART_RX_BUSY >= USART_RXBUF_SIZE) USART_RX_BUSY = 0;
		 return tmp;
	} else return -1;

}

void delay_us(uint16_t us) {
	HAL_TIM_Base_Stop(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	HAL_TIM_Base_Start(&htim3);

	while (__HAL_TIM_GET_COUNTER(&htim3) < us);

	HAL_TIM_Base_Stop(&htim3);


}

void count_us() {
	HAL_TIM_Base_Stop(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_Base_Start(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
    	DHT11_READ_FLAG = 1;
    }
}

void set_interval(uint32_t interval) {

	// dodac interwal > 16 bitow

	measurement_interval = interval;
	HAL_TIM_Base_Stop_IT(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, interval - 1);
	HAL_TIM_Base_Start_IT(&htim4);

	USART_fsend("INTERVAL_OK");
}

void get_interval() {
	// tutaj bedzie ramka
	USART_fsend("interval=%lu", measurement_interval);
}

uint16_t validate_and_atoi(const char *str, size_t length) {
    uint16_t result = 0;

    // sprawdzanie czy kazdy znak to cyfra
    for (size_t i = 0; i < length; i++) {
        if (str[i] < '0' || str[i] > '9') {
            //USART_fsend("invalid char '%c' during atoi\n", str[i]);
            return 0; // kod bledu
        }
        result = (result * 10) + (str[i] - '0');
    }

    return result;
}

//void err01() {
//
//}

void process_frame() {

	if (frame.length_int < 5 || frame.length_int > 256) {
		//err01();
		return;
	}

	uint8_t length = frame.length_int - frame.masked_counter; // dla odkodowanej ramki dlugosc musi byc pomniejszona o ilosc zamaskowanych znakow


	if (strncmp((char *)frame.data, "READ", 4) == 0) {

		char *start_ptr = (char *)&frame.data[4];
		char *dash_ptr = strchr(start_ptr, '-');

		if (!dash_ptr) {
			USART_fsend("brak myslnika");
			//err03();
			return;
		}

		char start_str[4] = {0};
		char count_str[3] = {0};

		size_t start_length = dash_ptr - (char *)frame.data - 4;
		if (start_length >= sizeof(start_str)) {
			USART_fsend("zly parametr");
			//err03();
			return;
		}
		memcpy(start_str, frame.data + 4, start_length);

		size_t count_length = (uint8_t *)frame.data + length - ((uint8_t *)dash_ptr + 1);
		if (count_length >= sizeof(count_str)) {
			//USART_fsend("zly parametr");
			//err03();
			return;
		}
		memcpy(count_str, dash_ptr + 1, count_length);

		uint8_t start = validate_and_atoi(start_str, start_length);
		uint8_t count = validate_and_atoi(count_str, count_length);

		if (start < 1 || start > 750 || count < 1 || count > 21 || (start + count - 1) > 750) {
			//USART_fsend("zly parametr");
			//err03();
			return;
		}

		//USART_fsend("READ(%d - %d)", start, count);
		return;
	}
	else if (strncmp((char *)frame.data, "COUNT_DATA", 10) == 0) {
		if (length != 10) {
			//USART_fsend("wrong command");
			//err02();
			return;
		}
//		else {
//			count_data();
//			return;
//		}
	}

	else if (strncmp((char *)frame.data, "SET_INTERVAL", 12) == 0) {
		if (length < 16 || length > 22) {
			//USART_fsend("wrong command");
			//err02();
			return;
		}

		char *numberStr = (char *)&frame.data[12];
		uint8_t numberLength = length - 12;

		for (uint8_t i = 0; i < numberLength; i++){
			if (!isdigit((unsigned char)numberStr[i])) {
				//err03();
				return;
			}
		}

		char *endptr;
		errno = 0; // globalna zmienna
		uint32_t interval = strtoul(numberStr, &endptr, 10);

//		if (*endptr != '\0') {
//			//err03();
//			//USART_fsend("blad przy konwersji");
//			return;
//		}

		if (*endptr == '\0' || interval < 2000 || errno == ERANGE) {
			//err03();
			return;
		}

		set_interval(interval);
		//USART_fsend("interval: %lu ", interval);
		return;
	}

	else if (strncmp((char *)frame.data, "GET_INTERVAL", 12) == 0) {
		if (frame.length_int != 12) {
			//USART_fsend("wrong command");
			//err02();
			return;
		}
		get_interval();
		return;

	}
}

void reset_frame() {
	memset(&frame, 0, sizeof(Frame));
	frame.state = IDLE;
	frame.crc_calculated = 0xFFFF;
}

void get_frame(uint8_t ch) {

	if (ch == FRAME_START) {
		reset_frame();
		frame.state = FIND_SENDER;
		return;
	}
	if (ch == '\0') {
		frame.state = IDLE;
		return;
	}

	switch (frame.state) {
	case IDLE: {
		return;
	}

	case FIND_SENDER: {
		if (ch >= 'A' && ch <= 'Z') {
			frame.sender[frame.sender_id] = ch;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);
			if (frame.sender_id == 1) {
				frame.sender[2] = '\0';
				if (strncmp((char *)frame.sender, SENDER, 2) == 0){
					//USART_fsend("sender ok");
					frame.state = FIND_RECEIVER;
					return;
				}
				else {
					frame.state = IDLE;
					return;
				}
			}
			else frame.sender_id++;
		}
		else frame.state = IDLE;
		return;
	}

	case FIND_RECEIVER: {
		if (ch >= 'A' && ch <= 'Z') {
			frame.receiver[frame.receiver_id] = ch;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);
			if (frame.receiver_id == 1) {
				frame.receiver[2] = '\0';
				if (strncmp((char *)frame.receiver, RECEIVER, 2) == 0) {
					//USART_fsend("receiver ok");
					frame.state = FIND_LENGTH;
					return;
				}
				else {
					frame.state = IDLE;
					return;
				}
			}
			else frame.receiver_id++;
		}
		else frame.state = IDLE;
		return;
	}

	case FIND_LENGTH: {
		if (ch >= '0' && ch <= '9') {
			frame.length[frame.length_id] = ch;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);
			if (frame.length_id == 2) {
				frame.length[3] = '\0';
				frame.length_int = atoi((char *)frame.length);
				//USART_fsend("length ok");
				frame.state = FIND_DATA;
				return;
			}
			else frame.length_id++;
		}
		else frame.state = IDLE;
		return;
	}

	case FIND_DATA: {

		if (frame.data_id + frame.masked_counter < frame.length_int) {

			if (ch == FRAME_START || ch == FRAME_END) {
				frame.state = IDLE;
				return;
			}

			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);

			if (ch == MASK) {
				frame.masked_counter++;
				frame.state = FIND_MASKED;
				return;
			}

			frame.data[frame.data_id++] = ch;

			if (frame.data_id + frame.masked_counter == frame.length_int) {
				frame.data[frame.data_id] = '\0';
				USART_fsend("data ok");
				frame.state = FIND_CRC;
			}
			return;
		}
		else {
			frame.state = IDLE;
			return;
		}
	}

	case FIND_MASKED: {

		if (frame.data_id + frame.masked_counter >= frame.length_int) {
			frame.state = IDLE;
			return;
		}

		switch(ch) {
		case MASKED_START: {
			frame.data[frame.data_id++] = FRAME_START;
			frame.state = FIND_DATA;
			break;
		}
		case MASKED_END: {
			frame.data[frame.data_id++] = FRAME_END;
			frame.state = FIND_DATA;
			break;
		}
		case MASK: {
			frame.data[frame.data_id++] = MASK;
			frame.state = FIND_DATA;
			break;
		}
		default: { // błąd, powrót do początku
			frame.state = IDLE;
			return;
		}
		}

		frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);

		if (frame.data_id < frame.length_int - 1) {
			frame.state = FIND_DATA;
		} else {
			frame.data[frame.data_id] = '\0';
			frame.state = FIND_CRC;
		}

		return;
	}

	case FIND_CRC: {
		if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')) {
			frame.crc_frame[frame.crc_id++] = ch;
			if (frame.crc_id == 4) {
				frame.crc_frame[4] = '\0';
				if ((uint16_t)strtol((char *)frame.crc_frame, NULL, 16) == frame.crc_calculated) {
					//USART_fsend("crc ok");
					frame.state = FIND_END;
					return;
				}
				else {
					//USART_fsend("crc blad");
					frame.state = IDLE;
					return;
				}
			}
		}
		else frame.state = IDLE;
		return;
	}

	case FIND_END: {
		if (ch == FRAME_END) {
			frame.complete = true;
			USART_fsend("ramka ok");
			process_frame();
			return;
		}
		else frame.state = IDLE;
		return;
	}
	}
}

void handle_char() {
	int16_t ch;
	if ((ch = USART_getchar()) >= 0) {
		//USART_fsend("  |%c|  ", ch);
		get_frame((uint8_t)ch);
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
  init_dht11(pDHT, GPIOA, GPIO_PIN_0, &htim2);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init(); // pwm input
  MX_TIM3_Init(); // delay us, count us
  MX_TIM4_Init(); // interval ms
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart2, &USART_BUF_RX[USART_RX_EMPTY], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // flaga z przerwania tim4
	  if (DHT11_READ_FLAG) {
		  readDHT11(pDHT);
		  DHT11_READ_FLAG = 0;
	  }
	  // jeśli bufor nie jest pusty
	  if (USART_RX_EMPTY != USART_RX_BUSY) {
		  handle_char();
	  }


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
	USART_fsend("problem");
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
