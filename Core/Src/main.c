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
volatile uint8_t is_handling = 0; // flaga sprawdzająca, czy jest aktualnie obsługiwany jakiś znak


/* Stany */
typedef enum {
	FIND_START,
	FIND_SENDER,
	FIND_RECEIVER,
	FIND_LENGTH,
	FIND_DATA,
	FIND_CRC,
	FIND_END,
	FIND_MASKED,
	FRAME_ERROR
} FrameState;

/* Struktura ramki */
typedef struct {
	uint8_t sender[3];
	uint8_t receiver[3];
	uint8_t length[4];
	uint8_t data_raw[257];
	uint8_t data[257];
	uint16_t crc;

	uint8_t sender_id;
	uint8_t receiver_id;
	uint8_t length_id;
	uint16_t data_id;
	uint16_t data_raw_id;
	uint8_t crc_id;

	uint16_t length_int;
	uint16_t masked_counter;
	FrameState state;
	bool complete;
} Frame;

/* Inicjalizacja */
Frame frame;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

uint16_t calculate_frame_crc(Frame *frame) {
    uint16_t crc = 0x0000; // Początkowa wartość CRC

    // Oblicz CRC dla pola receiver
    crc = calculate_crc((const char *)frame->receiver, sizeof(frame->receiver)) ^ crc;

    // Oblicz CRC dla pola sender
    crc = calculate_crc((const char *)frame->sender, sizeof(frame->sender)) ^ crc;

    // Oblicz CRC dla pola length
    crc = calculate_crc((const char *)frame->length, sizeof(frame->length)) ^ crc;

    // Oblicz CRC dla pola data
    crc = calculate_crc((const char *)frame->data_raw, sizeof(frame->data_raw)) ^ crc;

    return crc;
}

char* get_crc_hex(const char *input) {
	char tmp[257]; // tymczasowa tablica
	strncpy(tmp, input, 256);
	tmp[257] = '\0';
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
uint8_t usart_tx_has_data() {
    return USART_TX_EMPTY != USART_TX_BUSY;
}

uint8_t usart_rx_has_data() {
    return USART_RX_EMPTY != USART_RX_BUSY;
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



uint16_t validate_and_atoi(const char *str, size_t length) {
    uint16_t result = 0;

    // sprawdzanie czy kazdy znak to cyfra
    for (size_t i = 0; i < length; i++) {
        if (str[i] < '0' || str[i] > '9') {
            USART_fsend("invalid char '%c' during atoi\n", str[i]);
            return 65535; // maksymalna wartosc uint16 jako kod bledu
        }
        result = (result * 10) + (str[i] - '0');
    }

    return result;
}

void process_frame() {

	if (frame.crc != calculate_frame_crc(&frame)) return;

	//uint16_t length = (atoi(frame.length[0]) * 100) + (atoi(frame.length[1] * 10)) + atoi(frame.length[3]);

	char length_str[4] = {frame.length[0], frame.length[1], frame.length[3], '\0'};
	uint16_t length = atoi(length_str);

	if (length < 5 || length > 256) {
		USART_fsend("wrong length");
		//err01();
		return;
	}

	length -= frame.masked_counter; // dla odkodowanej ramki dlugosc musi byc pomniejszona o ilosc zamaskowanych znakow


	if (strncmp((char *)frame.data, "READ", 4) == 0) {
		if (length != 7) {
			USART_fsend("wrong parameter");
			//err03();
			return;
		}

		char parameter_str[4] = {frame.data[4], frame.data[5], frame.data[6]};
		uint16_t parameter = validate_and_atoi(parameter_str, 3);

		if (parameter < 1 || parameter > 750) {
			printf("wrong parameter");
			//err03();
			return;
		}
//		else if (parameter < dht_data_counter) {
//			printf ("not enough data in buffer");
//			err04();
//			return;
//		}
//		else {
//			read(length);
//			return;
//		}
	}
	else if (strncmp((char *)frame.data, "COUNT_DATA", 10) == 0) {
		if (frame.length_int != 10) {
			USART_fsend("wrong command");
			//err02();
			return;
		}
//		else {
//			count_data();
//			return;
//		}
	}

	else if (strncmp((char *)frame.data, "SET_INTERVAL", 12) == 0) {
		if (frame.length_int != 17) {
			USART_fsend("wrong command");
			//err02();
			return;
		}

		char parameter_str[6] = { frame.data[12], frame.data[13], frame.data[14], frame.data[15], frame.data[16], '\0' };
		uint16_t parameter = validate_and_atoi(parameter_str, 5);

		if (parameter < 2000 || parameter > 20000) {
			USART_fsend("wrong parameter");
			//err03();
			return;
		}
//		else {
//			set_interval(parameter);
//			return;
//		}
	}

	else if (strncmp((char *)frame.data, "GET_INTERVAL", 12) == 0) {
		if (frame.length_int != 12) {
			USART_fsend("wrong command");
			//err02();
			return;
		}
//		else {
//			get_interval();
//			return
//		}
	}
}

void reset_frame() {
	memset(&frame, 0, sizeof(Frame));
	frame.state = FIND_START;
}

void get_frame(uint8_t ch) {
	switch (frame.state) {

	case FIND_START: {
		if (ch == FRAME_START) {
			reset_frame();
			frame.state = FIND_SENDER;
		}
		return;
	}

	case FIND_SENDER: {
		if (ch >= 'A' && ch <= 'Z') {
			frame.sender[frame.sender_id] = ch;
			if (frame.sender_id == 1) {
				frame.sender[2] = '\0';
				if (strncmp((char *)frame.sender, SENDER, 2) == 0){
//					USART_fsend("sender ok");
					frame.state = FIND_RECEIVER;
					return;
				}
				else frame.state = FRAME_ERROR;
			}
			else frame.sender_id++;
		}
		else if (ch == FRAME_START || ch == FRAME_END) frame.state = FIND_START;
		else frame.state = FRAME_ERROR;
		break;
	}

	case FIND_RECEIVER: {
		if (ch >= 'A' && ch <= 'Z') {
			frame.receiver[frame.receiver_id] = ch;
			if (frame.receiver_id == 1) {
				frame.receiver[2] = '\0';
				if (strncmp((char *)frame.receiver, RECEIVER, 2) == 0) {
//					USART_fsend("receiver ok");
					frame.state = FIND_LENGTH;
					return;
				}
				else frame.state = FRAME_ERROR;
			}
			else frame.receiver_id++;
		}
		else if (ch == FRAME_START || ch == FRAME_END) frame.state = FIND_START;
		else frame.state = FRAME_ERROR;
		break;
	}

	case FIND_LENGTH: {
		if (ch >= '0' && ch <= '9') {
			frame.length[frame.length_id] = ch;
			if (frame.length_id == 2) {
				frame.length[3] = '\0';
				frame.length_int = atoi((char *)frame.length);
				frame.state = FIND_DATA;
				return;
			}
			else frame.length_id++;
		}
		else if (ch == FRAME_START || ch == FRAME_END) frame.state = FIND_START;
		else frame.state = FRAME_ERROR;
		break;
	}

	case FIND_DATA: {
		frame.data_raw[frame.data_raw_id++] = ch;

		if (frame.data_id < frame.length_int) {
			if (ch == MASK) {
				if (frame.data_id < frame.length_int - 1) {
					frame.state = FIND_MASKED;
					return;
				}
				else {	// jeśli znak maskujący jest ostatnim w danych, wtedy błąd
					frame.state = FRAME_ERROR;
				}
			}
			else if (ch == FRAME_START || ch == FRAME_END) {
				frame.state = FIND_START;
				return;
			}
			else {
				frame.data[frame.data_id] = ch;
				frame.data_id++;
				break;
			}
		}
		else {
			frame.data_raw[frame.data_raw_id] = '\0';
			frame.data[frame.data_id] = '\0';
			frame.state = FIND_CRC;
			return;
		}
	}

	case FIND_MASKED: {
		frame.masked_counter++;
		frame.data_raw[frame.data_raw_id++] = ch;
		switch(ch) {
		case MASKED_START: {
			frame.data[frame.data_id++] = ':';
			frame.state = FIND_DATA;
		}
		case MASKED_END: {
			frame.data[frame.data_id++] = ';';
			frame.state = FIND_DATA;
		}
		case MASK: {
			frame.data[frame.data_id++] = '/';
			frame.state = FIND_DATA;
		}
		default: {
			frame.state = FRAME_ERROR;
			return;
		}
		}
		if (frame.data_id < frame.length_int) {
			frame.state = FIND_DATA;
			return;
		} else {
			frame.data_raw[frame.data_raw_id] = '\0';
			frame.data[frame.data_id] = '\0';
			frame.state = FIND_CRC;
			return;
		}
	}

	case FIND_CRC: {
		if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')) {
			uint8_t value;
			if (ch >= '0' && ch <= '9') value = ch - '0';
			else value = ch - 'A' + 10;

			switch(frame.crc_id) {
			case 0: {
				frame.crc = value << 12;
				break;
			}
			case 1: {
				frame.crc |= value << 8;
				break;
			}
			case 2: {
				frame.crc |= value << 4;
				break;
			}
			case 3: {
				frame.crc |= value;
				frame.state = FIND_END;
				return;
			}
			}
			frame.crc_id++;
			break;
		}
		else if (ch == FRAME_START || ch == FRAME_END) frame.state = FIND_START;
		else frame.state = FRAME_ERROR;
		break;
	}

	case FIND_END: {
		if (ch == FRAME_END) {
			frame.complete = true;
			process_frame();
			return;
		}
		else if (ch == FRAME_START) frame.state = FIND_START;
		else frame.state = FRAME_ERROR;
		break;
	}

	case FRAME_ERROR: {
//		USART_fsend("error during frame processing");
		frame.state = FIND_START;
		return;
	}

	}
}

void handle_char() {
	//__disable_irq();
	is_handling = 1;
	//__enable_irq();

	int16_t ch;
	if ((ch = USART_getchar()) >= 0 && ch < 128) {
		USART_fsend("  |%c|  ", ch);
		get_frame((uint8_t)ch);
	}

	//__disable_irq();
	is_handling = 0;
	//__enable_irq();
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
	  if (USART_RX_EMPTY != USART_RX_BUSY && !is_handling) {
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
