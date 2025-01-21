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
volatile uint32_t systick_counter = 0;
uint32_t measurement_interval = 5000; // domyślna wartość interwału ustawiona na 5 sekund


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
dht11_t *pDHT = &dht; // wskaźnik do struktury dht11

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* === CRC === */
uint16_t calculate_crc_byte(uint16_t crc, uint8_t data) {
	crc ^= (data << 8); // xor aktualnej sumy z nowym bajtem przesuniętym o 8 bitów w lewo

	for (uint8_t i = 0; i < 8; i++) { // iteracja po każdym bicie nowego bajtu
		if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; // jeśli najbardziej znaczący bit crc jest równy 1,
		else crc <<= 1;								// xor przesuniętego crc z wielomianem, jeśli nie - samo przesunięcie
	}
	return crc; // zwrócenie sumy bez końcowego xor
}

uint16_t calculate_crc_string(const char *data) { // obliczanie sumy crc dla całego ciągu znaków
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < strlen(data); i++) {
        crc = calculate_crc_byte(crc, (uint8_t)data[i]);
    }

    return crc;
}


void USART_fsend(char* format, ...) {
	char tmp[265]; // maks rozmiar ramki
	va_list arglist; // arglist - wskaznik do struktury z przekazanymi argumentami
	va_start(arglist, format); // ustawienie wskaznika w arglist na pierwszy zmienny argument (pierwszy po format)
	vsprintf(tmp, format, arglist); // wstawienie do tmp formatowanego lancucha uzupelnionego o argumenty z arglist
	va_end(arglist); // koniec przetwarzania, zwolnienie zasobow
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


void send_read(uint16_t start, uint8_t count) {
    char frame[265] = {0};       // bufor dla pełnej ramki odpowiedzi (max długość: 251 danych + nagłówek + CRC)
    char measurement[11] = {0}; // bufor na jeden pomiar (stała długość: 10 znaków + '\0')
    uint16_t crc;               // suma crc
    uint16_t length = count * 10 + 1; // długość pola danych w ramce

    uint16_t offset = snprintf(frame, sizeof(frame), ":MCPC%03u|", length);

    // iteracja od najnowszego pomiaru do najstarszego
    for (uint8_t i = 0; i < count; i++) {
        uint16_t index = (pDHT->empty - start - i + DHT11_BUF_SIZE) % DHT11_BUF_SIZE;

        // pobranie danych pomiaru z bufora
        uint8_t humidity_int = pDHT->buf[index][0];
        uint8_t humidity_frac = pDHT->buf[index][1];
        uint8_t temp_int = pDHT->buf[index][2];
        uint8_t temp_frac = pDHT->buf[index][3];

        // formatowanie pomiaru w formacie "XX,X\YY,Y|"
        snprintf(measurement, sizeof(measurement), "%02u,%01u\\%02u,%01u|",
                 humidity_int % 100, humidity_frac % 10, temp_int % 100, temp_frac % 10);

        // dodanie do ramki
        strcat(frame + offset, measurement);
        offset += strlen(measurement);
    }

    // obliczenie CRC dla ramki (bez ':', CRC i ';')
    crc = calculate_crc_string(frame + 1);
    snprintf(frame + offset, sizeof(frame) - offset, "%04X;", crc);
    USART_fsend("%s", frame);
}


void set_interval(uint32_t interval) {
	systick_counter = 0;
	measurement_interval = interval;
	USART_fsend(":MCPC002OK06C5;");
}

//:MCPCx
void get_interval() {
    char frame[25] = {0};   // bufor na pełną ramkę
    char data[19] = {0};    // bufor dla danych używanych do obliczeń CRC
    uint16_t crc;     // suma CRC
    char number_str[11] = {0};

    snprintf(number_str, sizeof(number_str), "%lu", measurement_interval);
    uint8_t number_length = strlen(number_str);
    snprintf(data, sizeof(data), "MCPC%03u%lu", number_length, measurement_interval);
    crc = calculate_crc_string(data);
    snprintf(frame, sizeof(frame), ":%s%04X;", data, crc);

    USART_fsend("%s", frame);
}

void get_count_data() {
    char frame[20] = {0};       // bufor na ramke
    char data[14] = {0};        // bufor dla danych używanych do obliczeń CRC
    uint16_t crc;               // suma CRC
    char number_str[6] = {0};   // bufor do przechowywania liczby jako napisu

    snprintf(number_str, sizeof(number_str), "%u", pDHT->count);
    uint8_t number_length = strlen(number_str);
    snprintf(data, sizeof(data), "MCPC%03u%s", number_length, number_str);
    crc = calculate_crc_string(data);
    snprintf(frame, sizeof(frame), ":%s%04X;", data, crc);

    USART_fsend("%s", frame);
}


uint16_t validate_and_atoi(const char *str, size_t length) {
    uint16_t result = 0;

    // sprawdzanie czy kazdy znak to cyfra
    for (size_t i = 0; i < length; i++) {
        if (str[i] < '0' || str[i] > '9') {
            return 0; // kod bledu
        }
        result = (result * 10) + (str[i] - '0');
    }
    return result;
}

void err(uint8_t error_number) {
    if (error_number < 1 || error_number > 4) {
        return;
    }
    char message[19] = {0};
    uint16_t crc;

    switch(error_number) {
    case 1:
    	crc = 0x2E77;
    	break;
    case 2:
    	crc = 0x1E14;
    	break;
    case 3:
    	crc = 0x0E35;
    	break;
    case 4:
    	crc = 0x7ED2;
    	break;
    }
    sprintf(message, ":MCPC005ERR0%u%04X;", error_number, crc);
    USART_fsend("%s", message);
}

void process_frame() {

	if (frame.length_int < 5 || frame.length_int > 256) {
		err(1);
		return;
	}

	// dla odkodowanej ramki dlugosc musi byc pomniejszona o ilosc zamaskowanych znakow
	uint8_t length = frame.length_int - frame.masked_counter;

	if (strncmp((char *)frame.data, "READ", 4) == 0) {

		char *start_ptr = (char *)&frame.data[4];
		char *dash_ptr = strchr(start_ptr, '-');

		if (!dash_ptr) {
			err(3);
			return;
		}

		char start_str[4] = {0};
		char count_str[3] = {0};

		size_t start_length = dash_ptr - (char *)frame.data - 4;
		if (start_length >= sizeof(start_str)) {
			err(3);
			return;
		}
		memcpy(start_str, frame.data + 4, start_length);

		size_t count_length = (uint8_t *)frame.data + length - ((uint8_t *)dash_ptr + 1);
		if (count_length >= sizeof(count_str)) {
			err(3);
			return;
		}
		memcpy(count_str, dash_ptr + 1, count_length);

		uint16_t start = validate_and_atoi(start_str, start_length);
		uint16_t count = validate_and_atoi(count_str, count_length);

		if (start < 1 || start > 750 || count < 1 || count > 25 || (start + count - 1) > 750) {
			err(3);
			return;
		}

		if (start + count - 1 > pDHT->count) {
			err(4);
			return;
		}

		send_read(start, count);
		return;
	}
	else if (strncmp((char *)frame.data, "COUNT_DATA", 10) == 0) {
		if (length != 10) err(2);
		else get_count_data();
		return;
	}

	else if (strncmp((char *)frame.data, "SET_INTERVAL", 12) == 0) {
		if (length < 16 || length > 22) {
			err(2);
			return;
		}

		char *numberStr = (char *)&frame.data[12];
		uint8_t numberLength = length - 12;

		for (uint8_t i = 0; i < numberLength; i++){
			if (!isdigit((uint8_t)numberStr[i])) {
				err(1);
				return;
			}
		}

		char *endptr;
		errno = 0; // globalna zmienna
		uint32_t interval = strtoul(numberStr, &endptr, 10);

		if (*endptr != '\0' || interval < 2000 || errno == ERANGE) {
			err(3);
			return;
		}

		set_interval(interval);
		return;
	}

	else if (strncmp((char *)frame.data, "GET_INTERVAL", 12) == 0) {
		if (frame.length_int != 12) {
			err(2);
			return;
		}
		get_interval();
		return;

	}

	else err(2);
}

void reset_frame() {
	memset(&frame, 0, sizeof(Frame));
	frame.state = IDLE;
	frame.crc_calculated = 0xFFFF;
}

void get_frame() {

	int16_t ch;
	if ((ch = USART_getchar()) <= 0) {
		frame.state = IDLE;
		return;
	} else ch = (uint8_t)ch;

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
//					USART_fsend("sender ok");
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
//					USART_fsend("receiver ok");
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
//				USART_fsend("length ok");

				if (frame.length_int == 0) {
					frame.state = FIND_CRC;
					return;
				}
				frame.state = FIND_DATA;
				return;

			}
			else frame.length_id++;
		}
		else frame.state = IDLE;
		return;
	}

	case FIND_DATA: {

//		if (frame.data_id + frame.masked_counter < frame.length_int) {

			if (ch == FRAME_END) {
				frame.state = IDLE;
				return;
			}

			if (ch == MASK) {
				frame.masked_counter++;
				frame.state = FIND_MASKED;
				return;
			}

			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, ch);
			frame.data[frame.data_id++] = ch;

			if (frame.data_id + frame.masked_counter == frame.length_int) {
				frame.data[frame.data_id] = '\0';
//				USART_fsend("data ok");
				frame.state = FIND_CRC;
			}
			return;
		}
//		else {
//			frame.state = IDLE;
//			return;
//		}

	case FIND_MASKED: {

		if (frame.data_id + frame.masked_counter >= frame.length_int) {
			frame.state = IDLE;
			return;
		}

		switch(ch) {
		case MASKED_START: {
			frame.data[frame.data_id++] = FRAME_START;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, FRAME_START);
			frame.state = FIND_DATA;
			break;
		}
		case MASKED_END: {
			frame.data[frame.data_id++] = FRAME_END;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, FRAME_END);
			frame.state = FIND_DATA;
			break;
		}
		case MASK: {
			frame.data[frame.data_id++] = MASK;
			frame.crc_calculated = calculate_crc_byte(frame.crc_calculated, MASK);
			frame.state = FIND_DATA;
			break;
		}
		default: { // błąd, powrót do początku
			frame.state = IDLE;
			return;
		}
		}

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
//					USART_fsend("crc ok");
					frame.state = FIND_END;
					return;
				}
				else {
//					USART_fsend("crc blad");
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
//			USART_fsend("ramka ok");
			process_frame();
			return;
		}
		else frame.state = IDLE;
		return;
	}
	}
}

//void handle_char() {
//	int16_t ch;
//	if ((ch = USART_getchar()) >= 0) {
//		get_frame((uint8_t)ch);
//	}
//}

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
  //MX_TIM4_Init(); // interval ms
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart2, &USART_BUF_RX[USART_RX_EMPTY], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (systick_counter >= measurement_interval) {
		  systick_counter = 0;
		  readDHT11(pDHT);
	  }

	  // jeśli bufor nie jest pusty
	  if (USART_RX_EMPTY != USART_RX_BUSY) {
		  get_frame();
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
