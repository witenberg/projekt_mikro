/* USER CODE BEGIN Header */
/**
  **************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  **************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
#define USART_TXBUF_LEN 1512 //Liczba bajtów bufora nadawczego
#define USART_RXBUF_LEN 128 //Liczba bajtów bufora odbiorczego
uint8_t USART_TxBuf[USART_TXBUF_LEN];//tworzenie bufora nadawczego
uint8_t USART_RxBuf[USART_RXBUF_LEN];//tworzenie bufora odbiorczego


__IO int USART_TX_Empty=0; //wskaźniki zapisu (empty) i odczytu(busy)
__IO int USART_TX_Busy=0;
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t USART_kbhit(){ //Sprawdzenie czy tablica przypadkiem nie jest pusta
	if(USART_RX_Empty==USART_RX_Busy){
		return 0;
	}else{
		return 1;
	}
}//USART_kbhit

int16_t USART_getchar(){//pobranie znaku
int16_t tmp;
	if(USART_RX_Empty!=USART_RX_Busy){//Sprawdzenie czy tablica przypadkiem nie jest pusta
		 tmp=USART_RxBuf[USART_RX_Busy];//odczyt
		 USART_RX_Busy++; //przejscie na kolejny indeks odczytu
		 if(USART_RX_Busy >= USART_RXBUF_LEN)USART_RX_Busy=0;//Sprawdzenie czy nie przekroczyło długośći bufora odczytu
		 return tmp;
	}else return -1;
}//USART_getchar

uint8_t USART_getline(char *buf){
static uint8_t bf[128];
static uint8_t idx=0;
int i;
uint8_t ret;
 while(USART_kbhit()){ //Sprawdza czy wskaznik odczytu i zapisu są na tym samym miejscu
      bf[idx]=USART_getchar();
     // USART_fsend("[%02X]", bf[idx]);
      if(((bf[idx]==10)||(bf[idx]==13))){
    	  bf[idx]=0;
    	  for(i=0;i<=idx;i++){
    	     buf[i]=bf[i];//przekopiuj do bufora
    	  }
    	  ret=idx;
    	  idx=0;
    	  return ret;//odebrano linie
      }else{//jesli tekst dluzszy to zawijamy - trudno
    	  idx++;
    	  if(idx>=128)idx=0;
      }
  }
  return 0;
}//USART_getline

void USART_fsend(char* format,...){
char tmp_rs[128];
int i;
__IO int idx;
va_list arglist;
  va_start(arglist,format);
  vsprintf(tmp_rs,format,arglist);
  va_end(arglist);
  idx=USART_TX_Empty;
  for(i=0;i<strlen(tmp_rs);i++){
	  USART_TxBuf[idx]=tmp_rs[i];
	  idx++;
	  if(idx >= USART_TXBUF_LEN)idx=0;
  }
  __disable_irq();//wyłączamy przerwania
  if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){//sprawdzic dodatkowo zajetosc bufora nadajnika
	  USART_TX_Empty=idx;
	  uint8_t tmp=USART_TxBuf[USART_TX_Busy];
	  USART_TX_Busy++;
	  if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
	  HAL_UART_Transmit_IT(&huart2, &tmp, 1);
  }else{
	  USART_TX_Empty=idx;
  }
  __enable_irq();
}//fsend

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
   if(huart==&huart2){
	   if(USART_TX_Empty!=USART_TX_Busy){
		   uint8_t tmp=USART_TxBuf[USART_TX_Busy];
		   USART_TX_Busy++;
		   if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
		   HAL_UART_Transmit_IT(&huart2, &tmp, 1);//informacja zwortna-informuję o tym ze dany bajt został już wysłany i ze mozna odebrać następny
	   }
   }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart==&huart2){
		 USART_RX_Empty++;
		 if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0;
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1);

	 }
}

void USART2_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(_FILE, LINE);
  }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,&USART_RxBuf[0],1);

    USART_fsend("START %03d\r\n",72);
    USART_fsend("START1 0x%04X\r\n",72);
    USART_fsend("START2 %03d\r\n",72);
  uint32_t xxx=0;
  int len=0;
  char bx[200];
  uint32_t cntr=0;
    while (1)
    {
  	  cntr++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	  if((len= USART_getline(bx))>0){
  		  USART_fsend("REC[%d]> %s\r\n",len,bx);
  		  switch(bx[0]){
  		  case 'l':
  		  case 'L':
  			  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  			  USART_fsend("zmiana stanu diody\r\n");
  			  break;
  		  case 'S':
  		  case 's':
  			  USART_fsend("STAN SWITCHA USER %d\r\n",HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13));
  			  break;
  		  case '?':
  			  USART_fsend("LOOP CNT %ld\r\n",cntr);
  			  break;
  		  }//switch
  	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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