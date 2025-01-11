/*
 * dht11.c
 *
 *  Created on: Jan 6, 2025
 *      Author: kubaw
 */
#include "dht11.h"
#include "main.h"

void init_dht11(dht11_t *dht, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef *htim){
	dht->htim = htim;
	dht->port = port;
	dht->pin = pin;
	dht->empty = 0;
	dht->busy = 0;
	dht->count = 0;
}

void add_to_dht11_buf(dht11_t *dht, uint8_t data[4]) {
	for (uint8_t i = 0; i < 4; i++)
		dht->buf[dht->empty][i] = data[i];
	dht->empty = (dht->empty + 1) % DHT11_BUF_SIZE;
	if (dht->count < DHT11_BUF_SIZE) dht->count++;
}


void set_dht11_gpio_mode(dht11_t *dht, uint8_t mode) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (mode == OUTPUT) {
		GPIO_InitStruct.Pin = dht->pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
	} else if (mode == INPUT) {
		GPIO_InitStruct.Pin = dht->pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
	}
}

uint8_t readDHT11(dht11_t *dht) {
	uint8_t data[5] = {0};
	uint16_t high_time;

	//start komunikacji
	set_dht11_gpio_mode(dht, OUTPUT);
	__disable_irq();

	HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET); // stan niski na 18ms
	HAL_Delay(18);
	HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_SET);
	set_dht11_gpio_mode(dht, INPUT);


    // Oczekiwanie na zbocze narastające sygnału (sygnał gotowości czujnika)
	while(HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET);
    while(HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET);  // oczekiwanie na zbocze narastające
    while(HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET);    // oczekiwanie na zbocze opadające

    // start PWM Input
    HAL_TIM_IC_Start(dht->htim, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(dht->htim, TIM_CHANNEL_2);

    for (uint8_t i = 0; i < 40; i++) {
    	while(HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET); // oczekiwanie na zbocze narastajace czyli bit

    	// Zresetowanie flag przechwytywania
    	__HAL_TIM_CLEAR_FLAG(dht->htim, TIM_FLAG_CC1 | TIM_FLAG_CC2);

    	while(HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET);

    	uint16_t ccr1 = __HAL_TIM_GET_COMPARE(dht->htim, TIM_CHANNEL_1); // Zbocze narastające
    	uint16_t ccr2 = __HAL_TIM_GET_COMPARE(dht->htim, TIM_CHANNEL_2); // Zbocze opadające

    	if (ccr2 >= ccr1) high_time = ccr2 - ccr1;
    	else high_time = (dht->htim->Init.Period - ccr1) + ccr2 + 1; //jesli timer zaczal liczyc od poczatku

    	data[i / 8] <<= 1;
    	if (high_time > 50) data[i / 8] |= 1;
    }

    __enable_irq();

    if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
    	return 1;
    }

    USART_fsend("otrzymano dane");

    add_to_dht11_buf(dht, data);
    return 0;
}


