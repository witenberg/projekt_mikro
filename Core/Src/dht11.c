/*
 * dht11.c
 *
 *  Created on: Jan 11, 2025
 *      Author: witenberg
 */
#include "dht11.h"

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
    uint8_t bitIndex = 0;
    uint16_t high_time = 0;
//    uint16_t low_time = 0;

    // Start komunikacji
    set_dht11_gpio_mode(dht, OUTPUT);

    __disable_irq();
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET); // Stan niski na 18ms
    delay_us(18000);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_SET);
    delay_us(20);
    set_dht11_gpio_mode(dht, INPUT);


    count_us();
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(&htim3) >= 2000) {
            __enable_irq();
            return 1; // Timeout
        }
    }

    count_us();
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET) {
        if (__HAL_TIM_GET_COUNTER(&htim3) >= 2000) {
        	//USART_fsend("timeout2");
            __enable_irq();
            return 1; // Timeout
        }
    }

    count_us();
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(&htim3) >= 2000) {
        	//USART_fsend("timeout3");
            __enable_irq();
            return 1; // Timeout
        }
    }

    HAL_TIM_PWM_Start(dht->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(dht->htim, TIM_CHANNEL_2);

    for (bitIndex = 0; bitIndex < 40; bitIndex++) {
        __HAL_TIM_CLEAR_FLAG(dht->htim, TIM_FLAG_CC1 | TIM_FLAG_CC2);

        count_us();
        while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET) {
            if (__HAL_TIM_GET_COUNTER(&htim3) >= 1000) {
                //USART_fsend(" timeout i=%u ", bitIndex);
                __enable_irq();
                return 1;
            }
        }
//        low_time = __HAL_TIM_GET_COMPARE(dht->htim, TIM_CHANNEL_1);

        count_us();
        while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
            if (__HAL_TIM_GET_COUNTER(&htim3) >= 1000) {
                //USART_fsend(" timeout i=%u ", bitIndex);
                __enable_irq();
                return 1;
            }
        }
        high_time = __HAL_TIM_GET_COMPARE(dht->htim, TIM_CHANNEL_2);

        //USART_fsend("(%u - %u) ", low_time, high_time);

        if (high_time > 50) {
            data[bitIndex / 8] |= (1 << (7 - (bitIndex % 8))); // Ustawienie bitu
        } else {
            data[bitIndex / 8] &= ~(1 << (7 - (bitIndex % 8))); // Wyzerowanie bitu
        }
    }
    USART_fsend("DHT11 Data: Humidity=%u.%u%%, Temp=%u.%u ", data[0], data[1], data[2], data[3]);
    __enable_irq();

    return 0;
}

