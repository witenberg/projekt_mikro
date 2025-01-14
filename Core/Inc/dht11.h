/*
 * dht11.h
 *
 *  Created on: Jan 11, 2025
 *      Author: witenberg
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "gpio.h"
#include "main.h"
#include "stm32f1xx_it.h"
#include "tim.h"

#define OUTPUT 1
#define INPUT 0

#define DHT11_BUF_SIZE 750

typedef struct {
	GPIO_TypeDef* port; // gpio port dla dht (tutaj gpioa)
	uint16_t pin; // gpio pin dla dth (tutaj pa0)
	TIM_HandleTypeDef *htim; // timer dla dht (tutaj htim2)

	uint8_t buf[DHT11_BUF_SIZE][4];
	uint16_t empty;
	uint16_t busy;
	uint16_t count;
} dht11_t;

void init_dht11(dht11_t *dht, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef *htim);
void add_to_dht11_buf(dht11_t *dht, uint8_t data[4]);
void set_dht11_gpio_mode(dht11_t *dht, uint8_t mode);
uint8_t readDHT11(dht11_t *dht);

void USART_fsend(char* format, ...);
void delay_us(uint16_t us);
void count_us();


#endif /* INC_DHT11_H_ */
