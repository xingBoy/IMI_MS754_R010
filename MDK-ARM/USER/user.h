#ifndef _USER_H
#define _USER_H

#include "stdio.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include <math.h>
#include "erro_adc.h"

void Cont_Out3(void);
void Read_In3(void);
void Cont_Dac(void);
void DAC_cle(void);
void PWM(void);
void error(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

extern void MX_USART1_UART_Init(void);
extern void MX_TIM14_Init(uint16_t fre,uint8_t cycle,uint16_t per);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim14;
extern uint8_t RE_Buff[4];

/**/

#endif

