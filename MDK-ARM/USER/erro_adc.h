#ifndef _ERRO_ADC_H
#define _ERRO_ADC_H

#include "stdio.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include <math.h>

void error(void);
long hexToDec(char *source);

	
extern uint8_t RE_Buff[4];
extern double  DAC_data;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

#endif

//			//DAC_data:实际要输出的控制电压数据，该值可做校准用	
//		
//		if(RE_Buff[2]<=0x09) //x.0y电压范围校准
//		{
//			if(RE_Buff[1]==0x01||RE_Buff[1]==0x02||RE_Buff[1]==0x03)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.018; }
//				
//			else if(RE_Buff[1]==0x04||RE_Buff[1]==0x06)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.012; }
//				
//			else if(RE_Buff[1]==0x05)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.01; }
//				
//			else if(RE_Buff[1]==0x07)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.020; }
//				
//			else if(RE_Buff[1]==0x08)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.028; }
//				
//			else if(RE_Buff[1]==0x09)
//				{	DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.033; }
//			
//			printf("******09**********\r\n");
//		}
//		
//		else if(0x09<RE_Buff[2]&&RE_Buff[2]<=0x19)	//x.1y电压范围校准
//		{
//			if(RE_Buff[1]==0x08||RE_Buff[1]==0x09)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.087;
//			}
//			else if(RE_Buff[1]==0x07)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.078;
//			}
//			else	if(RE_Buff[1]==0x05||RE_Buff[1]==0x06)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.072;
//			}	
//			else  if(RE_Buff[1]==0x01||RE_Buff[1]==0x02||RE_Buff[1]==0x03||RE_Buff[1]==0x04)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.076;
//			}
//		}
//		
//		else if(0x19<RE_Buff[2]&&RE_Buff[2]<=0x29)	//x.2y电压范围校准
//		{
//			if(RE_Buff[1]==0x08)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.147;
//			}	
//			else	if(RE_Buff[1]==0x09)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.149;
//			}	
//			else	if(RE_Buff[1]==0x05||RE_Buff[1]==0x06)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.130;
//			}	
//			else  if(RE_Buff[1]==0x04)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.136;
//			}
//			else  if(RE_Buff[1]==0x01||RE_Buff[1]==0x02||RE_Buff[1]==0x03||RE_Buff[1]==0x07)
//			{
//				DAC_data=(double)((RE_Buff[1]*100+RE_Buff[2])/100.0)-0.138;
//			}
//		
//		}
//		
//		else 
//==============================================
