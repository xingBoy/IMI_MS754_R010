#include "stdio.h"
#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart3;
uint8_t ch;
uint8_t ch_r;

//重写这个函数,重定向printf函数到串口
/*fputc*/
int fputc(int c, FILE * f)
{
    ch=c;
    HAL_UART_Transmit(&huart3,&ch,1,1000);//发送串口
    return c;
}



//重定向scanf函数到串口 意思就是说接受串口发过来的数据
/*fgetc*/
int fgetc(FILE * F)    
{
    HAL_UART_Receive (&huart3,&ch_r,1,0xffff);//接收
    return ch_r;
}

