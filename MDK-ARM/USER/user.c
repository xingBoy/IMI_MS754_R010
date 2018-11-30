/********************************************************
		File:
					*输出三路控制电压(Cont_Out3)
					*读取3路24V输入(Read_In3)
					*DAC控制输出(Cont_Dac)
					*DAC_cle电压控制输出换算(DAC_cle)
					*PWM控制输出(PWM)
					*串口回调函数重写(HAL_UART_RxCpltCallback)
					
********************************************************/
#include "user.h"

uint8_t 	re_IN_buff[4];  //返回的读取外部电压输入IN状态buff
uint8_t 	re_Out_buff[4]; //返回的控制外部电压输出out状态buff
uint8_t 	data[2];				//SPI传输控制DAC的数据
uint8_t 	zero[4]={0}; 	  //清空缓存数组
uint16_t  Dn=0;						//根据实际控制电压换算出来的要输入的MCP4822的Dx控制值

double    DAC_data=0;		  //串口传输下来要控制的DAC数值大小
char 			uart_flag=0;		//接收完数据后校验数据无误标志位

uint8_t   cycle[20]={	//计数值为100时占空比数组
										  5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};

uint16_t   fre[40]={	//计数值为100时，频率控制数组，有个别频率误差较大用其他数组
										 960,480,320,240,192,160,137,119,106,95,86,79,73,67,63,59,55,
										 52,49,47,45,42,40,39,37,36,35,33,32,31,30,29,28,27,26,26,25,24,24,23
									 };

uint8_t cycle_60[20]={	//计数值为60时占空比数组
											3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60};

uint16_t fre_60[40]={	//计数值为60时，频率控制数组,用来减小19.5K、19K、14k、13.5K、12.5K、11.5K、11k的误差
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,75,72,
											69,24,63,26,58,56,29,30,31,32,33,34,35,36,37,41,40,40};

uint8_t cycle_20[20]={	//计数值为20时占空比数组
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

uint16_t fre_20[40]={	//计数值为20时，频率控制数组,用来减小17K、17.5K、18k的误差
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,75,72,
											69,24,63,26,58,28,29,30,31,32,33,140,136,132,37,41,40,40};
										
/*********************************************
	*函数名：Cont_Out3
	*输  入：无
	*输  出：无
	*说  明：控制3路OUT电压输出，并返回控制状态，
					 拉高打开继电器，输出电压
*********************************************/
void Cont_Out3(void)
{
		if(RE_Buff[1]==1)	//控制OUT1输出
		{
				if(RE_Buff[2]==1)	 //关闭OUT1
				{
					HAL_GPIO_WritePin(GPIOB,OUT1_Pin,GPIO_PIN_RESET); //拉低
				}
				else if(RE_Buff[2]==0)	//打开OUT1
				{
					HAL_GPIO_WritePin(GPIOB,OUT1_Pin,GPIO_PIN_SET);	//拉高
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000); //串口返回接收到的数
		}
		else if(RE_Buff[1]==2)	//控制OUT2输出
		{
				if(RE_Buff[2]==1)	 //关闭OUT2
				{
					HAL_GPIO_WritePin(GPIOB,OUT2_Pin,GPIO_PIN_RESET); //拉低
				}
				else if(RE_Buff[2]==0)	//打开OUT2
				{
					HAL_GPIO_WritePin(GPIOB,OUT2_Pin,GPIO_PIN_SET);	//拉高
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000);
		}
		else if(RE_Buff[1]==3)	//控制OUT3输出
		{
				if(RE_Buff[2]==1)	 //关闭OUT3
				{
					HAL_GPIO_WritePin(GPIOB,OUT3_Pin,GPIO_PIN_RESET); //拉低关闭继电器
				}
				else if(RE_Buff[2]==0)	//打开OUT3
				{
					HAL_GPIO_WritePin(GPIOB,OUT3_Pin,GPIO_PIN_SET);	//拉高打开继电器控制输出
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000);
		}
}

/*********************************************
	*函数名：Read_In3
	*输  入：无
	*输  出：无
	*说  明：读取3路电压输入转态，并串口返回，
					 读到低电平表示有电压输入
*********************************************/
void Read_In3(void)
{
		if(RE_Buff[1]==1)	//读取通道1状态
		{							  
				re_IN_buff[0]=2;
				re_IN_buff[1]=1;
				if(HAL_GPIO_ReadPin(GPIOB,IN1_Pin)==0)
				{
					re_IN_buff[2]=1;  //表示有电压输入
					re_IN_buff[3]=4;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN1_Pin)==1)
				{
					re_IN_buff[2]=0;	//表示无电压输入
					re_IN_buff[3]=3;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //回传读取出来的状态
		}
		else if(RE_Buff[1]==2)	//读取通道2状态
		{
				re_IN_buff[0]=2;
				re_IN_buff[1]=2;
				if(HAL_GPIO_ReadPin(GPIOB,IN2_Pin)==0)
				{
					re_IN_buff[2]=1;	//表示有电压输入
					re_IN_buff[3]=5;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN2_Pin)==1)
				{
					re_IN_buff[2]=0;	//表示无电压输入
					re_IN_buff[3]=4;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //回传读取出来的状态
		}
		else if(RE_Buff[1]==3) //读取通道3状态
		{
				re_IN_buff[0]=2;
				re_IN_buff[1]=3;
				if(HAL_GPIO_ReadPin(GPIOB,IN3_Pin)==0)
				{
					re_IN_buff[2]=1;	//表示有电压输入
					re_IN_buff[3]=6;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN3_Pin)==1)
				{
					re_IN_buff[2]=0;	//表示无电压输入
					re_IN_buff[3]=5;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //回传读取出来的状态
		}
}

/*********************************************
	*函数名：Cont_Dac
	*输  入：无
	*输  出：无
	*说  明：控制DAC输出，BUFF[1]代表电压整数，
					 BUFF[2]代表电压小数
*********************************************/
void Cont_Dac(void)
{
//	data[0]=(uint8_t)0x1f;
//	data[1]=0xff;
	//uint16_t data2[1]={0x10ff};
	
	DAC_cle();
	
	HAL_GPIO_WritePin(GPIOB,DAC_CS_Pin,GPIO_PIN_RESET); //片选使能MCP4822
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_SET);	   //拉高LDAC等待SPI传输完成
	HAL_Delay(1);
	
	HAL_SPI_Transmit(&hspi2,data,2,0xffff); //SPI传输数据到MCP4822
	//HAL_SPI_Transmit(&hspi2,(uint8_t *)data2,sizeof(data2),0xffff);
	
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB,DAC_CS_Pin,GPIO_PIN_SET); //断开使能MCP4822
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_RESET);	//使能LADC引脚，将DAC输入锁存器的数据送到输出锁存器
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_SET);	
}

/*********************************************
	*函数名：DAC_cle
	*输  入：无
	*输  出：无
	*说  明:DAC_out数据计算输出以及DAC通道选择
*********************************************/
void DAC_cle(void)
{
			int in_max=((RE_Buff[1]>>4)*10)+(int)(RE_Buff[1]&0xf);
			int in_min=(RE_Buff[2]>>4)*10+((int)RE_Buff[2]&0x0f);	
			DAC_data=(double)(in_max*100+in_min)/100.0 ;
			printf("DAC_data = %f\r\n",DAC_data);
			
			//error();	
		
			double DAC_out = (DAC_data/2.50);		//二级运算放大2.5倍，根据实际要控制输出的电压，计算出MCP48022要输出的电压	
		
		 // double DAC_out =2.5; //调试用代码
			
			//Dn=(double)(DAC_out*2000); //GN=1x
			Dn=(double)(DAC_out*1000); //GN=2x
	
			uint8_t Dn1=(Dn&0xf00)>>8;	//分离出高4位
			uint8_t Dn0=Dn&0x0ff;			//分离出低8位
			
			if(RE_Buff[0]==0x03)	//DAC通道1选择
			{
			
				//data[0]=(uint8_t)(0x30|Dn1);	//GN=1x;0011
				
				data[0]=(uint8_t)(0x10|Dn1);	//GN=2x;高8位寄存器配置加入控制数据 			
				data[1]=Dn0;			//低8位 DAC 输入数据
				//printf("A: data[0] = 0x%x ; data[1] = 0x%x\r\n",data[0],data[1]);	
			}
			else if(RE_Buff[0]==0x04)		//DAC通道2选择
			{
				data[0]=(uint8_t)(0x90|Dn1);
				data[1]=Dn0;								//低8位 DAC 输入数据
				//printf("B: data[0] = 0x%x ; data[1] = 0x%x\r\n",data[0],data[1]);				
			}
			
			/*printf("DAC_out = %f  \r\n",DAC_out);
			printf("Dn = %d \r\n",Dn);		
			Vout=[(2.048*Dn)*Gn]/2^n	Gn为2倍增益时，Gn=2；n=12时2^n=4096,此时 Vout=(2.048*Dn)*2/4096  Dn=Vout/0.001	
		  printf("Dn1 = 0x%x \r\nDn0 = 0x%x\r\n",Dn1,Dn0);	
			解析：data[0]=(uint8_t)0x10=0001 0000; 
			高8位【 A/B:0(A通道)	 -:0(无效位)	 GA:0(2倍增益)	 SHDN:1(当前通道使能)	0000:DAC数据】*/
	
}



/*********************************************
	*函数名：PWM
	*输  入：无
	*输  出：无
	*说  明：控制PWM输出，频率0.5~20KHz,占空比5%跳变
*********************************************/
void PWM(void)
{		
		printf("fre = %d\r\n",RE_Buff[1]-1);
		printf("cycle = %d\r\n",RE_Buff[2]-1);
	
		//per=20;减小17K、17.5K、18k的频率误差
		if(RE_Buff[1]==0x22||RE_Buff[1]==0x23||RE_Buff[1]==0x24)	
			MX_TIM14_Init(fre_20[RE_Buff[1]-1],cycle_20[RE_Buff[2]-1],20);
		
		//per=60,减小19.5K、19K、14k、13.5K、12.5K、11.5K、11k的频率误差
		else if(RE_Buff[1]==0x27||RE_Buff[1]==0x26||RE_Buff[1]==0x1b||RE_Buff[1]==0x19||RE_Buff[1]==0x17||RE_Buff[1]==0x16||RE_Buff[1]==0x1c)	
			MX_TIM14_Init(fre_60[RE_Buff[1]-1],cycle_60[RE_Buff[2]-1]+1,60);
		
		else //per=100
			MX_TIM14_Init(fre[RE_Buff[1]-1],cycle[RE_Buff[2]-1],100);
	
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); //PWM开启函数
}

/*********************************************
	*函数名：HAL_UART_RxCpltCallback
	*输  入：无
	*输  出：无
	*说  明：串口接收回调函数重写,进行数据校验，
					 处理DAC数据，接收完数据会执行该函数
*********************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/**/
	for(char i=0;i<4;i++) //检测接收数据对不对，调试用
	{	
		printf("RE_Buff[%d] = 0x%x\r\n",i,RE_Buff[i]);	
	}
	if(RE_Buff[0]+RE_Buff[1]+RE_Buff[2]==RE_Buff[3])	//接收数据校验
	{	
		uart_flag=1;
		printf("uart_flag = %d\r\n",uart_flag);	
	}
	else 
	{
		uart_flag=0;
		MX_USART1_UART_Init();
		memcpy(RE_Buff,zero,4);	//清除无效数据
		printf("uart_flag = %d\r\n",uart_flag);	
	}
	
}

	
/********************File END***************************/
