/********************************************************
		File:
					*�����·���Ƶ�ѹ(Cont_Out3)
					*��ȡ3·24V����(Read_In3)
					*DAC�������(Cont_Dac)
					*DAC_cle��ѹ�����������(DAC_cle)
					*PWM�������(PWM)
					*���ڻص�������д(HAL_UART_RxCpltCallback)
					
********************************************************/
#include "user.h"

uint8_t 	re_IN_buff[4];  //���صĶ�ȡ�ⲿ��ѹ����IN״̬buff
uint8_t 	re_Out_buff[4]; //���صĿ����ⲿ��ѹ���out״̬buff
uint8_t 	data[2];				//SPI�������DAC������
uint8_t 	zero[4]={0}; 	  //��ջ�������
uint16_t  Dn=0;						//����ʵ�ʿ��Ƶ�ѹ���������Ҫ�����MCP4822��Dx����ֵ

double    DAC_data=0;		  //���ڴ�������Ҫ���Ƶ�DAC��ֵ��С
char 			uart_flag=0;		//���������ݺ�У�����������־λ

uint8_t   cycle[20]={	//����ֵΪ100ʱռ�ձ�����
										  5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};

uint16_t   fre[40]={	//����ֵΪ100ʱ��Ƶ�ʿ������飬�и���Ƶ�����ϴ�����������
										 960,480,320,240,192,160,137,119,106,95,86,79,73,67,63,59,55,
										 52,49,47,45,42,40,39,37,36,35,33,32,31,30,29,28,27,26,26,25,24,24,23
									 };

uint8_t cycle_60[20]={	//����ֵΪ60ʱռ�ձ�����
											3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,48,51,54,57,60};

uint16_t fre_60[40]={	//����ֵΪ60ʱ��Ƶ�ʿ�������,������С19.5K��19K��14k��13.5K��12.5K��11.5K��11k�����
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,75,72,
											69,24,63,26,58,56,29,30,31,32,33,34,35,36,37,41,40,40};

uint8_t cycle_20[20]={	//����ֵΪ20ʱռ�ձ�����
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

uint16_t fre_20[40]={	//����ֵΪ20ʱ��Ƶ�ʿ�������,������С17K��17.5K��18k�����
											1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,75,72,
											69,24,63,26,58,28,29,30,31,32,33,140,136,132,37,41,40,40};
										
/*********************************************
	*��������Cont_Out3
	*��  �룺��
	*��  ������
	*˵  ��������3·OUT��ѹ����������ؿ���״̬��
					 ���ߴ򿪼̵����������ѹ
*********************************************/
void Cont_Out3(void)
{
		if(RE_Buff[1]==1)	//����OUT1���
		{
				if(RE_Buff[2]==1)	 //�ر�OUT1
				{
					HAL_GPIO_WritePin(GPIOB,OUT1_Pin,GPIO_PIN_RESET); //����
				}
				else if(RE_Buff[2]==0)	//��OUT1
				{
					HAL_GPIO_WritePin(GPIOB,OUT1_Pin,GPIO_PIN_SET);	//����
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000); //���ڷ��ؽ��յ�����
		}
		else if(RE_Buff[1]==2)	//����OUT2���
		{
				if(RE_Buff[2]==1)	 //�ر�OUT2
				{
					HAL_GPIO_WritePin(GPIOB,OUT2_Pin,GPIO_PIN_RESET); //����
				}
				else if(RE_Buff[2]==0)	//��OUT2
				{
					HAL_GPIO_WritePin(GPIOB,OUT2_Pin,GPIO_PIN_SET);	//����
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000);
		}
		else if(RE_Buff[1]==3)	//����OUT3���
		{
				if(RE_Buff[2]==1)	 //�ر�OUT3
				{
					HAL_GPIO_WritePin(GPIOB,OUT3_Pin,GPIO_PIN_RESET); //���͹رռ̵���
				}
				else if(RE_Buff[2]==0)	//��OUT3
				{
					HAL_GPIO_WritePin(GPIOB,OUT3_Pin,GPIO_PIN_SET);	//���ߴ򿪼̵����������
				}
				HAL_UART_Transmit(&huart1,RE_Buff,sizeof(RE_Buff),1000);
		}
}

/*********************************************
	*��������Read_In3
	*��  �룺��
	*��  ������
	*˵  ������ȡ3·��ѹ����ת̬�������ڷ��أ�
					 �����͵�ƽ��ʾ�е�ѹ����
*********************************************/
void Read_In3(void)
{
		if(RE_Buff[1]==1)	//��ȡͨ��1״̬
		{							  
				re_IN_buff[0]=2;
				re_IN_buff[1]=1;
				if(HAL_GPIO_ReadPin(GPIOB,IN1_Pin)==0)
				{
					re_IN_buff[2]=1;  //��ʾ�е�ѹ����
					re_IN_buff[3]=4;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN1_Pin)==1)
				{
					re_IN_buff[2]=0;	//��ʾ�޵�ѹ����
					re_IN_buff[3]=3;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //�ش���ȡ������״̬
		}
		else if(RE_Buff[1]==2)	//��ȡͨ��2״̬
		{
				re_IN_buff[0]=2;
				re_IN_buff[1]=2;
				if(HAL_GPIO_ReadPin(GPIOB,IN2_Pin)==0)
				{
					re_IN_buff[2]=1;	//��ʾ�е�ѹ����
					re_IN_buff[3]=5;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN2_Pin)==1)
				{
					re_IN_buff[2]=0;	//��ʾ�޵�ѹ����
					re_IN_buff[3]=4;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //�ش���ȡ������״̬
		}
		else if(RE_Buff[1]==3) //��ȡͨ��3״̬
		{
				re_IN_buff[0]=2;
				re_IN_buff[1]=3;
				if(HAL_GPIO_ReadPin(GPIOB,IN3_Pin)==0)
				{
					re_IN_buff[2]=1;	//��ʾ�е�ѹ����
					re_IN_buff[3]=6;
				}
				else if(HAL_GPIO_ReadPin(GPIOB,IN3_Pin)==1)
				{
					re_IN_buff[2]=0;	//��ʾ�޵�ѹ����
					re_IN_buff[3]=5;
				}
				HAL_UART_Transmit(&huart1,re_IN_buff,sizeof(re_IN_buff),1000); //�ش���ȡ������״̬
		}
}

/*********************************************
	*��������Cont_Dac
	*��  �룺��
	*��  ������
	*˵  ��������DAC�����BUFF[1]�����ѹ������
					 BUFF[2]�����ѹС��
*********************************************/
void Cont_Dac(void)
{
//	data[0]=(uint8_t)0x1f;
//	data[1]=0xff;
	//uint16_t data2[1]={0x10ff};
	
	DAC_cle();
	
	HAL_GPIO_WritePin(GPIOB,DAC_CS_Pin,GPIO_PIN_RESET); //Ƭѡʹ��MCP4822
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_SET);	   //����LDAC�ȴ�SPI�������
	HAL_Delay(1);
	
	HAL_SPI_Transmit(&hspi2,data,2,0xffff); //SPI�������ݵ�MCP4822
	//HAL_SPI_Transmit(&hspi2,(uint8_t *)data2,sizeof(data2),0xffff);
	
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB,DAC_CS_Pin,GPIO_PIN_SET); //�Ͽ�ʹ��MCP4822
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_RESET);	//ʹ��LADC���ţ���DAC�����������������͵����������
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA,LDAC_Pin,GPIO_PIN_SET);	
}

/*********************************************
	*��������DAC_cle
	*��  �룺��
	*��  ������
	*˵  ��:DAC_out���ݼ�������Լ�DACͨ��ѡ��
*********************************************/
void DAC_cle(void)
{
			int in_max=((RE_Buff[1]>>4)*10)+(int)(RE_Buff[1]&0xf);
			int in_min=(RE_Buff[2]>>4)*10+((int)RE_Buff[2]&0x0f);	
			DAC_data=(double)(in_max*100+in_min)/100.0 ;
			printf("DAC_data = %f\r\n",DAC_data);
			
			//error();	
		
			double DAC_out = (DAC_data/2.50);		//��������Ŵ�2.5��������ʵ��Ҫ��������ĵ�ѹ�������MCP48022Ҫ����ĵ�ѹ	
		
		 // double DAC_out =2.5; //�����ô���
			
			//Dn=(double)(DAC_out*2000); //GN=1x
			Dn=(double)(DAC_out*1000); //GN=2x
	
			uint8_t Dn1=(Dn&0xf00)>>8;	//�������4λ
			uint8_t Dn0=Dn&0x0ff;			//�������8λ
			
			if(RE_Buff[0]==0x03)	//DACͨ��1ѡ��
			{
			
				//data[0]=(uint8_t)(0x30|Dn1);	//GN=1x;0011
				
				data[0]=(uint8_t)(0x10|Dn1);	//GN=2x;��8λ�Ĵ������ü���������� 			
				data[1]=Dn0;			//��8λ DAC ��������
				//printf("A: data[0] = 0x%x ; data[1] = 0x%x\r\n",data[0],data[1]);	
			}
			else if(RE_Buff[0]==0x04)		//DACͨ��2ѡ��
			{
				data[0]=(uint8_t)(0x90|Dn1);
				data[1]=Dn0;								//��8λ DAC ��������
				//printf("B: data[0] = 0x%x ; data[1] = 0x%x\r\n",data[0],data[1]);				
			}
			
			/*printf("DAC_out = %f  \r\n",DAC_out);
			printf("Dn = %d \r\n",Dn);		
			Vout=[(2.048*Dn)*Gn]/2^n	GnΪ2������ʱ��Gn=2��n=12ʱ2^n=4096,��ʱ Vout=(2.048*Dn)*2/4096  Dn=Vout/0.001	
		  printf("Dn1 = 0x%x \r\nDn0 = 0x%x\r\n",Dn1,Dn0);	
			������data[0]=(uint8_t)0x10=0001 0000; 
			��8λ�� A/B:0(Aͨ��)	 -:0(��Чλ)	 GA:0(2������)	 SHDN:1(��ǰͨ��ʹ��)	0000:DAC���ݡ�*/
	
}



/*********************************************
	*��������PWM
	*��  �룺��
	*��  ������
	*˵  ��������PWM�����Ƶ��0.5~20KHz,ռ�ձ�5%����
*********************************************/
void PWM(void)
{		
		printf("fre = %d\r\n",RE_Buff[1]-1);
		printf("cycle = %d\r\n",RE_Buff[2]-1);
	
		//per=20;��С17K��17.5K��18k��Ƶ�����
		if(RE_Buff[1]==0x22||RE_Buff[1]==0x23||RE_Buff[1]==0x24)	
			MX_TIM14_Init(fre_20[RE_Buff[1]-1],cycle_20[RE_Buff[2]-1],20);
		
		//per=60,��С19.5K��19K��14k��13.5K��12.5K��11.5K��11k��Ƶ�����
		else if(RE_Buff[1]==0x27||RE_Buff[1]==0x26||RE_Buff[1]==0x1b||RE_Buff[1]==0x19||RE_Buff[1]==0x17||RE_Buff[1]==0x16||RE_Buff[1]==0x1c)	
			MX_TIM14_Init(fre_60[RE_Buff[1]-1],cycle_60[RE_Buff[2]-1]+1,60);
		
		else //per=100
			MX_TIM14_Init(fre[RE_Buff[1]-1],cycle[RE_Buff[2]-1],100);
	
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); //PWM��������
}

/*********************************************
	*��������HAL_UART_RxCpltCallback
	*��  �룺��
	*��  ������
	*˵  �������ڽ��ջص�������д,��������У�飬
					 ����DAC���ݣ����������ݻ�ִ�иú���
*********************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/**/
	for(char i=0;i<4;i++) //���������ݶԲ��ԣ�������
	{	
		printf("RE_Buff[%d] = 0x%x\r\n",i,RE_Buff[i]);	
	}
	if(RE_Buff[0]+RE_Buff[1]+RE_Buff[2]==RE_Buff[3])	//��������У��
	{	
		uart_flag=1;
		printf("uart_flag = %d\r\n",uart_flag);	
	}
	else 
	{
		uart_flag=0;
		MX_USART1_UART_Init();
		memcpy(RE_Buff,zero,4);	//�����Ч����
		printf("uart_flag = %d\r\n",uart_flag);	
	}
	
}

	
/********************File END***************************/
