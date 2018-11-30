/*********************************************
	*函数名：error
	*输  入：无
	*输  出：无
	*说  明：电压软件校准
*********************************************/
#include "erro_adc.h"

void error(void)
{		
	int in_max=((RE_Buff[1]>>4)*10)+(int)(RE_Buff[1]&0xf);
	int in_min=(RE_Buff[2]>>4)*10+((int)RE_Buff[2]&0x0f);	
	if(RE_Buff[0]==0x03)	//DAC——A校验
	{

		if(RE_Buff[1]==0x01||RE_Buff[1]==0x02)			
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.0171;  //y.xx校验值

		else if(RE_Buff[1]==0x03)
		{DAC_data=(double)(in_max*100+in_min)/100.0 - 0.0145;}
		
		else if(RE_Buff[1]==0x05)
		 DAC_data=(double)(in_max*100+in_min)/100.0 - 0.0101;
		
		else if(RE_Buff[1]==0x04||RE_Buff[1]==0x06)
		 DAC_data=(double)(in_max*100+in_min)/100.0 - 0.006;
		
		else if(RE_Buff[1]==0x07)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.013;  //1.xx校验值
		
		else if(RE_Buff[1]==0x09||RE_Buff[1]==0x08)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.022;
		
		else 	
			DAC_data=(double)(in_max*100+in_min)/100.0 ;
		
		printf("A:DAC_data = %f\r\n",DAC_data);
	}
	
	else if(RE_Buff[0]==0x04)		//DAC——B校验
	{

	
		if(RE_Buff[1]==0x01||RE_Buff[1]==0x02)			
			DAC_data=(double)(in_max*100+in_min)/100.0;
		
	  else if(RE_Buff[1]==0x03||RE_Buff[1]==0x04)			
		  DAC_data=(double)(in_max*100+in_min)/100.0-0.004;	
		
		else if(RE_Buff[1]==0x05)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.012;
		
		else if(RE_Buff[1]==0x06)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.02;
		
		else if(RE_Buff[1]==0x07)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.028;
		
		else if(RE_Buff[1]==0x08)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.037;
		
		else if(RE_Buff[1]==0x09)
			DAC_data=(double)(in_max*100+in_min)/100.0 - 0.043;
		
		else 
			DAC_data=(double)(in_max*100+in_min)/100.0 + 0.007;
		
			printf("B:DAC_data = %f\r\n",DAC_data);
	}
	
}





