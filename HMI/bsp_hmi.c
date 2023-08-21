/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_hmi.c/h
  * @version    V1.0.0
  * @brief      淘晶池T1串口屏通讯文件
  * @note       基于RM-A开发板开发
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-08-20       zxy            First version
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
	
	
#include "bsp_hmi.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "tjc_usart_hmi.h"

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart6, &ch, 1, 0xffff);
  return ch;
}

void HMI_Write_txt(uint8_t n, float val)
{
//	static unsigned char tjcstr[30];
//	static uint8_t ucHMIEnd[3] = {0xFF,0xFF,0xFF};
	switch(n)
	{
		case HMI_TASK_CODE: //任务码
			printf("t0.txt=\"%d\"",(int)val);
			printf("\xff\xff\xff");
			break;
		
		case HMI_POS_X:			
			printf("t5.txt=\"%.2f\"",val);
			printf("\xff\xff\xff");
			break;
		
		case HMI_POS_Y:
			printf("t7.txt=\"%.2f\"",val);
			printf("\xff\xff\xff");
			break;
		
		case HMI_POS_YAW:
			printf("t9.txt=\"%.2f\"",val);
			printf("\xff\xff\xff");
			break;
		
		case HMI_IDT_CODE:	//颜色识别到的顺序
			
			break;
		
		case HMI_TASK_PCS:	//task_flow 流程码

			break;
		
		case HMI_START_CODE:
			//HAL_UART_Transmit(&huart6,ucHMIEnd,(uint8_t)3,200);
			break;
	}
}
