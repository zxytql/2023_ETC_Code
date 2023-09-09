/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_hmi.c/h
  * @version    V1.0.0
  * @brief      �Ծ���T1������ͨѶ�ļ�
  * @note       ����RM-A�����忪��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-08-20       zxy            First version
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
	
	
#include "bsp_hmi.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

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
		case HMI_TASK_CODE: //������
			printf("t0.txt=\"%d+%d\"",(int)val/1000,(int)val%1000); //���ǰ��λ�ͺ���λ
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
		
		case HMI_IDT_CODE:	//��ɫʶ�𵽵�˳��
			
			break;
		
		case HMI_TASK_PCS:	//task_flow ������
			printf("t13.txt=\"%d\"",(int)val);
			printf("\xff\xff\xff");
			break;
		
		case HMI_START_CODE:
			//HAL_UART_Transmit(&huart6,ucHMIEnd,(uint8_t)3,200);
			break;
		
		case HMI_VISION_STA:
			if(val == 1)
			{
				printf("t14.txt=\"VISION LUMP!\""); //ʶ��ɫ��
				printf("\xff\xff\xff");				
			}
			else if(val == 2)
			{
				printf("t14.txt=\"VISION RING!\""); //ʶ��ɫ��
				printf("\xff\xff\xff");						
			}
			else
			{
				printf("t14.txt=\"WAIT VISION\""); //δ���������ݴ���
				printf("\xff\xff\xff");					
			}
			break;
			
		case HMI_RAS_POS_ERR_X:
			printf("t15.txt=\"%d\"",(int)val); //X_ERR
			printf("\xff\xff\xff");				
			break;

		case HMI_RAS_POS_ERR_Y:
			printf("t16.txt=\"%d\"",(int)val); //Y_ERR
			printf("\xff\xff\xff");				
			break;
		
		default:
			break;
	}
}
