/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_GM65.c/h
  * @version    V1.0.0
  * @brief      ɨ��ģ��GM65֧�ְ�
  * @note       ����RM-A�忪�������������������Ҫ������Ӧ����
  * @history
  *  Version     Date            Author          Modification
  *  V1.0.0     2023-08-22        zxy             First version
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
	
	
#include "bsp_GM65.h"
#include "usart.h"
#include "task_flow.h"

uint8_t temp_buff;
char GM_65_buff[3] = {0};
int GM_65_num_1;
int GM_65_num_2;
uint8_t GM65_UART_rx_buff[32] = {0};
uint8_t GM65_UART_rx_cnt = 0;
uint8_t GM65_UART_rx_flag = 0;
uint8_t _0D_cnt = 0;

uint8_t GM_65_On_Cmd[9] = {0x7E,0x00,0x08,0x01,0x00,0x02,0x01,0xAB,0xCD};
uint8_t GM_65_On_fed[7] = {0x02,0x00,0x00,0x01,0x00,0x33,0x31};


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART7) //GM65_UART
	{
		if(temp_buff != 0x0D) 
		{
			GM65_UART_rx_buff[GM65_UART_rx_cnt++] = temp_buff;  //Fix me ûɨ����ô�죿
//			
//			if(temp_buff == 0x7E && UART1_rx_buff[8] == 0xCD) //˵����һ��ûɨ��
//			{
//				memset(UART1_rx_buff,0x00,sizeof(UART1_rx_buff)); //��ջ�����
//				UART1_rx_cnt = 0;
//			}
		}
		else //���յ��س� ˵��ɨ��������
		{
			memcpy(GM_65_buff,GM65_UART_rx_buff+7,3); //��������
			GM_65_num_1 = atoi(GM_65_buff); //���ַ���ת��Ϊ����
			memcpy(GM_65_buff,GM65_UART_rx_buff+11,3);
			GM_65_num_2 = atoi(GM_65_buff); //���ַ���ת��Ϊ����
			memset(GM65_UART_rx_buff,0x00,sizeof(GM65_UART_rx_buff)); //��ջ�����
			GM65_UART_rx_flag = 1;
		}
		HAL_UART_Receive_IT(&huart7,(uint8_t *)&temp_buff,1); //���¿����ж�
	}
}

void GM65_Scan(void)
{
	static int i = 0; //��¼���ͳɹ�����
	static int send_flag = 1; //���ͱ�־
	static int tick_start = 0, tick_now = 0;
	
	tick_now = HAL_GetTick(); //��ȡ��ǰtick
	
	if(GM65_UART_rx_flag == 1) //���յ���ά�����ݣ��������ͺ���
	{
		task_flow.task_process++;
		return;
	}
	
	if(send_flag == 1)
	{
		if(HAL_UART_Transmit(&huart7,GM_65_On_Cmd,sizeof(GM_65_On_Cmd),0xff) == HAL_OK)
		{
			i++;
			send_flag = 0;
		}
		tick_start = HAL_GetTick(); //��¼�ϴη�����Ϻ�tick
	}

	if(tick_now - tick_start >= 5000) //����5�� ����û�ж�����ά������ ���·���
	{
		send_flag = 1;
	}
}
