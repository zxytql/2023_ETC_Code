/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_GM65.c/h
  * @version    V1.0.0
  * @brief      扫码模块GM65支持包
  * @note       基于RM-A板开发，更换开发板可能需要更换对应引脚
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
			GM65_UART_rx_buff[GM65_UART_rx_cnt++] = temp_buff;  //Fix me 没扫到怎么办？
//			
//			if(temp_buff == 0x7E && UART1_rx_buff[8] == 0xCD) //说明第一次没扫到
//			{
//				memset(UART1_rx_buff,0x00,sizeof(UART1_rx_buff)); //清空缓存区
//				UART1_rx_cnt = 0;
//			}
		}
		else //接收到回车 说明扫到了数据
		{
			memcpy(GM_65_buff,GM65_UART_rx_buff+7,3); //拷贝数据
			GM_65_num_1 = atoi(GM_65_buff); //将字符串转换为整数
			memcpy(GM_65_buff,GM65_UART_rx_buff+11,3);
			GM_65_num_2 = atoi(GM_65_buff); //将字符串转换为整数
			memset(GM65_UART_rx_buff,0x00,sizeof(GM65_UART_rx_buff)); //清空缓存区
			GM65_UART_rx_flag = 1;
		}
		HAL_UART_Receive_IT(&huart7,(uint8_t *)&temp_buff,1); //重新开启中断
	}
}

void GM65_Scan(void)
{
	static int i = 0; //记录发送成功次数
	static int send_flag = 1; //发送标志
	static int tick_start = 0, tick_now = 0;
	
	tick_now = HAL_GetTick(); //获取当前tick
	
	if(GM65_UART_rx_flag == 1) //接收到二维码数据，跳出发送函数
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
		tick_start = HAL_GetTick(); //记录上次发送完毕后tick
	}

	if(tick_now - tick_start >= 5000) //超过5秒 还是没有读到二维码数据 重新发送
	{
		send_flag = 1;
	}
}
