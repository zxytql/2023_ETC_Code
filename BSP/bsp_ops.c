/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_ops.c/h
  * @version    V1.0.1
  * @brief      平面定位系统OPS-9支持包
  * @history
  *  Version        Date            Author          Modification
  *  V1.0.0         2022-05-10      zxy             First Version
  * @verbatim
  * [V1.0.0]
  * 移植自@YLT的程序
  * =============================================================================
  *                                  OPS-9 数据协议
  * =============================================================================
  * 通信协议    UART
  * 波特率     115200
  * 起始位     1
  * 数据位     8
  * 停止位     1
  * 校验位     0
  *    帧头           数据          帧尾
  * [HEAD 2BYTE] [DATA 24BYTE] [END 2BYTE]
  *  24   =  4    4    4     4     4      4
  * DATA -> YAW PITCH ROLL POS_X POS_Y YAW_SPEED
  * 低字节优先
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
#include "bsp_ops.h"
#include "string.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "usart.h"

/******** Global Variables ********/
ops_data_t ops_data;

/**
 * @brief   Ops平面定位系统数据解析及处理
 * @param   Ops: Ops数据联合体
 * @param   buf: 串口数据
 */
void Ops_Frame_Parse(ops_data_t *ops, uint8_t *buf)
{
    if (buf[0] == OPS_HEAD_H && buf[1] == OPS_HEAD_L && buf[26] == OPS_END_H && buf[27] == OPS_END_L)
    {
        /** 联合体 共享内存 访问value[]即可获得float数据 **/
        memcpy(&(ops->data[0]), &(buf[2]), 24);
				
    }
}

void Ops_Stract(uint8_t strDestination[], uint8_t strSource[], int num)
{
	int i = 0, j = 0;
	
	while(strDestination[i] != '\0') { i++; }
	
	for(j = 0; j < num; j++)
	{
		strDestination[i++] = strSource[j];
	}
}

/**
 * @brief   Ops初始化等待
 * @note    Ops上电后需要15秒才会发送数据
 */
void Ops_Waiting(void)
{
    do
    {
        osDelay(10);
    } while (ops_data.val[Yaw_speed] == 0.0f);

    //ActionDoneBuzzer();
//    Ops_Calibration();

}

/**
 * @brief   Ops校准
 * @note    出厂时已经校准 无特殊需求不必较准
 */
void Ops_Calibration(void)
{
    uint8_t buff[4];
    buff[0] = 'A';
    buff[1] = 'C';
    buff[2] = 'T';
    buff[3] = 'R';
    for (uint8_t i = 0; i < 4; i++)
    {
        while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC));
        HAL_UART_Transmit(&huart6,&buff[i],4,0xffff);
    }

}

/**
 * @brief   设定Ops航向角 x y坐标
 * @param   yaw: 航向角
 * @param   pos_x: 平面坐标 X
 * @param   pos_y: 平面坐标 Y
 */
void Ops_Set_All(float yaw, float pos_x, float pos_y)
{
    union
    {
        uint8_t send_buff[16];
        float value[4];
    }buff;
    buff.send_buff[0]  = 'A';
    buff.send_buff[1]  = 'C';
    buff.send_buff[2]  = 'T';
    buff.send_buff[3]  = 'A';
    buff.value[1] = yaw;
    buff.value[2] = pos_x;
    buff.value[3] = pos_y;
    for (uint8_t i = 0; i < 16; i++)
    {
        while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC));
        HAL_UART_Transmit(&huart6,&buff.send_buff[i],16,0xffff);
    }
}
int buff1=0;

/**
 * @brief   设定Ops平面坐标
 * @param   pos_x: 平面坐标 X
 * @param   pos_y: 平面坐标 Y
 */
void Ops_Set_Pos(float pos_x,float pos_y)
{
    union
    {
        uint8_t send_buff[12];
        float value[3];
    }buff;
    buff.send_buff[0]  = 'A';
    buff.send_buff[1]  = 'C';
    buff.send_buff[2]  = 'T';
    buff.send_buff[3]  = 'D';
    buff.value[1] = pos_x;
    buff.value[2] = pos_y;
    for (uint8_t i = 0; i < 12; i++)
    {
        while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC));
        buff1++;
        HAL_UART_Transmit_IT(&huart6,&buff.send_buff[i],12);
    }
}

/**
 * @brief   设定Ops平面坐标 X
 * @param   pos_x: 平面坐标 X
 */
void Ops_Set_X(float pos_x)
{
//    union
//    {
//        uint8_t send_buff[8];
//        float value[2];
//    }buff;
//    buff.send_buff[0]  = 'A';
//    buff.send_buff[1]  = 'C';
//    buff.send_buff[2]  = 'T';
//    buff.send_buff[3]  = 'X';
//    buff.value[1] = pos_x;
//    for (uint8_t i = 0; i < 8; i++)
//    {
//        while(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_TXE == RESET));
//        HAL_UART_Transmit(&huart8,&buff.send_buff[i],8,0xffff);
//    }
	int i = 0;
	uint8_t Update_x[8] = "ACTX";
	
	static union
	{
		float X;
		uint8_t data[4];
	}New_set;
	
	New_set.X = pos_x;
	Ops_Stract(Update_x,New_set.data,4);
	
	for(i = 0; i < 8; i++)
	{
		while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TXE) == RESET);
		HAL_UART_Transmit(&huart6,&Update_x[i],1,0xffff);
	}
}

/**
 * @brief   设定Ops平面坐标 Y
 * @param   pos_y: 平面坐标 Y
 */
void Ops_Set_Y(float pos_y)
{
    union
    {
        uint8_t send_buff[8];
        float value[2];
    }buff;
    buff.send_buff[0]  = 'A';
    buff.send_buff[1]  = 'C';
    buff.send_buff[2]  = 'T';
    buff.send_buff[3]  = 'Y';
    buff.value[1] = pos_y;
    for (uint8_t i = 0; i < 8; i++)
    {
        //while(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_TXE));
        HAL_UART_Transmit(&huart6,&buff.send_buff[i],8,0xffff);
    }
}

/**
 * @brief   设定Ops航向角
 * @param   yaw: 航向角
 */
void Ops_Set_Yaw(float yaw)
{
    union
    {
        uint8_t send_buff[8];
        float value[2];
    }buff;
    buff.send_buff[0]  = 'A';
    buff.send_buff[1]  = 'C';
    buff.send_buff[2]  = 'T';
    buff.send_buff[3]  = 'J';
    buff.value[1] = yaw;
    for (uint8_t i = 0; i < 8; i++)
    {
        while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC));
        HAL_UART_Transmit(&huart6,&buff.send_buff[i],8,0xffff);
    }
		osDelay(10);
}

/**
 * @brief   设定Ops零位
 * @param   yaw: 航向角
 */
void Ops_Set_Zero(void)
{
		uint8_t str[] = "ACT0";
		//uint8_t str[] = "0TCA";
	//HAL_UART_Transmit(&huart8,"ACT0",strlen("ACT0"),0xffff);
	HAL_UART_Transmit(&huart6,str,sizeof(str),0xffff);
}

float Ops_Get_X(void)
{
	return ops_data.val[3];
}

float Ops_Get_Y(void)
{
	return ops_data.val[4];
}

float Ops_Get_Yaw(void)
{
	return ops_data.val[0];
}

float Ops_Get_Yaw_Speed(void)
{
	return ops_data.val[5];
}
