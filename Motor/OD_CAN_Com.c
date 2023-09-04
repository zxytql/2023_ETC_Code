/**
  **************************(C) COPYRIGHT 2020-2023 HCRT****************************
  * @file       OD_CAN_Com.c/h
  * @version    V1.0.0
  * @brief      OD-0.5.4 CAN通讯包
  * @note       只封装了常用的命令 剩下的请自行参考Odrive官方文档
  * @history
  *  Version      Date            Author             Modification
  *  V1.0.0     2023-04-09          zxy              First Version
  * @verbatim
  * @endverbatim
  **************************(C) COPYRIGHT 2020-2022 HCRT****************************
  */
	
#include "OD_CAN_Com.h"
#include "string.h"
#include "cmsis_os.h"

/**** Global Variables ****/
uint8_t od_trans_byte[4] = {0};
OD_union_t OD_union;



/**************************/

/**
 * @brief  浮点数转4字节
 * @param  f: 浮点数
 * @retval uint8*: 四字节数组
 */
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;           //注意，会丢失精度
	od_trans_byte[0] = (longdata & 0xFF000000) >> 24;
	od_trans_byte[1] = (longdata & 0x00FF0000) >> 16;
	od_trans_byte[2] = (longdata & 0x0000FF00) >> 8;
	od_trans_byte[3] = (longdata & 0x000000FF);
	
	//大端小端转换
	static unsigned char temp;
	temp = od_trans_byte[3];
	od_trans_byte[3] = od_trans_byte[0];
	od_trans_byte[0] = temp;
	//
	temp = od_trans_byte[1];
	od_trans_byte[1] = od_trans_byte[2];
	od_trans_byte[2] = temp;
//	OD_union.val = f;
//	memcpy(od_trans_byte,OD_union.byte,4);
	return od_trans_byte;	
}

void OD_Send_CAN(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t data[8])
{
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxMessage[8];
	
  TxHeader.StdId = ID;
  TxHeader.ExtId = ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxMessage[0] = data[0];
  TxMessage[1] = data[1];
  TxMessage[2] = data[2];
  TxMessage[3] = data[3];
  TxMessage[4] = data[4];
  TxMessage[5] = data[5];
  TxMessage[6] = data[6];
  TxMessage[7] = data[7];

  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);	
}

void OD_Axis_Set_CloseLoop(uint16_t od_axis)
{
	static uint16_t cmd_id = 0x07;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0x08,0,0,0,0,0,0,0};
	OD_Send_CAN(&hcan1,can_id,data);
	osDelay(OD_COM_DELAY_TIME);
}

void OD_Axis_Set_Idle(uint16_t od_axis)
{
	static uint16_t cmd_id = 0x07;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0x01,0,0,0,0,0,0,0};
	OD_Send_CAN(&hcan1,can_id,data);	
	osDelay(OD_COM_DELAY_TIME);
}

void OD_Set_Ctrl_Mode(uint16_t od_axis,uint16_t ctrl_mode, uint16_t input_mode)
{
	static uint16_t cmd_id = 0x0b;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {ctrl_mode,0,0,0,input_mode,0,0,0};
	OD_Send_CAN(&hcan1,can_id,data);	
	osDelay(OD_COM_DELAY_TIME);
}

void OD_Set_Input_Vel(uint16_t od_axis, float vel)
{
	static uint16_t cmd_id = 0x0d;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};
	memcpy(data, Float_to_Byte(vel), sizeof(float));
	OD_Send_CAN(&hcan1,can_id,data);	
}

void OD_Set_Input_Pos(uint16_t od_axis, float pos)
{
	static uint16_t cmd_id = 0x0c;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};
	memcpy(data, Float_to_Byte(pos), sizeof(float));
	OD_Send_CAN(&hcan1,can_id,data);		
}

void OD_Reboot(uint16_t od_axis)
{
	static uint16_t cmd_id = 0x16;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0};
	OD_Send_CAN(&hcan1,can_id,data);	
	osDelay(OD_COM_DELAY_TIME);
}

void OD_Clear_Errors(uint16_t od_axis)
{
	static uint16_t cmd_id = 0x18;
	uint16_t can_id = od_axis << 5 | cmd_id;
	uint8_t data[8] = {0};
	OD_Send_CAN(&hcan1,can_id,data);		
	osDelay(OD_COM_DELAY_TIME);
}
