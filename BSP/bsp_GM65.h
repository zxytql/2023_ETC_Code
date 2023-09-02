#ifndef BSP_GM65_H
#define BSP_GM65_H

#include "stdint.h"
#include "string.h"
#include "stdlib.h"

//#define GM65_UART huart7

/**** Extern ****/
extern char GM_65_buff[3];

extern int GM_65_num_1;
extern int GM_65_num_2;

extern uint8_t GM65_UART_rx_buff[32];
extern uint8_t GM65_UART_rx_cnt;
extern uint8_t GM65_UART_rx_flag;

/**** Function ****/
void GM65_Scan(void);

#endif
