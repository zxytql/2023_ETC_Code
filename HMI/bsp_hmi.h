#ifndef BSP_HMI_H
#define BSP_HMI_H

#include "stdint.h"

#define HMI_START_CODE	(99)
#define HMI_TASK_CODE (0)
#define HMI_POS_X			(5)
#define HMI_POS_Y			(7)
#define HMI_POS_YAW		(9)
#define HMI_IDT_CODE	(11)
#define HMI_TASK_PCS	(13)
#define HMI_VISION_STA (14)
/**** Function ****/
void HMI_Write_txt(uint8_t n, float val);

#endif
