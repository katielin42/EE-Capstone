/*
 * can_task.h
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */

#ifndef INC_CAN_TASK_H_
#define INC_CAN_TASK_H_

bool CAN_Read_Message(uint32_t *ID, uint8_t data[]);
ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[]);

#endif /* INC_CAN_TASK_H_ */


