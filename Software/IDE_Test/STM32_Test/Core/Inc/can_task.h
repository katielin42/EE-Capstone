/*
 * can_task.h
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#ifndef INC_CAN_TASK_H_
#define INC_CAN_TASK_H_


//int temperatureDecode (int high, int low);
void can_Init(void);
void canSend(void);
extern int decodedTemperature;
//int canRecieve(void);

#endif /* INC_CAN_TASK_H_ */


