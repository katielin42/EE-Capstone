/*
 * sd_task.h
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */

#ifndef INC_SD_TASK_H_
#define INC_SD_TASK_H_

void SD_init(void);
void SD_process(char *filename, void *buffer, int length);

#endif /* INC_SD_TASK_H_ */
