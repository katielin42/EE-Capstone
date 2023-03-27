/*
 * adc_task.h
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */

#ifndef INC_ADC_TASK_H_
#define INC_ADC_TASK_H_

void ADC_Init(void *argument);
extern float ADC_buffer_processed[];
extern osSemaphoreId_t ADC_semHandle;
#endif /* INC_ADC_TASK_H_ */
