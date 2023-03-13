/*
 * adc_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"

static uint16_t ADC_buffer_raw[9];
static uint16_t ADC_buffer_processed[3];
osSemaphoreId_t ADC_semHandle;
osStaticSemaphoreDef_t ADC_sem_ctrl_blk;
const osSemaphoreAttr_t ADC_sem_attributes = {
  .name = "ADC_sem",
  .cb_mem = &ADC_sem_ctrl_blk,
  .cb_size = sizeof(ADC_sem_ctrl_blk),
};
/* USER CODE BEGIN PV */

/* Definitions for myMutex01 */
/* Definitions for threads */
osThreadId_t thr_1;
const osThreadAttr_t thr_1_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void ADC_Init(void) {
	ADC_semHandle = osSemaphoreNew(1, 1, &ADC_sem_attributes);
	thr_1 = osThreadNew(ADC_collect, &hadc1, &thr_1_attributes);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	// ADC averaging
	for(int i = 0 ; i < 3; i++) {
		ADC_buffer_processed[i] = (ADC_buffer_raw[0 + i] + ADC_buffer_raw[3 + i] + ADC_buffer_raw[6 + i])/3;
	}
	osSemaphoreRelease(ADC_semHandle);
}

void ADC_collect(void *argument)
{
	ADC_HandleTypeDef *hadc = argument;
  /* Infinite loop */
	HAL_ADC_Start_DMA(hadc, (uint32_t*)ADC_buffer_raw, 9);
  for(;;)
  {
	  osSemaphoreAcquire(ADC_semHandle, osWaitForever);
	  // process adc buffer good
    osDelay(1);
  }
}

