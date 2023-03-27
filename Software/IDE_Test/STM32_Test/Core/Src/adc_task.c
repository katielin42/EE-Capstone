/*
 * adc_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
///adasdas///
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"

//create raw and processed adc value buffers, ADC semaphore handle
static uint16_t ADC_buffer_raw[9];
float ADC_buffer_processed[3];
osSemaphoreId_t ADC_semHandle;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;

osStaticSemaphoreDef_t ADC_sem_ctrl_blk;
const osSemaphoreAttr_t ADC_sem_attributes = {
  .name = "ADC_sem",
  .cb_mem = &ADC_sem_ctrl_blk,
  .cb_size = sizeof(ADC_sem_ctrl_blk),
};

//private variables
float vref = 3.3, gnd = 0, adc_reso = 4095;
/* USER CODE BEGIN PV */



//initialize ADC thread
void ADC_Init(void *argument) {
	ADC_HandleTypeDef *hadc = argument;
	ADC_semHandle = osSemaphoreNew(1, 1, &ADC_sem_attributes);
	//start to collect ADC signals into ADC buffer through the DMA
	HAL_ADC_Start_DMA(hadc, (uint32_t*)ADC_buffer_raw, 9);
}

//since ADC 1 has 3 pins we are getting data from, our adc buffer raw has space for 9 values
//aka 3x of adc pins.
//We average out the 3 ADC values for each pin
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	// average the ADC values from 3 consecutive samples and convert them into the analog values with the formula (VREF-GND)/ADC_BIT_RESOLUTION
	//// analog values from adc bit sample = (VREF-GND)*ADC_bits/4095
	for(int i = 0 ; i < 3; i++) {
		ADC_buffer_processed[i] = ((vref-gnd)/adc_reso)*(((float)ADC_buffer_raw[0 + i] + ADC_buffer_raw[3 + i] + ADC_buffer_raw[6 + i])/3);
	}
	osSemaphoreRelease(ADC_semHandle);
}

