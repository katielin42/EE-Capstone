/*
 * sd_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN 1 */
FRESULT res; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
uint16_t rtext[_MAX_SS];/* File read buffer */
/* USER CODE END 1 */

void SD_init(void){
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
		Error_Handler();
	}
	else{
		if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			Error_Handler();
		}
	}
}
//
//void* is common denominator for all pointers. Temp var to be changed to use with anything else
void SD_process(char *filename, void *buffer, int length){
			//Open file for writing (Create)
			if(f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
				Error_Handler();
			}
			else{
				//Write to the text file
				res = f_write(&SDFile, buffer, length, (void *)&byteswritten);
				if((byteswritten == 0) || (res != FR_OK)){
					Error_Handler();
				}
				else{
					f_close(&SDFile);
				}
			}
			//create if condition to check if buffer is null/bad function calls
//	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
}
