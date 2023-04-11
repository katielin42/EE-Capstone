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


//if unmount SD card, need to rerun this block SD_init again to re-mount sd card, otherwise data collection process will be halted.
void SD_init(void){
	//Mounts the SD card. If cannot mount SD card, throw error
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
		Error_Handler();
	}
	else{
		//Make a volume and initializes a file system on the SD card to get it ready for writing. If cannot mkfs, throw error
		if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			Error_Handler();
		}
	}
}
//
//void* is common denominator for all pointers. Temp var to be changed to use with anything else
void SD_process(const char *filename, void *buffer, int length){
			//Open file for writing (Create)
			if(f_open(&SDFile, filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK){
				Error_Handler();
			}
			else{
				//Write to the text file
				res = f_write(&SDFile, buffer, length, (void *)&byteswritten);
				if((byteswritten == 0) || (res != FR_OK)){
					Error_Handler();
				}
				//close sd file
				else{
					f_close(&SDFile);
				}
			}
			//create if condition to check if buffer is null/bad function calls
}
