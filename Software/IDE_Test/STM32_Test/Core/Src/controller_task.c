#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"
//#include "can_task.h"
#include "sd_task.h"
#include




//initialize rtOS objects
/* Definitions for thread for controller object */
osThreadId_t thr_1;
const osThreadAttr_t thr_1_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


//define variables
char yourmom[] = "URMOM.txt";
float APPS_VPA = 0, APPS_VPA2 = 0, BSE = 0;
//initialize write buffer for the SD card, size is arbitrary just be large enough to contain the chars
char buffer[20];

void controller_state_machine(void *args);



void state_machine_init(void){
	thr_1 = osThreadNew(controller_state_machine, &hadc1, &thr_1_attributes);
}



void controller_state_machine(void *args){
	SD_init();



//	SD_process(yourmom, &ADC_P0, sizeof(ADC_P0));
	  for(;;)
	  {
		  osSemaphoreAcquire(ADC_semHandle, 1);
		  APPS_VPA=ADC_buffer_processed[0];
		  APPS_VPA2=ADC_buffer_processed[1];
		  BSE=ADC_buffer_processed[2];
		  int n = snprintf(buffer, sizeof(buffer), "PA0 Value is: %1.2f\n", APPS_VPA);
		  osDelay(1000);
		  SD_process(yourmom, buffer, n);
	      osDelay(1000);
	  }
	  // State transition if/else logic


}
