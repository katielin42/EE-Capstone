#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"
//#include "can_task.h"
#include "sd_task.h"




//initialize rtOS objects
/* Definitions for thread for controller object */
osThreadId_t thr_1;
const osThreadAttr_t thr_1_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


//define variables
char yourmom[] = "yourmom";
float ADC_P0 = 0;


void controller_state_machine(void *args);



void state_machine_init(void){
	thr_1 = osThreadNew(controller_state_machine, &hadc1, &thr_1_attributes);
}

void controller_state_machine(void *args){
	ADC_P0=ADC_buffer_processed[0];
	SD_process(yourmom, &ADC_P0, 1);
	  for(;;)
	  {
		  osSemaphoreAcquire(ADC_semHandle, 1);


	    osDelay(1);
	  }
	  // State transition if/else logic


}
