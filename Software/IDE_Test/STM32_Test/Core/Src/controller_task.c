#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"
#include <stdbool.h>
#include "bsp_driver_sd.h"
//#include "can_task.h"
#include "sd_task.h"
#include "can_task.h"




//initialize rtOS objects
/* Definitions for thread for controller object */
osThreadId_t thr_1;
const osThreadAttr_t thr_1_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


//define variables
const char errlog[] = "ErrorLog.txt";

void controller_state_machine(void *args);

static inline bool check_error(float APPS_VPA, float APPS_VPA2, float BSE){
	//| (APPS_VPA2 >4.5) | (BSE<0.5) | (APPS_VPA2 <0.5) |
	return ((APPS_VPA<=0.5) | (APPS_VPA>=4.5) | (BSE>4.5));
}

void state_machine_init(void){
	thr_1 = osThreadNew(controller_state_machine, &hadc1, &thr_1_attributes);
}

void update_values(float *APPS_VPA, float *APPS_VPA2, float * BSE){
	  *APPS_VPA=ADC_buffer_processed[0];
	  *APPS_VPA2=ADC_buffer_processed[1];
	  *BSE=ADC_buffer_processed[2];
}

void controller_state_machine(void *args){
	SD_init();
	float APPS_VPA = 0, APPS_VPA2 = 0, BSE = 0;
//	uint8_t sd_status = BSP_SD_IsDetected();
	char buffer[120];
	char buffer2[90];
	//dummy var for storing motor temp
	uint32_t startTimeStamp = osKernelGetSysTimerCount();
	//initialize write buffer for the SD card, size is arbitrary just be large enough to contain the chars
//	if (sd_status == SD_PRESENT) {
//		SD_init();
//	}
	  for(;;)
	  {
		  osSemaphoreAcquire(ADC_semHandle, 1);
//		  if ((sd_status != BSP_SD_IsDetected()) && (BSP_SD_IsDetected() == SD_PRESENT)){
//			  SD_init();
//		  }
//		  sd_status = BSP_SD_IsDetected();
		  update_values(&APPS_VPA, &APPS_VPA2, &BSE);
		  int n = snprintf(buffer, sizeof(buffer), "Error log: APPS Value is %1.2f, APPS2 Value is %1.2f, BSE Value is %1.2f, Setting motor torque to 0Nm; \n", APPS_VPA, APPS_VPA2, BSE);
		  int p = snprintf(buffer2, sizeof(buffer2), "Error log: Motor Temperature is %i, Setting motor torque to 0Nm; \n", decodedTemperature);
		  //maybe can replace with another while 1 looperoni
		  if (check_error(APPS_VPA, APPS_VPA2, BSE)){
			  if(osKernelGetSysTimerCount() - startTimeStamp >= 100) {
				  //send error frame
//				  if (sd_status == SD_PRESENT) {
				  	  canSend();
					  SD_process(errlog, buffer, n);
				  }
//			  }
		  }
		  else {
			  startTimeStamp = osKernelGetSysTimerCount();
		  }
		  if (decodedTemperature>120){
			  canSend();
			  SD_process(errlog, buffer2, p);
			  //else if received motor temperature is >the temp we want,
			  //send can frame to reduce speed
//			  if (sd_status == SD_PRESENT) {
//				  SD_process(yourmom, buffer, n);
//			  }
		  }
	  }
}
