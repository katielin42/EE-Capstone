#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "adc_task.h"

///controller variables///

static uint32_t ticks = 1;
typedef StaticSemaphore_t osStaticMutexDef_t;
osStaticMutexDef_t myMutex01ControlBlock;

///controller OS objects///

///New mutex initialize
/* Definitions for myMutex01 */
osMutexId_t mutex_1;

const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01",
  .cb_mem = &myMutex01ControlBlock,
  .cb_size = sizeof(myMutex01ControlBlock),
};

// Start a new thread for ADC signal processing
osThreadId_t thr_1;
const osThreadAttr_t thr_1_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

///main controller loop///
void controller_state_machine(void *args);

///controller initialization///
void controller_init(void){
	/* creation of  mutex, need to add the threads accessing the buffer resource*/
	mutex_1 = osMutexNew(&myMutex01_attributes);
	///initialize a new thread for controller state machine
	thr_1 = osThreadNew(controller_state_machine, NULL, &thr_1_attributes);
};

///controller state helper functions///
void operational_state_tx_rx(void){};
void operational_state_rx(void){};
void APPS_sensor_1_fault(void){};
void APPS_sensor_2_fault(void){};
void BSE_fault(void){};
void limit_voltage_fault(void){};

void controller_state_machine(void *args){
	  for(;;)
	  {
		  osSemaphoreAcquire(ADC_semHandle, ticks);
	    osDelay(1);
	  }
};
