# EE-Capstone
This repository is the master collection of technical files (excluding reports and posters) for ECE490/491 group 1 capstone - Formula SAE EVCU (electric vehicle control unit) project. Group members are enrolled in electrical engineering at the U of A. Our valuable consultant and helpers are staff and instructors of ECE 490/491 as well as Le Minh Dang, a computer engineering student, all of whom are from the University of Alberta as well. 


## Hardware
Below is a high level hardware block diagram describing our fabricated PCB. Detailed 3d-view of the PCB and schematic can be found under Hardware Design/EVCU Rev B/EVCU. 

![image](https://user-images.githubusercontent.com/47064869/230803179-39cdc258-3503-49b4-8827-90abdd42a2b8.png)
Design created by: Thomas Osei-Bonsu Jr. and Katie Lin

## Software
Below is a high level software flowchart block diagram describing our software control flow. Detailed Documentation about code can be found below. We are using STM32Cube IDE to develop our code currently, so our configuration for our software has mostly been STM32Cube IDE presets.

![image](https://user-images.githubusercontent.com/47064869/230803031-e13324f6-1f2f-4af6-9df3-3e316a8646e2.png)
Design created by: Katie Lin and Thomas Osei-Bonsu Jr.

**Libraries/modules/middlewares we used:** 

+ **FreeRTOS**: for scheduling tasks and other parallel programming functionalities

+ **FATFS**: for communications and r/w functions with the SD card


**Current code structure**:

```STM32_test.ioc```: Settings file for pin configurations, turning peripherials on/off with STM32Cube IDE...etc.

```main.c```: pin defines, generation of global tasks/mutexes/threads and starting the kernel. Also imports all the other functions and runs them. 

```SD_task.c && SD_task.h```: calls functions to perform r/w functions to and from sd card, as well as mount/dismount...etc. 

```ADC_task.c && ADC_task.h```: processes values going into ADC pins and saves them in an intermediate buffer for further logic/processing. 

```CAN_task.c && CAN_task.h```: processes operations relevant to STM32 BxCAN functionalities, such as sending and receiving CAN messages, encoding and decoding of such messages...etc. 


```controller_task.c && controller_task.h```: a giant state machine with cases, interrupts...etc that links all the tasks together. Based on the inputs and outputs of the task files described above, decide the next steps of what the MCU needs to do, usually sending CAN messages and error logging upon error detection. 
