# EE-Capstone
This repository is the master collection of technical files (excluding reports and posters) for ECE490/491 group 1 capstone - Formula SAE EVCU (electric vehicle control unit) project. Group members are enrolled in electrical engineering at the U of A. Our valuable consultant and helpers are staff and instructors of ECE 490/491 as well as Minh, a computer engineering student, all of whom are from the University of Alberta as well. 


## Hardware
Below is a high level hardware block diagram describing our fabricated PCB. Detailed 3d-view of the PCB and schematic can be found under Hardware Design/EVCU Rev B/EVCU. 

![image](https://user-images.githubusercontent.com/47064869/226085153-47a4a0b2-e5be-4171-b2b0-8e58c341f8d1.png)


## Software
Below is a high level software block diagram describing our software control flow. Detailed Documentation about code can be found below. We are using STM32Cube IDE to develop our code currently, so our configuration for our software has mostly been STM32Cube IDE presets.

![image](https://user-images.githubusercontent.com/47064869/226085287-a1d061f7-ef34-4d06-9ba2-6139424c0545.png)


**Libraries/modules/middlewares we used:** 

+ **FreeRTOS**: for scheduling tasks and other parallel programming functionalities

+ **FATFS**: for communications and r/w functions with the SD card

+ **OPEN SAE CAN J-1939**: CANBUS protocol that our drive inverter uses to communicate with EVCU and other CAN modules on vehicle. 


**Current code structure**:

```STM32_test.ioc```: Settings file for pin configurations, turning peripherials on/off with STM32Cube IDE...etc.

```main.c```: pin defines, generation of global tasks/mutexes/threads and starting the kernel. Also imports all the other functions and runs them. 

```SD_task.c && SD_task.h```: calls functions to perform r/w functions to and from sd card, as well as mount/dismount...etc. 

```ADC_task.c && ADC_task.h```: processes values going into ADC pins and saves them in an intermediate buffer for further logic/processing. 

```CAN_task.c && CAN_task.h```: processes operations relevant to CAN J-1939, such as sending and receiving CAN messages, encoding and decoding of such messages...etc. 


```controller_task.c && controller_task.h```: a giant state machine with cases, interrupts...etc that links all the tasks together. Based on the inputs and outputs of the task files described above, decide the next steps of what the MCU needs to do. 
