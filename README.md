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


##User Manual (for EVCU PCB)

![image](https://user-images.githubusercontent.com/47064869/232248181-f159828e-b120-455e-b322-5c2face49e1b.png)

Above is a physical image of the EVCU PCB. 

**Hardware Setup**

The setup for EVCU PCB should ideally be done after reviewing the schematic and layout. For the EVCU PCB, provide a 12v DC input at J2 and ground signal. 


At J3, the APPS VPA/VPA2 and BSE sensor signal inputs to the ADC are labelled. Please consult the accelerator pedal datasheets to hook the APPS pedal/BSE sensors up correctly. 


The J5 DB9 connector is the CAN connector that can be hooked up to the motor controller. Our CAN bus speed is 0.5Mbit/second, so configure the motor controller accordingly. 


Before powering the board up, please ensure a microSD card is inserted and the sensors are powered and operational and connected to the analog inputs of this PCB. The current firmware will start running once the board is powered up. If any firmware errors occur, disconnect the PCB from power, ensure the hardware components are all connected, and reconnect the PCB to power. 


**Firmware Setup**

If one wishes to modify the firmware, please download STM32CubeIDE and clone this repository. Then, hook up serial wire debug pins in J1 to a debug programmer. These configurations can be found online. Then, clicking on the run or debug button will flash the updated code onto the EVCU PCB MCU. 

+ If the error conditions for the analog APPS/BSE sensors need to be modified, they can be changed in controller_task.c under the function check_error_condition(). Currently the code checks for APPS/BSE <0.5 or >4.5V as specified by the rules, but I believe the datasheet for one of the sensors says otherwise. 
+ If any CAN address/settings need to be modified, they can be changed in CAN_task.c

**Caveats**
+ If SD card logs aren't removed after powering off the board, they will be wiped at the next instance the board is powered up. 
+ SD card cannot be removed in the middle of operation. 
+ SD card might die due to too many writes, just replace a new micro SD card if you notice the EVCU firmware behaving weirdly with the SD card. 
