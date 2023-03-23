/*
 * can_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "Open_SAE_J1939/Open_SAE_J1939.h"

uint8_t addressIndex;
uint32_t motorTemp_ID = 0x18FEA4FE;
uint32_t engineSpeed_ID = 0x18F004FE;
uint8_t data[];
// put in main
J1939 motorTemp_ECU = {0};
J1939 engineSpeed_ECU = {0};
int motorScale = 0.03125;
Open_SAE_J1939_Startup_ECU(&motorTemp_ECU);
Open_SAE_J1939_Startup_ECU(&engineSpeed_ECU);
Open_SAE_J1939_Listen_For_Messages(&motorTemp_ECU);
//

void CAN_init(J1939 *motorTemp_ECU, J1939 *engineSpeed_ECU) {
	for (addressIndex = 0; addressIndex < 255; addressIndex++) {
		motorTemp_ECU->other_ECU_address[addressIndex] = 0xFF;
		engineSpeed_ECU->other_ECU_address[addressIndex] = 0xFF;
	}
	motorTemp_ECU->ID = 0x18FEA4FE;
	engineSpeed_ECU->ID = 0x18F004FE;
	motorTemp_ECU.information_this_ECU.this_ECU_address = 0x15;
	engineSpeed_ECU.information_this_ECU.this_ECU_address = 0xBE;
}

int CAN_decode(*temperatureID, data[]){
	int temperature;
	CAN_Read_Message(*motorTemp_ECU.ID, motorTemp_ECU.data); //why is data unsigned 8
	temperature = data[1]*motorScale;
	return temperature;
}
void CAN_send(engineSpeed_ECU.ID, data[8]) {
	CAN_Send_Message(engineSpeed_ECU.ID, data[8]);
}

// read motor temperature address: 0x15
//  send engine speed data address: 0xBE
// define addresses here
// check if bx of fd can l4r9
// engine speed is 0xBEU or is it 0xBE ?
// I want to extract the motor temperature
//PGN_ENGINE_TEMPERATURE_1_65262 = 0x00FEEEU
//PGN_ELECTRONIC_ENGINE_CONTROLLER_1_61444 = 0x00F004U
