#include "main.h"

/*
void WT901_calibrateAcceleration(void){
//void modbusWrite(uint8_t id, uint16_t addr, int16_t val)	
	modbusWrite(0x50, 0x0069, 0xB588);  //unlock
	modbusWrite(0x50, 0x0001, 0x0001);  //foce caliblation mode
	wait1s();wait1s();wait1s();wait1s();wait1s();
	modbusWrite(0x50, 0x0001, 0x0000);  //foce normal mode
	modbusWrite(0x50, 0x0000, 0x0000);  //save
}

void WT901_queReadData(void){
//	modbusRead(0x50, 0x0034, 0x0006);  //AX-GZ
//		modbusRead(0x50, 0x0034, 0x0003);  //Acc
//		modbusRead(0x50, 0x003d, 0x0003);  //Angle
		modbusRead(0x50, 0x0034, 0x000C);  //AX-YAW
//			modbusRead(0x50, 0x0004, 0x0001);
}
*/

