#include "main.h"
/*
void modbusWrite(uint8_t id, uint16_t addr, int16_t val){
	uint8_t d[8];
	uint32_t crcVal;
	d[0] = id;
	d[1] = 0x06;  //modbus write
	d[2] = (uint8_t)(addr>>8);
	d[3] = (uint8_t)(addr&0x00FF);
	d[4] = (uint8_t)(val>>8);
	d[5] = (uint8_t)(val&0x00FF);
	crcVal = CRC16(d,6);
	d[6] = (uint8_t)(0x000000FF&crcVal);
	d[7] = (uint8_t)(crcVal>>8);
	u1TxEnable();
	wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
	sendUart1Packet(d, 8);
	wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
	u1TxDisable();	
}

void modbusRead(uint8_t id, uint16_t addr, int16_t val){
	uint8_t d[8];
	uint32_t crcVal;
	d[0] = id;
	d[1] = 0x03;  //modbus read
	d[2] = (uint8_t)(addr>>8);
	d[3] = (uint8_t)(addr&0x00FF);
	d[4] = (uint8_t)(val>>8);
	d[5] = (uint8_t)(val&0x00FF);
	crcVal = CRC16(d,6);
	d[6] = (uint8_t)(0x000000FF&crcVal);
	d[7] = (uint8_t)(crcVal>>8);
	u1TxEnable();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
	wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
	sendUart1Packet(d, 8);
	wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
	u1TxDisable();	
}
*/

