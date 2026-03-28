/*
code for AZD-KRD stepper driver
HM-60252
ModbusRTU
*/
#include "main.h"

//Modbus function code
#define FUNCCODE_READ (0x03)
#define FUNCCODE_WRITE (0x06)
#define FUNCCODE_SEQ_WRITE (0x10)
#define FUNCCODE_SEQ_READ_WRITE (0x17)

#define DIRECT_DRIVE_TOP_ADDR (0x0058)

/********************
Query send
*********************/
void sendQuery(uint8_t port, uint8_t *data, uint8_t len){
  if(port==5){
		u5TxEnable();
		wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
    sendUart5Packet(data, len);
		wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
		u5TxDisable();
  }else if(port==4){
    u4TxEnable();
		wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
    sendUart4Packet(data, len);
		wait10us();wait10us();wait10us();  //Modbus silent interval 3.5char @230400bps = 15.19uS 
		u4TxDisable();
	}else if(port == 1){

	}		
}

/********************
function code 0x03
resister read
count:read resister count
*********************/
void queReadRegister(uint8_t port, uint8_t id, uint16_t addr, uint16_t count){
	uint8_t d[8];
	uint32_t crcVal;
	d[0] = id;
	d[1] = FUNCCODE_READ;
	d[2] = (uint8_t)(addr>>8);
	d[3] = (uint8_t)(addr&0x00FF);
	d[4] = (uint8_t)(count>>8);
	d[5] = (uint8_t)(count&0x00FF);
	crcVal = CRC16(d,6);
	d[6] = (uint8_t)(0x000000FF&crcVal);
	d[7] = (uint8_t)(crcVal>>8);
	sendQuery(port, d, 8);
}

/********************
function code 0x06
resister write
val:write value
*********************/
void queWriteRegister(uint8_t port, uint8_t id, uint16_t addr, int16_t val){
	uint8_t d[8];
	uint32_t crcVal;
	d[0] = id;
	d[1] = FUNCCODE_WRITE;
	d[2] = (uint8_t)(addr>>8);
	d[3] = (uint8_t)(addr&0x00FF);
	d[4] = (uint8_t)(val>>8);
	d[5] = (uint8_t)(val&0x00FF);
	crcVal = CRC16(d,6);
	d[6] = (uint8_t)(0x000000FF&crcVal);
	d[7] = (uint8_t)(crcVal>>8);
	sendQuery(port, d, 8);
}

/********************
function code 0x10
resister seq write
count:resister count(16bit resister count)
val:write value
*********************/
void queWriteRegisterSeq(uint8_t port, uint8_t id, uint16_t addr, uint16_t count, uint16_t* val){
	uint8_t d[100];
	uint32_t crcVal;
	int i;
	d[0] = id;
	d[1] = FUNCCODE_SEQ_WRITE;
	d[2] = (uint8_t)(addr>>8);
	d[3] = (uint8_t)(addr&0x00FF);
	d[4] = (uint8_t)(count>>8);
	d[5] = (uint8_t)(count&0x00FF);
	d[6] = (uint8_t)(count*2);
	for(i=0; i<count; i++){
		d[7+(i*2)] = (uint8_t)(val[i]>>8);
		d[8+(i*2)] = (uint8_t)(val[i]&0x00FF);
	}
	crcVal = CRC16(d,((count*2)+7));
	d[(count*2)+7] = (uint8_t)(0x000000FF&crcVal);
	d[(count*2)+8] = (uint8_t)(crcVal>>8);
	sendQuery(port, d, ((count*2)+9));
}


/********************
function code 0x17
resister seq read & write
rAddr:read top address
wAddr:write  top address
rCount:read resister count(16bit resister count)
wCount:write resister count(16bit resister count)
val:write value
*********************/
void queReadWriteRegisterSeq(uint8_t port, uint8_t id, 
	uint16_t rAddr, uint16_t rCount,
  uint16_t wAddr, uint16_t wCount,
  uint16_t* val)
{
	uint8_t d[100];
	uint32_t crcVal;
	int i;
	d[0] = id;
	d[1] = FUNCCODE_SEQ_READ_WRITE;
	d[2] = (uint8_t)(rAddr>>8);
	d[3] = (uint8_t)(rAddr&0x00FF);
	d[4] = (uint8_t)(rCount>>8);
	d[5] = (uint8_t)(rCount&0x00FF);
	d[6] = (uint8_t)(wAddr>>8);
	d[7] = (uint8_t)(wAddr&0x00FF);
	d[8] = (uint8_t)(wCount>>8);
	d[9] = (uint8_t)(wCount&0x00FF);
	d[10] = (uint8_t)(wCount*2);
	for(i=0; i<wCount; i++){
		d[11+(i*2)] = (uint8_t)(val[i]>>8);
		d[12+(i*2)] = (uint8_t)(val[i]&0x00FF);
	}
	crcVal = CRC16(d,((wCount*2)+11));
	d[(wCount*2)+11] = (uint8_t)(0x000000FF&crcVal);
	d[(wCount*2)+12] = (uint8_t)(crcVal>>8);
	sendQuery(port, d, ((wCount*2)+13));
}



/********************
Direct data drive,
HM-60252 p294
*********************/
void setDirectDataDrive(uint8_t port, uint8_t id,
	                uint32_t dataNum,
									uint32_t housiki,
									uint32_t pos,
									uint32_t speed,
									uint32_t kidouRate,
									uint32_t teisiRate,
									uint32_t current,
									uint32_t trigger)
{
	uint8_t d[100];
	uint32_t crcVal;
	uint32_t addrTop = DIRECT_DRIVE_TOP_ADDR;
	d[0] = id;
	d[1] = FUNCCODE_SEQ_WRITE;
	d[2] = (uint8_t)(addrTop>>8);
	d[3] = (uint8_t)(addrTop&0x00FF);
	d[4] = 0x00;  //count MSB
	d[5] = 0x10;  //count LSB
	d[6] = 0x20;  //countx2
	//Unen data num
	d[7] = (uint8_t)(dataNum>>24);
	d[8] = (uint8_t)(dataNum>>16);
	d[9] = (uint8_t)(dataNum>>8);
	d[10] = (uint8_t)(dataNum&0x000000FF);
	//housiki
	d[11] = (uint8_t)(housiki>>24);
	d[12] = (uint8_t)(housiki>>16);
	d[13] = (uint8_t)(housiki>>8);
	d[14] = (uint8_t)(housiki&0x000000FF);
	//ichi
	d[15] = (uint8_t)(pos>>24);
	d[16] = (uint8_t)(pos>>16);
	d[17] = (uint8_t)(pos>>8);
	d[18] = (uint8_t)(pos&0x000000FF);
	//sokudo
	d[19] = (uint8_t)(speed>>24);
	d[20] = (uint8_t)(speed>>16);
	d[21] = (uint8_t)(speed>>8);
	d[22] = (uint8_t)(speed&0x000000FF);
	//kidou rate
	d[23] = (uint8_t)(kidouRate>>24);
	d[24] = (uint8_t)(kidouRate>>16);
	d[25] = (uint8_t)(kidouRate>>8);
	d[26] = (uint8_t)(kidouRate&0x000000FF);
	//teisi rate
	d[27] = (uint8_t)(teisiRate>>24);
	d[28] = (uint8_t)(teisiRate>>16);
	d[29] = (uint8_t)(teisiRate>>8);
	d[30] = (uint8_t)(teisiRate&0x000000FF);
	//unten denryu
	d[31] = (uint8_t)(current>>24);
	d[32] = (uint8_t)(current>>16);
	d[33] = (uint8_t)(current>>8);
	d[34] = (uint8_t)(current&0x000000FF);
	//hanei trriger
	d[35] = (uint8_t)(trigger>>24);
	d[36] = (uint8_t)(trigger>>16);
	d[37] = (uint8_t)(trigger>>8);
	d[38] = (uint8_t)(trigger&0x000000FF);
	crcVal = CRC16(d,39);
	d[39] = (uint8_t)(0x000000FF&crcVal);
	d[40] = (uint8_t)(crcVal>>8);
	sendQuery(port, d, 41);
}

/********************
set position by default setting
estimate time = 5.5ms
*********************/
void setPositionByDefaultSetting(uint8_t port, uint8_t id, uint32_t pos){
//	setDirectDataDrive(port, id, 0, 0x00000001, pos, 20000, 1000000, 1000000, 1000, 1);  //normal
//	setDirectDataDrive(port, id, 0, 0x00000001, pos, 20000, 100000, 100000, 1000, 1);  //slow
	setDirectDataDrive(port, id, 0, 0x00000001, pos, 4000000, 30000000, 30000000, 1000, 1);  //high
}

/********************
IO status request
estimate time = 4ms
*********************/
void queIoStatusRequest(uint8_t port, uint8_t id){
	queReadRegister(port, id, REG_IO_STATUS, 2);
}

/********************
Alerm reset request
estimate time = 4.2ms
*********************/
void queAlarmReset(uint8_t port, uint8_t id){
	uint16_t val[2];
	val[0] = 0x0000;
	val[1] = 0x0002;
	queWriteRegisterSeq(port, id, REG_ALM_RESET, 2, val); 
}

/********************
Z-HOME request
estimate time = 4ms
*********************/
void queZhomeRequest(uint8_t port, uint8_t id){
	queWriteRegister(port, id, REG_IO_INPUT, 0x0000);
	wait10ms();
	queWriteRegister(port, id, REG_IO_INPUT, 0x0010);
}

/********************
FREE request
estimate time = 4ms
*********************/
void queFreeRequest(uint8_t port, uint8_t id){
	queWriteRegister(port, id, REG_IO_INPUT, 0x0040);
}


/********************

*********************/
void queSetposAndReadStatus(uint8_t port, uint8_t id,
	                uint32_t dataNum,
									uint32_t housiki,
									uint32_t pos,
									uint32_t speed,
									uint32_t kidouRate,
									uint32_t teisiRate,
									uint32_t current,
									uint32_t trigger)
{
//	uint32_t wAddrTop = DIRECT_DRIVE_TOP_ADDR;
  uint16_t wData[16];
	wData[0] = (uint16_t)(dataNum>>16);
	wData[1] = (uint16_t)(dataNum&0x0000FFFF);
	wData[2] = (uint16_t)(housiki>>16);
	wData[3] = (uint16_t)(housiki&0x0000FFFF);
	wData[4] = (uint16_t)(pos>>16);
	wData[5] = (uint16_t)(pos&0x0000FFFF);
	wData[6] = (uint16_t)(speed>>16);
	wData[7] = (uint16_t)(speed&0x0000FFFF);
	wData[8] = (uint16_t)(kidouRate>>16);
	wData[9] = (uint16_t)(kidouRate&0x0000FFFF);
	wData[10] = (uint16_t)(teisiRate>>16);
	wData[11] = (uint16_t)(teisiRate&0x0000FFFF);
	wData[12] = (uint16_t)(current>>16);
	wData[13] = (uint16_t)(current&0x0000FFFF);
	wData[14] = (uint16_t)(trigger>>16);
	wData[15] = (uint16_t)(trigger&0x0000FFFF);
	queReadWriteRegisterSeq(port, id, REG_IO_STATUS, 2, DIRECT_DRIVE_TOP_ADDR, 16, wData);
}

	
/*
Set servo position and status read request.
(queSetposAndReadStatus arguments)
uint8_t port,
uint8_t id,
uint32_t dataNum, //unten data number(always 0)
uint32_t housiki, //housiki 1:Absolute positon
uint32_t pos,  //target step value 
uint32_t speed,  //max 50000
uint32_t kidouRate,  //x1000 value of rate
uint32_t teisiRate,  //x1000 value of rate
uint32_t current,  //x10 value of %
uint32_t trigger  //drive trigger (must set '1' for start)
*/
void setPosAndReadStatusByDefaultSetting(uint8_t port, uint8_t id, uint32_t pos){
	queSetposAndReadStatus(port, id, 0, 0x00000001, pos, 20000, 150000, 150000, 1000, 1);
}

/*


*/

/*
***(absolute position slow setting)***
Set servo position and status read request.
(queSetposAndReadStatus arguments)
uint8_t port,
uint8_t id,
uint32_t dataNum, //unten data number(always 0)
uint32_t housiki, //housiki 1:Absolute positon
uint32_t pos,  //target step value 
uint32_t speed,  //max 50000  the bigger the faster
uint32_t kidouRate,  //x1000 value of rate  the bigger the faster
uint32_t teisiRate,  //x1000 value of rate  the bigger the faster
uint32_t current,  //x10 value of %
uint32_t trigger  //drive trigger (must set '1' for start)
*/
void setPosAndReadStatusByDefaultSetting3(uint8_t port, uint8_t id, uint32_t pos){
	queSetposAndReadStatus(port, id, 0, 0x00000002, pos, 5000, 5000, 5000, 1000, 1);
	//queSetposAndReadStatus(port, id, 0, 0x00000002, pos, 50000, 500000, 500000, 1000, 1);
	//queSetposAndReadStatus(port, id, 0, 0x00000002, pos, 10000, 150000, 150000, 1000, 1);
}

void setRelativePosAndReadStatus(uint8_t port, uint8_t id, int32_t pos, uint32_t speed, uint32_t late){
	queSetposAndReadStatus(port, id, 0, 0x00000002, pos, speed, late, late, 1000, 1);
}

//***continuous operation*** default speed is 5000
void setContinuousSpeed(uint8_t port, uint8_t id, uint32_t speed){
	uint16_t UntenSpeedAddr = 0x0480;
	uint16_t myCount = 2;
	uint16_t mySpeed[2];
	mySpeed[0] = (speed>>16);
	mySpeed[1] = (speed & 0x0000FFFF);
	queWriteRegisterSeq(port, id, UntenSpeedAddr, myCount, mySpeed);
}

void continuousDrive(uint8_t port, uint8_t id, int32_t dirction){
	//setContinuousSpeed(4, 1, CONT_SPEED);
	uint16_t driverComAddr = 0x007D;
	if(dirction > 0){
		queWriteRegister(port, id, driverComAddr, 0x4000);  //FW-POS ON:0x4000
	}else if(dirction < 0){
		queWriteRegister(port, id, driverComAddr, 0x8000);  //RV-POS ON:0x8000
	}else{
		queWriteRegister(port, id, driverComAddr, 0x0000);  //STOP:0x0000
	}
}





