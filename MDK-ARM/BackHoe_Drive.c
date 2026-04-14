/*
CAT725 Drive based on WA30 Drive
*/
#include "main.h"
#include <stdint.h>
#include <string.h>

#define POS_OFFSET 150
#define SVCNT 8  //servo count for normal bachhoe
#define P4CNT 8  //servo count for port4(U4)			//+++251120 solved frequentry PCB resets
//#define P5CNT 5  //servo count for port5(U5)		//---251120 solved frequentry PCB resets
#define RETURN_PACKET_SIZE 9  //retrun packet size from servo driver
//#define U4BUF_SIZE 256
#define U4BUF_SIZE 100
//#define U5BUF_SIZE 256

//servo limit
#define LX_IN (-1700)
#define LX_OUT (+1700)
#define LY_BACK (-1700)
#define LY_FRONT (+1700)
#define RX_IN (+1800)
#define RX_OUT (-1600)
#define RY_BACK (+1800)
#define RY_FRONT (-1600)
#define RR_FRONT (-3150)
#define RR_BACK (+3500)
#define LR_FRONT (+3200)
#define LR_BACK (-3400)
#define VL_MAX (-13000)
#define KY_OF (0)
#define KY_ON (-8300)
#define LY_ST (-14200)

//servo ID map
#define LX_ID 1  //left lever right&left
#define LY_ID 2  //left lever front&back
#define RX_ID 3  //right lever right&left
#define RY_ID 4  //right lever front&back
#define RR_ID 5  //right run front&back
#define LR_ID 6  //left run front&back
#define VL_ID 7  //volume
#define KY_ID 8  //engine key

//UART port map (U1,U4,U5 are RS485 ports)
#define LX_PT 4
#define LY_PT 4
#define RX_PT 4
#define RY_PT 4
#define RR_PT 4
#define LR_PT 4
#define VL_PT 4
#define KY_PT 4


//error bit mask
#define ERR_BIT_MSK 0x80

//target step
int32_t LX;
int32_t LY;
int32_t RX;
int32_t RY;
int32_t RR;
int32_t LR;
int32_t VL;
int32_t KY;

/*
uint8_t u5RxBuf[100];
uint32_t U5Q;
int32_t u5rxPos=0;
int32_t u5rxCnt=0;
uint8_t u5rxLen=0;
uint8_t u5CheckSum;
uint8_t u5LastByte;
*/

uint8_t u4RxBuf[U4BUF_SIZE];
uint32_t U4Q;
int32_t u4rxPos=0;
int32_t u4rxCnt=0;
uint8_t u4rxLen=0;
uint8_t u4CheckSum;
uint8_t u4LastByte;

uint8_t svStatus[SVCNT+1];  //servo status: svStatus[0] no use

/*
wait: send data -> retrun data  1cycle length
*/
//***
void packetWait(void){
	resetWDT();
//	wait1ms();wait1ms();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
//	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
}

void U4RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART4);
	u4RxBuf[u4rxPos] = rxData;
	u4rxPos++;
}

/*
void U5RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART5);
	u5RxBuf[u5rxPos] = rxData;
	u5rxPos++;	
}
*/

//buffer position reset
void rxBufferPositionReset(void){
	int i;
//	u1rxPos = 0;
//	u5rxPos = 0;
	u4rxPos = 0;
	for(i=0; i<U4BUF_SIZE; i++){
		u4RxBuf[i] = 0;
	}
//	for(i=0; i<U5BUF_SIZE; i++){
//		u5RxBuf[i] = 0;
//	}
}


/*
servo stick position data span check and soft filter
*/
uint8_t spanChk(uint8_t d){
	uint8_t result;
	result = d;
	if(d<50){
		result = 50;
	}
	else if((d>145)&& (d<155)){  //like soft filter
		result = 150;
	}
	else if(d>250){
		result = 250;
	}
	return result;
}

void allFreeOn(void){
	queFreeRequest(4,1); packetWait();
	queFreeRequest(4,2); packetWait();
	queFreeRequest(4,3); packetWait();
	queFreeRequest(4,4); packetWait();
	queFreeRequest(4,5); packetWait();
	queFreeRequest(4,6); packetWait();
	queFreeRequest(4,7); packetWait();
	queFreeRequest(4,8); packetWait();
}

void allFreeOff(void){
	queWriteRegister(4, 1, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 2, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 3, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 4, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 5, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 6, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 7, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 8, REG_IO_INPUT, 0x0000); packetWait();
}

//all Z-HOME
void allZhome(void){
	queZhomeRequest(4,1); packetWait();
	queZhomeRequest(4,2); packetWait();
	queZhomeRequest(4,3); packetWait();
	queZhomeRequest(4,4); packetWait();
	queZhomeRequest(4,5); packetWait();
	queZhomeRequest(4,6); packetWait();
	queZhomeRequest(4,7); packetWait();
//	queZhomeRequest(4,8); packetWait();  //key don't touch
	wait1s();wait1s();
}

/*
int32_t convToStep(int32_t val, int32_t maxVal, int32_t minVal){
	int32_t result;
	if(val > 0){
		result = (int32_t)((float)val * ((float)maxVal/(float)100));
	}else if(val<0){
		result = (int32_t)((float)val * ((float)minVal/(float)100));
	}else{
		result = 0;
	}
	return result;
}
*/


//extern uint32_t DIO;  //CAT725 sensing DIO 0x02=RightLimit 0x04=LeftLimit

void emgStopPosition(void){
	rxBufferPositionReset();
	LX=0;
//	setPosAndReadStatusByDefaultSetting(LX_PT, LX_ID, LX);  //port4	
	setPositionByDefaultSetting(LX_PT, LX_ID, LX);  //port4	
  packetWait();
	LY=0;
//	setPosAndReadStatusByDefaultSetting(LY_PT, LY_ID, LY);  //port4	
	setPositionByDefaultSetting(LY_PT, LY_ID, LY);  //port4	
  packetWait();
	RX=0;
//	setPosAndReadStatusByDefaultSetting(RX_PT, RX_ID, RX);  //port4	
	setPositionByDefaultSetting(RX_PT, RX_ID, RX);  //port4	
  packetWait();
	RY=0;
//	setPosAndReadStatusByDefaultSetting(RY_PT, RY_ID, RY);  //port4	
	setPositionByDefaultSetting(RY_PT, RY_ID, RY);  //port4
  packetWait();
	RR=0;
//	setPosAndReadStatusByDefaultSetting(RR_PT, RR_ID, RR);  //port4	
	setPositionByDefaultSetting(RR_PT, RR_ID, RR);  //port4	
  packetWait();
	LR=0;
//	setPosAndReadStatusByDefaultSetting(LR_PT, LR_ID, LR);  //port4	
	setPositionByDefaultSetting(LR_PT, LR_ID, LR);  //port4
  packetWait();
	VL=0;
//	setPosAndReadStatusByDefaultSetting(VL_PT, VL_ID, VL);  //port4	
	setPositionByDefaultSetting(VL_PT, VL_ID, VL);  //port4
  packetWait();
	//key don't thouch

}

void stopAllControlAndResetStatus(void){
	emgStopPosition();
	pato2Off();
}

/*
Invert the Kanatouch-controller stick value. 
50=250
60=190
..
150=150
..
240=60
250=50
*/
uint8_t invertStickValue(uint8_t d){
	int32_t temp;
	temp = (int32_t)d;
	temp = (250-d)+50;
	return (uint8_t)temp;
}


int32_t convToStep(int32_t val, int32_t a, int32_t b){
    int32_t result = 0;
    if(val > 0){
        result = val * (a / 100);
    } else if(val < 0){
        result = (-val) * (b / 100);
    }
    return result;
}

//+++260414
float tempVal;
void backhoeDirectServoDrive(uint8_t* inBuf){
	//culcurate command value to step
	int32_t tempVal;
	tempVal = (((int32_t)(spanChk(inBuf[2]))) - POS_OFFSET);  //convert to -100 to +100 val
	RX = convToStep(tempVal, RX_OUT, RX_IN);
	tempVal = (((int32_t)(spanChk(inBuf[3]))) - POS_OFFSET);  //convert to -100 to +100 val
	RY	= convToStep(tempVal, RY_FRONT, RY_BACK);
	tempVal = (((int32_t)(spanChk(inBuf[4]))) - POS_OFFSET);  //convert to -100 to +100 val
	LX = convToStep(tempVal, LX_IN, LX_OUT);
	tempVal = (((int32_t)(spanChk(inBuf[5]))) - POS_OFFSET);  //convert to -100 to +100 val
	LY	= convToStep(tempVal, LY_FRONT, LY_BACK);
	tempVal = (((int32_t)(spanChk(inBuf[6]))) - POS_OFFSET);  //convert to -100 to +100 val
	RR	= convToStep(tempVal, RR_FRONT, RR_BACK);
	tempVal = (((int32_t)(spanChk(inBuf[7]))) - POS_OFFSET);  //convert to -100 to +100 val
	LR	= convToStep(tempVal, LR_FRONT, LR_BACK);
	tempVal = (((int32_t)(spanChk(inBuf[8]))) - POS_OFFSET);  //convert to -100 to +100 val
	VL	= tempVal*(VL_MAX/100);
	
	//send position to servos
	rxBufferPositionReset();  //for servo error check receive   +++251120
//	setPosAndReadStatusByDefaultSetting(LX_PT, LX_ID, LX);  //port4
	setPositionByDefaultSetting(LX_PT, LX_ID, LX);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(LY_PT, LY_ID, LY);  //port4
	setPositionByDefaultSetting(LY_PT, LY_ID, LY);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(RX_PT, RX_ID, RX);  //port4
	setPositionByDefaultSetting(RX_PT, RX_ID, RX);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(RY_PT, RY_ID, RY);  //port4
	setPositionByDefaultSetting(RY_PT, RY_ID, RY);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(RR_PT, RR_ID, RR);  //port4
	setPositionByDefaultSetting(RR_PT, RR_ID, RR);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(LR_PT, LR_ID, LR);  //port4
	setPositionByDefaultSetting(LR_PT, LR_ID, LR);  //port4
	packetWait();
//	setPosAndReadStatusByDefaultSetting(VL_PT, VL_ID, VL);  //port4
	setPositionByDefaultSetting(VL_PT, VL_ID, VL);  //port4
	packetWait();

	//check servo error  +++251120
	/*
	int i;
	
	for(i=0; i<P4CNT; i++){
		if(((u4RxBuf[(i*RETURN_PACKET_SIZE)+6]) & ERR_BIT_MSK) != 0){
			
			queAlarmReset(4, u4RxBuf[i*RETURN_PACKET_SIZE]);
			packetWait();
		}
		svStatus[u4RxBuf[i*RETURN_PACKET_SIZE]] = u4RxBuf[(i*RETURN_PACKET_SIZE)+6];
	}
	*/
	
}

