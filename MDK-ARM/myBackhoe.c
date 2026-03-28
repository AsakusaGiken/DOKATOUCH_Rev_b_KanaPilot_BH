/*
Kana-touch Backhoe
since 2021/4/22

USRT2:Communication to upper system
UART1:no use for backhoe(can use as RS485)
UART4:Servo Communication:ID1,2,7,8 (RS485)
UART5:Servo Communication:ID3,4,5,6,9 (RS485)
u4RxBuf:Return value from servos with status:ID1,2,7,8
u5RxBuf:Return value from servos with status:ID3,4,5,6,9

*/
#include "main.h"

#define POS_OFFSET 150
#define SVCNT 9  //servo count for backhoe type
#define P4CNT 4  //servo count for port4(U4)
#define P5CNT 5  //servo count for port5(U5)
#define RETURN_PACKET_SIZE 9  //retrun packet size from servo driver
#define U4BUF_SIZE 100
#define U5BUF_SIZE 100

extern uint8_t u2RxBuf[100];
extern uint32_t U2Q;
extern uint8_t u2CheckSum;
extern uint8_t u2LastByte;

bool isFree;
bool nowFree;
bool preFree;
bool isSvErr[SVCNT];


uint8_t u5RxBuf[100];
uint32_t U5Q;
int32_t u5rxPos=0;
int32_t u5rxCnt=0;
uint8_t u5rxLen=0;
uint8_t u5CheckSum;
uint8_t u5LastByte;

uint8_t u4RxBuf[100];
uint32_t U4Q;
int32_t u4rxPos=0;
int32_t u4rxCnt=0;
uint8_t u4rxLen=0;
uint8_t u4CheckSum;
uint8_t u4LastByte;

uint8_t svStatus[SVCNT+1];  //servo status: svStatus[0] no use

/*
Y:front/back 
X:right/left move
*/



//servo gain
//#define RY_MUX 50
#define RY_MUX 70
#define RX_MUX 125
#define LY_MUX 50
#define LX_MUX 50
#define RF_MUX 50
#define LF_MUX 50
#define VL_MUX 20
#define AT_MUX 50

//servo limit
//minus value is span value
//***210825 setting change
#define RY_MAX (1050)  //1050
#define RY_MIN (940)  //-940
#define RX_MAX (1220)
#define RX_MIN (1080)  //-1290
#define LY_MAX (830)
#define LY_MIN (910)  //-910
#define LX_MAX (1050)
#define LX_MIN (1120)  //-1500
#define RF_MAX (3700)
#define RF_MIN (3200)  //-3200
#define LF_MAX (3200)
#define LF_MIN (3200)  //-3200


#define VL_MAX (2050)

//key position
#define KY_OFF 0
#define KY_ON 690
#define KY_ST 1700

//servo ID map
#define RY_ID 1  //right Y
#define RX_ID 2  //right X
#define LY_ID 3  //left Y
#define LX_ID 4  //left X
#define RF_ID 5  //right Foot
#define LF_ID 6  //left Foot
#define VL_ID 7  //speed volume
#define KY_ID 8  //engine key
#define AT_ID 9  //attachment pedal

//UART port map (U1,U4,U5 are RS485 ports)
#define RY_PT 4  //right Y
#define RX_PT 4  //right X
#define LY_PT 5  //left Y
#define LX_PT 5  //left X
#define RF_PT 5  //right Foot
#define LF_PT 5  //left Foot
#define VL_PT 4  //speed volume
#define KY_PT 4  //engine key
#define AT_PT 5  //attachment pedal

//data position in u2RxBuf[]
#define RY_POS 2
#define RX_POS 3
#define LY_POS 4
#define LX_POS 5
#define RF_POS 8
#define LF_POS 9
#define VL_POS 7
#define AT_POS 6
#define DIO_POS 12  //LSB of DIO
#define DIO_POS2 11  //MSB of DIO

//DIO mask
#define DIO_KEYON 0x01
#define DIO_KEYST 0x02
#define DIO_ANZEN 0x04
#define DIO_TILTOFF 0x20

//error bit mask
#define ERR_BIT_MSK 0x80

//target step
int32_t RY;  //right Y
int32_t RX;  //right X
int32_t LY;  //left Y
int32_t LX;  //left X
int32_t RF;  //right Foot
int32_t LF;  //left Foot
int32_t VL;  //speed volume
int32_t KY;  //engine key
int32_t AT;  //attachment pedal

void backhoeInit(void){
	isFree = true;
	nowFree = true;
	preFree = true;
	
	//	pato1On();
	pato1Off();  //debug
	pato2Off();
	pato3Off();
	
	//all free
	allFreeOn();
	
	
}



/*
wait: send data -> retrun data  1cycle length
*/
void packetWait(void){
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
}

//buffer position reset
void rxBufferPositionReset(void){
	int i;
//	u1rxPos = 0;
	u5rxPos = 0;
	u4rxPos = 0;
	for(i=0; i<U4BUF_SIZE; i++){
		u4RxBuf[i] = 0;
	}
	for(i=0; i<U5BUF_SIZE; i++){
		u5RxBuf[i] = 0;
	}
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
	else if((d>147)&& (d<153)){  //like soft filter
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
	queFreeRequest(4,7); packetWait();
	queFreeRequest(4,8); packetWait();
	queFreeRequest(5,3); packetWait();
	queFreeRequest(5,4); packetWait();
	queFreeRequest(5,5); packetWait();
	queFreeRequest(5,6); packetWait();
	queFreeRequest(5,9); packetWait();
}

//need mask??
void allFreeOff(void){
	queWriteRegister(4, 1, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 2, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 7, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 8, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(5, 3, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(5, 4, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(5, 5, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(5, 6, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(5, 9, REG_IO_INPUT, 0x0000); packetWait();
}

//all Z-HOME
void allZhome(void){
	queZhomeRequest(4,1); packetWait();
	queZhomeRequest(4,2); packetWait();
	queZhomeRequest(4,7); packetWait();
	queZhomeRequest(4,8); packetWait();
	queZhomeRequest(5,3); packetWait();
	queZhomeRequest(5,4); packetWait();
	queZhomeRequest(5,5); packetWait();
	queZhomeRequest(5,6); packetWait();
	queZhomeRequest(5,9); packetWait();
	wait1s();wait1s();
}

int32_t convToStep(int32_t val, int32_t maxVal, int32_t minVal){
	int32_t result;
	if(val > 0){
		result = val * (maxVal/100);
	}else if(val<0){
		result = val * (minVal/100);
	}else{
		result = 0;
	}
	return result;
}

/*
foce the same situation of "Anzen-Lever Down"
*/
void emgSubRelayOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_13; wait10us();
	GPIOE->BSRR |= GPIO_BSRR_BS_7;
}	

/*
no effect for Anzen circuit
*/
void emgSubRelayOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_7; wait10us();
  GPIOE->BRR |= GPIO_BSRR_BS_13;
}	

uint32_t TILT_Q =0;
/*
All servo drive
takes 45mS
*/

void setNeutral(void){
	int32_t tempVal;
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);  //abs to int with offset(-150..0..+150)
	RY = convToStep(tempVal, RY_MAX, RY_MIN);
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);
	RX = convToStep(tempVal, RX_MAX, RX_MIN);
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);
	LY = convToStep(tempVal, LY_MAX, LY_MIN);
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);
	LX = convToStep(tempVal, LX_MAX, LX_MIN);
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);
	RF = convToStep(tempVal, RF_MAX, RF_MIN);
	tempVal = (((uint32_t)(spanChk(150))) - POS_OFFSET);
	LF = convToStep(tempVal, LF_MAX, LF_MIN);
	VL = (((uint32_t)(u2RxBuf[VL_POS]-50))*(VL_MAX/200));
	AT = 0;		
}

void setTransmittedValue(void){
	int32_t tempVal;
	tempVal = (((uint32_t)(spanChk(u2RxBuf[RY_POS]))) - POS_OFFSET);  //abs to int with offset(-150..0..+150)
	RY = convToStep(tempVal, RY_MAX, RY_MIN);
	tempVal = (((uint32_t)(spanChk(u2RxBuf[RX_POS]))) - POS_OFFSET);
	RX = convToStep(tempVal, RX_MAX, RX_MIN);
	tempVal = (((uint32_t)(spanChk(u2RxBuf[LY_POS]))) - POS_OFFSET);
	LY = convToStep(tempVal, LY_MAX, LY_MIN);
	tempVal = (((uint32_t)(spanChk(u2RxBuf[LX_POS]))) - POS_OFFSET);
	LX = convToStep(tempVal, LX_MAX, LX_MIN);
	tempVal = (((uint32_t)(spanChk(u2RxBuf[RF_POS]))) - POS_OFFSET);
	RF = convToStep(tempVal, RF_MAX, RF_MIN);
	tempVal = (((uint32_t)(spanChk(u2RxBuf[LF_POS]))) - POS_OFFSET);
	LF = convToStep(tempVal, LF_MAX, LF_MIN);
	VL = (((uint32_t)(u2RxBuf[VL_POS]-50))*(VL_MAX/200));
	AT = 0;	
}

bool isTiltStopDisableSwOn(void){
	bool result = false;
	if((u2RxBuf[DIO_POS2] & DIO_TILTOFF) != 0){
		result = true;
	}
	return result;
}

void allServoDrive(uint8_t *d){
	int i;
//	int32_t tempVal;

	switch(TILT_Q){
		//before tilt stop
		case 0:
			//tilt alarm
			if(isTiltStop()){
				//set neutral position
				setNeutral();
				TILT_Q = 1;
			}
			//normal operation
			else{
			//culc position value
				setTransmittedValue();
			}	
      break;
    //after tilt stop
    case 1:
		  if(isTiltStopDisableSwOn()){
				setTransmittedValue();
			}
      else{
				setNeutral();
      }	
			if(isTiltSafe()){
				TILT_Q = 0;
			}
      break;		
	}
	
	//check and set key position
	if((u2RxBuf[DIO_POS] & DIO_KEYST) != 0){
		KY = KY_ST;
		emgSubRelayOn();  //foce cellmotor enable
	}else if((u2RxBuf[DIO_POS] & DIO_KEYON) != 0){
		KY = KY_ON;
		emgSubRelayOff();  //normal situation
	}else{
		KY = KY_OFF;
		emgSubRelayOff();  //normal situation
	}	
	
	//send positon to servos
	//setPosAndReadStatusByDefaultSetting(uint8_t port, uint8_t id, uint32_t pos)
	rxBufferPositionReset();
	setPosAndReadStatusByDefaultSetting(RY_PT, RY_ID, RY);  //port4
	setPosAndReadStatusByDefaultSetting(LY_PT, LY_ID, LY);  //port5
	packetWait();
	setPosAndReadStatusByDefaultSetting(RX_PT, RX_ID, RX);  //port4
	setPosAndReadStatusByDefaultSetting(LX_PT, LX_ID, LX);  //port5
	packetWait();
	setPosAndReadStatusByDefaultSetting(VL_PT, VL_ID, VL);  //port4
	setPosAndReadStatusByDefaultSetting(RF_PT, RF_ID, RF);  //port5
  packetWait();
	setPosAndReadStatusByDefaultSetting(KY_PT, KY_ID, KY);  //port4
	setPosAndReadStatusByDefaultSetting(LF_PT, LF_ID, LF);  //port5
  packetWait();
  setPosAndReadStatusByDefaultSetting(AT_PT, AT_ID, AT);  //port5
	packetWait();
	
	//check servo error
	for(i=0; i<P4CNT; i++){
		if(((u4RxBuf[(i*RETURN_PACKET_SIZE)+6]) & ERR_BIT_MSK) != 0){
			
			queAlarmReset(4, u4RxBuf[i*RETURN_PACKET_SIZE]);
			packetWait();
		}
		svStatus[u4RxBuf[i*RETURN_PACKET_SIZE]] = u4RxBuf[(i*RETURN_PACKET_SIZE)+6];
	}
	for(i=0; i<P5CNT; i++){
		if(((u5RxBuf[(i*RETURN_PACKET_SIZE)+6]) & ERR_BIT_MSK) != 0){
			queAlarmReset(5, u5RxBuf[i*RETURN_PACKET_SIZE]);
			packetWait();
		}
		svStatus[u5RxBuf[i*RETURN_PACKET_SIZE]] = u5RxBuf[(i*RETURN_PACKET_SIZE)+6];
	}

	
	//free check
	if((u2RxBuf[DIO_POS] & DIO_ANZEN) != 0){
		nowFree = false;
	}
	else{
		nowFree = true;
	}
	//servo off
	if((preFree==false)&&(nowFree==true)){
		//***210707	
		//***Key only not z-home
		queZhomeRequest(4,1); packetWait();
		queZhomeRequest(4,2); packetWait();
		queZhomeRequest(4,7); packetWait();
//		queZhomeRequest(4,8); packetWait();  //key
		queZhomeRequest(5,3); packetWait();
		queZhomeRequest(5,4); packetWait();
		queZhomeRequest(5,5); packetWait();
		queZhomeRequest(5,6); packetWait();
		queZhomeRequest(5,9); packetWait();
		wait1s();wait1s();		
		//***210707		
		
		allFreeOn();
		pato3Off();
	}
	//servo on
	if((preFree==true)&&(nowFree==false)){
		allFreeOff();
//		allZhome();
		//***Key only not z-home
		queZhomeRequest(4,1); packetWait();
		queZhomeRequest(4,2); packetWait();
		queZhomeRequest(4,7); packetWait();
//		queZhomeRequest(4,8); packetWait();  //key
		queZhomeRequest(5,3); packetWait();
		queZhomeRequest(5,4); packetWait();
		queZhomeRequest(5,5); packetWait();
		queZhomeRequest(5,6); packetWait();
		queZhomeRequest(5,9); packetWait();
		wait1s();wait1s();

		pato2Off();
		pato3On();
	}
	preFree = nowFree;


}


//servo response
void U5RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART5);
	u5RxBuf[u5rxPos] = rxData;
	u5rxPos++;	
}

//servo response
void U4RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART4);
	u4RxBuf[u4rxPos] = rxData;
	u4rxPos++;	
}



void stopAllControlAndResetStatus(void){
	pato2On();  //disp error pato lamp
	allZhome();
	wait1s();wait1s();wait1s();
	allFreeOn();
	packetWait();
	pato3Off();
	preFree=true;
	nowFree=true;
}

/*
Handle sensors
*/
//debug
uint8_t dummyVal=0;
void dummySend(void){
	sendUart2Byte(dummyVal);
	dummyVal++;
}



void sensorQue(void){
//	dummySend();
	sendSensorData();
	WT901_queReadData();  //sensor read request for the next data
}

/*
Auto Calibration Button check
*/
bool isAutoCalibButtonPushed(void){
	bool result;
	if((GPIOC->IDR & GPIO_IDR_ID8)!=0){
		result = false;
	}else{
		result = true;
	}
	return result;
}

/*
main routine for backhoe
*/
void backhoe_1_main(void){
	//Auto Calibration check
	if(isAutoCalibButtonPushed()){
//		autoCalib();
		wait1s();
	}
	
	//Communication check
	if(U2Q == 3){
		  u2RxInterruptDisable();
		  ledOn();
		  if(u2LastByte == u2CheckSum){   //check sum check
				pato2Off();
				switch(u2RxBuf[0]){
					case 10:
						if(u2RxBuf[13] == 0x26){  //delimiter check
						  allServoDrive(u2RxBuf);
							sensorQue();
						}
						break;
				}
		  }
			u2RxInterruptEnable();
			ledOff();
			U2Q = 0;
		}
}
