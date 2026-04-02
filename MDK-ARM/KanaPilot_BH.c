/*
WA30 ACM
*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//float sens_data[6];

uint8_t packetCount;
extern bool isRearRadarDetected;
extern uint8_t soundHornTimes;
uint8_t hornOnOffCnt;
#define HORN_ON_DUTY 2
#define HORN_OFF_DUTY 4

bool isEmgStop=false;

void initKanaPilotBH(void){
  u2RxInterruptEnable();
	
	//	pato1On();
	pato1Off();  //debug
	pato2Off();
	pato3Off();
	
	//all free
	//allFreeOn();
/*	
	sens_data[0] = 0.0;
	sens_data[1] = 0.0;
	sens_data[2] = 0.0;
	sens_data[3] = 0.0;
	sens_data[4] = 0.0;
	sens_data[5] = 0.0;
*/
//	DIO = 0x00000000;
	packetCount=0;
	
	//set steering speed (port, id, speed)
	setContinuousSpeed(4, 1, CONT_SPEED);
}

uint32_t DIO;
extern float Pitch,Roll,SwingRate,BoomAngle,ArmAngle,BacketAngle,RotationAngle,SensorTemp;
extern uint16_t BoomLength,ArmLength,BacketLength;
extern uint8_t IsRotationInit;			//0:no initialised 1:initialized 
extern uint8_t RotationDirection;  	//0:left 1:right *just previous movement

uint8_t dt[256];
void sendSensData(void){
	u2RxLedOn();
	uint8_t temp[4];
//	int i;
	dt[0] = 0xFF;
	dt[1] = packetCount; packetCount++;
	memcpy(temp, &Pitch, sizeof(float));
	dt[2] = temp[0];
	dt[3] = temp[1];
	dt[4] = temp[2];
	dt[5] = temp[3];
	memcpy(temp, &Roll, sizeof(float));
	dt[6] = temp[0];
	dt[7] = temp[1];
	dt[8] = temp[2];
	dt[9] = temp[3];
	memcpy(temp, &SwingRate, sizeof(float));
	dt[10] = temp[0];
	dt[11] = temp[1];
	dt[12] = temp[2];
	dt[13] = temp[3];
	memcpy(temp, &BoomAngle, sizeof(float));
	dt[14] = temp[0];
	dt[15] = temp[1];
	dt[16] = temp[2];
	dt[17] = temp[3];
	memcpy(temp, &ArmAngle, sizeof(float));
	dt[18] = temp[0];
	dt[19] = temp[1];
	dt[20] = temp[2];
	dt[21] = temp[3];
	memcpy(temp, &BacketAngle, sizeof(float));
	dt[22] = temp[0];
	dt[23] = temp[1];
	dt[24] = temp[2];
	dt[25] = temp[3];
	memcpy(temp, &RotationAngle, sizeof(float));
	dt[26] = temp[0];
	dt[27] = temp[1];
	dt[28] = temp[2];
	dt[29] = temp[3];
	memcpy(temp, &SensorTemp, sizeof(float));
	dt[30] = temp[0];
	dt[31] = temp[1];
	dt[32] = temp[2];
	dt[33] = temp[3];
	//attachment1
	dt[34] = 0;
	dt[35] = 0;
	dt[36] = 0;
	dt[37] = 0;
	//attachment2
	dt[38] = 0;
	dt[39] = 0;
	dt[40] = 0;
	dt[41] = 0;
	//backet tilt
	dt[42] = 0;
	dt[43] = 0;
	dt[44] = 0;
	dt[45] = 0;
	//backet rotate
	dt[46] = 0;
	dt[47] = 0;
	dt[48] = 0;
	dt[49] = 0;
	//haido ban
	dt[50] = 0;
	dt[51] = 0;
	dt[52] = 0;
	dt[53] = 0;
	//boom swing
	dt[54] = 0;
	dt[55] = 0;
	dt[56] = 0;
	dt[57] = 0;
	//arm swing
	dt[58] = 0;
	dt[59] = 0;
	dt[60] = 0;
	dt[61] = 0;
	//reserve
	dt[62] = 0;
	dt[63] = 0;
	dt[64] = 0;
	dt[65] = 0;
	//reserve
	dt[66] = 0;
	dt[67] = 0;
	dt[68] = 0;
	dt[69] = 0;
	memcpy(temp, &BoomLength, sizeof(uint16_t));
	dt[70] = temp[0];
	dt[71] = temp[1];
	memcpy(temp, &ArmLength, sizeof(uint16_t));
	dt[72] = temp[0];
	dt[73] = temp[1];
	memcpy(temp, &BacketLength, sizeof(uint16_t));
	dt[74] = temp[0];
	dt[75] = temp[1];
	//reserve
	dt[75]=0;
	dt[77]=0;
	//reserve
	dt[78]=0;
	dt[79]=0;
	//digital
	DIO=0;
	if(isEmgStop){
		DIO += 0x00000001;
	}
	if(IsRotationInit==0x01){
		DIO += 0x00000002;
	}
	if(RotationDirection==0x01){
		DIO += 0x00000004;
	}
	dt[80]=(uint8_t)((DIO>>24)&0x000000FF);
	dt[81]=(uint8_t)((DIO>>16)&0x000000FF);
	dt[82]=(uint8_t)((DIO>>8)&0x000000FF);
	dt[83]=(uint8_t)(DIO&0x000000FF);
	sendUart2Packet(dt, 84);
	u2RxLedOff();
}

//T2 100mS interval setting
void initTimers(void){
	//fck=16MHz=0.0625uS
	TIM2->PSC = 15999;  //fck/30=1count=0.0625uSx16000=1000uS/Count 1S=1000Count 
	//TIM2->ARR = 1000;  //1S interval
	TIM2->ARR = 100;  //100ms interval
	TIM2->CNT = 0;
	TIM2->DIER |= TIM_DIER_UIE;  //interrupt enable
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;  //auto-reload en, overflow update, timer enable	
}

//100mS inerval flg on
bool is100msInterval;
void T2_Callback(void){
	is100msInterval = true;
}




/**************************
main
**************************/
//#define COM_TIMEOUT 10  //around 1sec
//#define COM_TIMEOUT 15  //around 1.5sec
#define COM_TIMEOUT 20  //around 2.0sec
uint32_t comTimeCnt=0;
uint8_t ledStat;
uint8_t ledBlinkcounter;
#define LED_BLINK_DUTY 5  //x100mS
#define SENSOR_SEND_INTERVAL 1 //x100mS
uint8_t tes=0;
bool isCom=false;
bool preCom=false;
bool isEmgStopRequested=false;
uint32_t rearRadarDetectCnt=0;
uint8_t RQ=0;
void mainKanaPilotBH(void){
	
	//100msInterval
	if(is100msInterval){
		is100msInterval = false;
		resetWDT();
		
		//led blink
		ledBlinkcounter++;
		if(ledBlinkcounter > LED_BLINK_DUTY){
			ledBlinkcounter=0;
			if(ledStat==0){
				ledOn();
				ledStat=1;
			}else{
				ledOff();
				ledStat=0;
			}		
		}
		
		//horn check
		if(soundHornTimes>0){
			if(hornOnOffCnt<HORN_ON_DUTY){
				hornOn();
			}else{
				if(hornOnOffCnt<HORN_OFF_DUTY){
					hornOff();
				}else{
					//hornOff();
					soundHornTimes--;
					hornOnOffCnt=0;
				}
			}
			hornOnOffCnt++;
		}
		
		
		//sensor value send to PC
		sendSensData();
		
		//PC message check
//		checkPcCom();
		
		//main com timeout check
		comTimeCnt++;
		if(comTimeCnt > COM_TIMEOUT){
			if(isCom){
			  //stopAllControlAndResetStatus();  //---260106
				isEmgStopRequested=true;  //+++260106
				isCom=false;
			}
			comTimeCnt=0;
		}
		
	}
	
	//PC message check
	checkPcCom();
	//Sensor Read
	sensorsRead();  //rs485 sensors read request and parse
	
	//rear radar detect check
	switch(RQ){
		case 0:  //idle
			if(isRearRadarDetected){
				rearRadarDetectCnt++;
				if(rearRadarDetectCnt>3){  //bounce check
					isEmgStopRequested=true;
					RQ=1;
				}
			}else{
				rearRadarDetectCnt=0;
			}
			break;
		case 1:  //emg stopping
			if(!isRearRadarDetected){
				RQ=0;
			}
			break;
	}
	
	//emg stop rutine
	if(isEmgStopRequested){
		stopAllControlAndResetStatus();
		isEmgStopRequested=false;
	}
	
	wait10ms();
	wait10ms();

	
}
