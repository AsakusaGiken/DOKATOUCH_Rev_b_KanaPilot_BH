/*
WA30 ACM
*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>



float sens_data[6];
uint32_t DIO;
uint8_t packetCount;
bool isRearRadarDetected=false;

void wa30Init(void){
  u2RxInterruptEnable();
	
	//	pato1On();
	pato1Off();  //debug
	pato2Off();
	pato3Off();
	
	//all free
	//allFreeOn();
	sens_data[0] = 0.0;
	sens_data[1] = 0.0;
	sens_data[2] = 0.0;
	sens_data[3] = 0.0;
	sens_data[4] = 0.0;
	sens_data[5] = 0.0;
	DIO = 0x00000000;
	packetCount=0;
	
	//set steering speed (port, id, speed)
	setContinuousSpeed(4, 1, CONT_SPEED);
}

uint8_t dt[30];
void sendSensData(void){
	u2RxLedOn();
	
	int i;
	dt[0] = 0xFF;
	dt[1] = packetCount; packetCount++;
	for(i=0; i<6; i++){
		uint8_t temp[4];
		memcpy(temp, &sens_data[i], sizeof(float));
		dt[i*4+2] = temp[0];
		dt[i*4+2+1] = temp[1];
		dt[i*4+2+2] = temp[2];
		dt[i*4+2+3] = temp[3];
	}
	//digital
	dt[29]=(uint8_t)((DIO>>24)&0x000000FF);
	dt[28]=(uint8_t)((DIO>>16)&0x000000FF);
	dt[27]=(uint8_t)((DIO>>8)&0x000000FF);
	dt[26]=(uint8_t)(DIO&0x000000FF);
	
	sendUart2Packet(dt, 30);
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
void wa30_main(void){
	
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
		
		//sensor value send to PC
		sendSensData();
		
		//ACM incomming message check
		checkAcmIncomming();
		
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
	
	sensorsRead();  //rs485 sensors read request and parse
	
	//rear radar detect check
	//+++260116
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
	//+++260116
	if(isEmgStopRequested){
		stopAllControlAndResetStatus();
		isEmgStopRequested=false;
	}
	
	wait10ms();
	wait10ms();  //+++261116
	//wait1ms();  //+++251104
	
	

	
}
