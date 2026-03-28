#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define U1_BUFFER_SIZE 300
uint8_t u1RxBuf[U1_BUFFER_SIZE];
int32_t u1RxPos=0;
int32_t u1RxPrePos=0;
int32_t U1Q=0;
uint8_t u1RxSum,u1RxNum,u1RxLen,u1RxCnt;
uint8_t acmSensBuf[30];
uint8_t acmSensPos;
extern float sens_data[6];
extern uint32_t DIO;
extern bool isRearRadarDetected;

void U1RX_Callback(void){
	u1RxLedOn();
	uint8_t rxData = LL_USART_ReceiveData8(USART1);
	u1RxBuf[u1RxPos] = rxData;
	u1RxPos++;
	if(u1RxPos >= U1_BUFFER_SIZE) u1RxPos=0;
	u1RxLedOff();
}

float senDbg[1000];
uint16_t senCnt=0;

/*******************
sensor read
*******************/
#define ID_SYATAI 0xE1
#define ID_BOOM 0xE2
#define ID_ARM 0xE3
#define ID_BACKET 0xE4
#define ID_ARTICULATE 0xE5
#define ID_ACBK 0xE6
#define ID_PROX1 0xD1
#define ID_PROX2 0xD2
#define ID_PROX3 0xD3
#define ID_PROX4 0xD4
#define ID_PROX5 0xD5
#define ID_PROX6 0xD6
#define ID_RADAR 0xD7
#define COM_SENSREQ 0x1C
#define DIOMSK_RLIM 0x01  //Right Limit SW
#define DIOMSK_LLIM 0x02  //Left Limit SW
#define DIOMSK_SUIHEI 0x04  //Suihei SW
#define DIOMSK_RLIM_SETPOS 0x00000002
#define DIOMSK_LLIM_SETPOS 0x00000004
#define DIOMSK_SUIHEI_SETPOS 0x00000008
#define DIOMSK_USFL_SETPOS 0x00000100
#define DIOMSK_USFM_SETPOS 0x00000200
#define DIOMSK_USFR_SETPOS 0x00000400
#define DIOMSK_TFFL_SETPOS 0x00000800
#define DIOMSK_TFFM_SETPOS 0x00001000
#define DIOMSK_TFFR_SETPOS 0x00002000
#define DIOMSK_USRL_SETPOS 0x00004000
#define DIOMSK_USRM_SETPOS 0x00008000
#define DIOMSK_USRR_SETPOS 0x00010000
#define DIOMSK_TFRL_SETPOS 0x00020000
#define DIOMSK_TFRM_SETPOS 0x00040000
#define DIOMSK_TFRR_SETPOS 0x00080000

#define PROX_TH 1000  //1000mm threshold


uint8_t sqCnt;
void sensorRequest(uint8_t id){
	u1TxLedOn();
	uint8_t d[10];
	int i;
	uint8_t sum=0;
	d[0] = 0xFF;  //sync
	d[1] = 0x05;  //len
	d[2] = 0x1C;  //com
	d[3] = sqCnt; sqCnt++;  //seq counter
	d[4] = id;
	d[5] = 0x26; //del
	for(i=0; i<6; i++){
		sum += d[i];
	}
	d[6] = sum;
	
	u1TxEnable();
	wait10us();wait10us();wait10us();
	sendUart1Packet(d, 7);
	wait10us();wait10us();wait10us();
	u1TxDisable();
	u1TxLedOff();
}


int debugVal;  //debug +++251206

float preArt2;
float nowArt2;
void checkSensorIncomming(void){
	int i,myPos;
	int32_t serchLength=0;
	if(u1RxPos>u1RxPrePos){
		serchLength = u1RxPos - u1RxPrePos;
	}else if(u1RxPos < u1RxPrePos){
		serchLength = (int32_t)(U1_BUFFER_SIZE) - u1RxPrePos + u1RxPos;
	}
	myPos = u1RxPrePos;
	u1RxPrePos = u1RxPos;
	if(serchLength > 0){
		resetWDT();
		for(i=0; i<serchLength; i++){
			switch(U1Q){
				//idle
				case 0:
					if(u1RxBuf[myPos]==0xFF){
						u1RxSum=0xFF;
						U1Q=1;
					}
					break;
				case 1:
					u1RxLen = u1RxBuf[myPos];
				  u1RxSum += u1RxLen;
				  acmSensPos = 0;
				  u1RxCnt=0;
				  U1Q=2;
					break;
				case 2:
					acmSensBuf[acmSensPos] = u1RxBuf[myPos];
				  u1RxSum += acmSensBuf[acmSensPos];
				  acmSensPos++;
				  if(acmSensPos >= 30){
						U1Q=0;
					}else{
						u1RxCnt++;
						if(u1RxCnt >= (u1RxLen-2)){
							U1Q = 3;
						}
					}
					break;
				case 3:
					if(u1RxBuf[myPos] == 0x26){
						u1RxSum += u1RxBuf[myPos];
						U1Q = 4;
					}else{
						U1Q=0;
					}
					break;
				case 4:
					if(u1RxSum == u1RxBuf[myPos]){
						//command check
						if(acmSensBuf[0]==COM_SENSREQ){
							uint8_t temp[4];
							temp[0]=acmSensBuf[3]; temp[1]=acmSensBuf[4];
							temp[2]=acmSensBuf[5]; temp[3]=acmSensBuf[6];
							//id check
							//WL Boom and Backet
							if(acmSensBuf[2]==ID_BOOM){
								memcpy(&sens_data[1], temp, sizeof(float));
								//sens_data[1] = (float)(((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);
							}else if(acmSensBuf[2]==ID_BACKET){
								//sens_data[2] = (float)(((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);
								memcpy(&sens_data[2], temp, sizeof(float));
							}
							//Articurate
							if(acmSensBuf[2]==ID_ARTICULATE){
								//sens_data[0] = (float)(((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);
								memcpy(&sens_data[0], temp, sizeof(float));
								
								//+++251206 articulate sensor debug
//								if(sens_data[0] < -40){
//									debugVal++;
//								}
								
								/* ---251120
								if((acmSensBuf[7]&DIOMSK_RLIM)!= 0x00){
									DIO |= (DIOMSK_RLIM_SETPOS);
								}else{
									DIO &= ~(DIOMSK_RLIM_SETPOS);
								}
								if((acmSensBuf[7]&DIOMSK_LLIM)!= 0x00){
									DIO |= (DIOMSK_LLIM_SETPOS);
								}else{
									DIO &= ~(DIOMSK_LLIM_SETPOS);
								}
								if((acmSensBuf[7]&DIOMSK_SUIHEI)!= 0x00){
									DIO |= (DIOMSK_SUIHEI_SETPOS);
								}else{
									DIO &= ~(DIOMSK_SUIHEI_SETPOS);
								}
								*/
							}
							//Accel and Blake
							if(acmSensBuf[2]==ID_ACBK){
								sens_data[3] = (float)(((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);  //accel
								sens_data[4] = (float)(((uint16_t)acmSensBuf[5]<<8) + (uint16_t)acmSensBuf[6]);  //blake
							}
							
							//Radar
							if(acmSensBuf[2]==ID_RADAR){
								if(acmSensBuf[3]==0x00){
									isRearRadarDetected=false;
								}else{
									isRearRadarDetected=true;
								}
							}
							
							/*
							//Prox1
							if(acmSensBuf[2]==ID_PROX1){
								uint16_t sens1 = (((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);  //sensor1
								uint16_t sens2 = (((uint16_t)acmSensBuf[5]<<8) + (uint16_t)acmSensBuf[6]);  //sensor2
								uint16_t sens3 = (((uint16_t)acmSensBuf[7]<<8) + (uint16_t)acmSensBuf[8]);  //sensor3	
								DIO = (sens1 < PROX_TH) ? (DIO | DIOMSK_USFL_SETPOS) : (DIO & ~DIOMSK_USFL_SETPOS);
								DIO = (sens2 < PROX_TH) ? (DIO | DIOMSK_USFM_SETPOS) : (DIO & ~DIOMSK_USFM_SETPOS);
								DIO = (sens3 < PROX_TH) ? (DIO | DIOMSK_USFR_SETPOS) : (DIO & ~DIOMSK_USFR_SETPOS);
							}
							//Prox2
							if(acmSensBuf[2]==ID_PROX2){
								uint16_t sens1 = (((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);  //sensor1
								uint16_t sens2 = (((uint16_t)acmSensBuf[5]<<8) + (uint16_t)acmSensBuf[6]);  //sensor2
								uint16_t sens3 = (((uint16_t)acmSensBuf[7]<<8) + (uint16_t)acmSensBuf[8]);  //sensor3	
								DIO = (sens1 < PROX_TH) ? (DIO | DIOMSK_TFFL_SETPOS) : (DIO & ~DIOMSK_TFFL_SETPOS);
								DIO = (sens2 < PROX_TH) ? (DIO | DIOMSK_TFFM_SETPOS) : (DIO & ~DIOMSK_TFFM_SETPOS);
								DIO = (sens3 < PROX_TH) ? (DIO | DIOMSK_TFFR_SETPOS) : (DIO & ~DIOMSK_TFFR_SETPOS);
							}//Prox3
							if(acmSensBuf[2]==ID_PROX3){
								uint16_t sens1 = (((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);  //sensor1
								uint16_t sens2 = (((uint16_t)acmSensBuf[5]<<8) + (uint16_t)acmSensBuf[6]);  //sensor2
								uint16_t sens3 = (((uint16_t)acmSensBuf[7]<<8) + (uint16_t)acmSensBuf[8]);  //sensor3	
								DIO = (sens1 < PROX_TH) ? (DIO | DIOMSK_USRL_SETPOS) : (DIO & ~DIOMSK_USRL_SETPOS);
								DIO = (sens2 < PROX_TH) ? (DIO | DIOMSK_USRM_SETPOS) : (DIO & ~DIOMSK_USRM_SETPOS);
								DIO = (sens3 < PROX_TH) ? (DIO | DIOMSK_USRR_SETPOS) : (DIO & ~DIOMSK_USRR_SETPOS);
							}//Prox4
							if(acmSensBuf[2]==ID_PROX4){
								uint16_t sens1 = (((uint16_t)acmSensBuf[3]<<8) + (uint16_t)acmSensBuf[4]);  //sensor1
								uint16_t sens2 = (((uint16_t)acmSensBuf[5]<<8) + (uint16_t)acmSensBuf[6]);  //sensor2
								uint16_t sens3 = (((uint16_t)acmSensBuf[7]<<8) + (uint16_t)acmSensBuf[8]);  //sensor3	
								DIO = (sens1 < PROX_TH) ? (DIO | DIOMSK_TFRL_SETPOS) : (DIO & ~DIOMSK_TFRL_SETPOS);
								DIO = (sens2 < PROX_TH) ? (DIO | DIOMSK_TFRM_SETPOS) : (DIO & ~DIOMSK_TFRM_SETPOS);
								DIO = (sens3 < PROX_TH) ? (DIO | DIOMSK_TFRR_SETPOS) : (DIO & ~DIOMSK_TFRR_SETPOS);
							}
							//Prox4
							//Prox5
							*/
							
							
								
						}else{
							U1Q=0;
						}
					}
					U1Q=0;
					break;
			}//end switch
			myPos++;
			if(myPos>=U1_BUFFER_SIZE) myPos=0;
		}//end for
	}//end if
}

uint32_t sensCnt=0;
void sensorsRead(void){
	switch(sensCnt){
		case 0:
			//sensorRequest(ID_BOOM);
		  sensorRequest(ID_ARTICULATE);
		  break;
		case 1:
			//sensorRequest(ID_BACKET);
		  sensorRequest(ID_ACBK);
			break;
		case 2:
			sensorRequest(ID_RADAR);
			break;
		case 3:
//			sensorRequest(ID_PROX2);
		  break;
		case 4:
//			sensorRequest(ID_PROX3);
			break;
		case 5:
//			sensorRequest(ID_PROX4);
			break;
	}
	sensCnt++;
	if(sensCnt >= 6) sensCnt=0;
	checkSensorIncomming();
}

