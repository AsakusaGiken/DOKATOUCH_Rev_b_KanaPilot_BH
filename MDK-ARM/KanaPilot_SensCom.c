#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define ID_SYATAI 0xE1
#define ID_BOOM 0xE2
#define ID_ARM 0xE3
#define ID_BACKET 0xE4
#define ID_ROTATION 0xE5
#define ID_BOOM_LEN 0xEA
#define ID_ARM_LEN 0xEB
#define ID_BACKET_LEN 0xEC
#define ID_RADAR 0xD7
#define COM_SENSREQ 0x1C

#define U1_BUFFER_SIZE 300
uint8_t u1RxBuf[U1_BUFFER_SIZE];
int32_t u1RxPos=0;
int32_t u1RxPrePos=0;
int32_t U1Q=0;
uint8_t u1RxSum,u1RxNum,u1RxLen,u1RxCnt;
uint8_t sensBuf[30];
uint8_t sensPos;

//Construction Machinery Specific Variables
float Pitch,Roll,SwingRate,BoomAngle,ArmAngle,BacketAngle,RotationAngle,SensorTemp;
uint16_t BoomLength,ArmLength,BacketLength;
uint8_t IsRotationInit=0x00;			//0:no initialised 1:initialized 
uint8_t RotationDirection=0x00;  	//0:left 1:right *just previous movement
bool isRearRadarDetected=false;

void U1RX_Callback(void){
	u1RxLedOn();
	uint8_t rxData = LL_USART_ReceiveData8(USART1);
	u1RxBuf[u1RxPos] = rxData;
	u1RxPos++;
	if(u1RxPos >= U1_BUFFER_SIZE) u1RxPos=0;
	u1RxLedOff();
}

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

/*******************
sensor read
*******************/
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
				  sensPos = 0;
				  u1RxCnt=0;
				  U1Q=2;
					break;
				case 2:
					sensBuf[sensPos] = u1RxBuf[myPos];
				  u1RxSum += sensBuf[sensPos];
				  sensPos++;
				  if(sensPos >= 30){
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
						if(sensBuf[0]==COM_SENSREQ){
							uint8_t temp[10];
							temp[0]=sensBuf[3]; temp[1]=sensBuf[4];
							temp[2]=sensBuf[5]; temp[3]=sensBuf[6];
							//id check
							//Syatai Angle
							if(sensBuf[2]==ID_SYATAI){
								memcpy(&Pitch, temp, sizeof(float));
								temp[0]=sensBuf[7]; temp[1]=sensBuf[8];temp[2]=sensBuf[9]; temp[3]=sensBuf[10];
								memcpy(&Roll, temp, sizeof(float));
								temp[0]=sensBuf[11]; temp[1]=sensBuf[12];temp[2]=sensBuf[13]; temp[3]=sensBuf[14];
								memcpy(&SwingRate, temp, sizeof(float));
								temp[0]=sensBuf[15]; temp[1]=sensBuf[16];temp[2]=sensBuf[17]; temp[3]=sensBuf[18];
								memcpy(&SensorTemp, temp, sizeof(float));
							}
							//Boom Angle
							else if(sensBuf[2]==ID_BOOM){
								memcpy(&BoomAngle, temp, sizeof(float));
							}
							//Arm Angle
							else if(sensBuf[2]==ID_ARM){
								memcpy(&ArmAngle, temp, sizeof(float));
							}
							//Backet Angle
							else if(sensBuf[2]==ID_BACKET){
								memcpy(&BacketAngle, temp, sizeof(float));						
							}
							//Rotation Angle
							else if(sensBuf[2]==ID_ROTATION){
								memcpy(&RotationAngle, temp, sizeof(float));
								IsRotationInit = sensBuf[7];
								RotationDirection = sensBuf[8];
							}
							//Boom Length
							else if(sensBuf[2]==ID_BOOM_LEN){
								BoomLength = ((uint16_t)(sensBuf[4])<<8)+(uint16_t)(sensBuf[3]);
							}
							//Arm Length
							else if(sensBuf[2]==ID_ARM_LEN){
								ArmLength = ((uint16_t)(sensBuf[4])<<8)+(uint16_t)(sensBuf[3]);
							}
							//Backet Length
							else if(sensBuf[2]==ID_BACKET_LEN){
								BacketLength = ((uint16_t)(sensBuf[4])<<8)+(uint16_t)(sensBuf[3]);
							}
							//Radar
							else if(sensBuf[2]==ID_RADAR){
								if(sensBuf[3]==0x00){
									isRearRadarDetected=false;
								}else{
									isRearRadarDetected=true;
								}
							}
							
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
		  sensorRequest(ID_SYATAI);
		  break;
		case 1:
		  sensorRequest(ID_BOOM);
			break;
		case 2:
			sensorRequest(ID_ARM);
			break;
		case 3:
			sensorRequest(ID_BACKET);
		  break;
		case 4:
			sensorRequest(ID_ROTATION);
			break;
		case 5:
			sensorRequest(ID_BOOM_LEN);
			break;
		case 6:
			sensorRequest(ID_ARM_LEN);
			break;
		case 7:
			sensorRequest(ID_BACKET_LEN);
			break;
		case 8:
			sensorRequest(ID_RADAR);
			break;
	}
	sensCnt++;
	if(sensCnt >= 9) sensCnt=0;
	checkSensorIncomming();
}

