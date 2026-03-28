#include "main.h"

uint8_t u1RxBuf[30];
uint32_t U1Q;
int32_t u1rxPos=0;
int32_t u1rxCnt=0;
uint8_t u1rxLen=0;
uint8_t u1CheckSum;
uint8_t u1LastByte;

int16_t AX,AY,AZ,WX,WY,WZ;
int16_t HX,HY,HZ,RO,PI,YA;

//6-axis sensor response ***
void U1RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART1);
//	u1RxBuf[u1rxPos] = rxData;
//	u1rxPos++;		
	switch(U1Q){
		case 0:
			if(rxData == 0x50){
				U1Q = 1;
			}
			break;
		case 1:
			if(rxData == 0x03){
				U1Q = 2;
			}else{
				U1Q = 0;
			}
			break;
		case 2:
      u1rxLen = rxData;
		  u1rxPos = 0;
		  U1Q = 3;
		  break;
		case 3:
			u1RxBuf[u1rxPos] = rxData;
		  u1rxPos++;
		  if(u1rxPos >= u1rxLen){
				U1Q = 4;
		  }
		  break;
      			
	}
}

/*
Send sensor datas
*/
uint8_t sensSeq=0;
void sendSensorData(void){
	u2TxLedOn();
	int i;
	uint8_t d[30];
	uint8_t SUM=0;
	if(U1Q == 4){
		AX = (((uint16_t)u1RxBuf[0])<<8) | (uint16_t)u1RxBuf[1];
		AY = (((uint16_t)u1RxBuf[2])<<8) | (uint16_t)u1RxBuf[3];
		AZ = (((uint16_t)u1RxBuf[4])<<8) | (uint16_t)u1RxBuf[5];
		WX = (((uint16_t)u1RxBuf[6])<<8) | (uint16_t)u1RxBuf[7];
		WY = (((uint16_t)u1RxBuf[8])<<8) | (uint16_t)u1RxBuf[9];
		WZ = (((uint16_t)u1RxBuf[10])<<8) | (uint16_t)u1RxBuf[11];
		HX = (((uint16_t)u1RxBuf[12])<<8) | (uint16_t)u1RxBuf[13];
		HY = (((uint16_t)u1RxBuf[14])<<8) | (uint16_t)u1RxBuf[15];
		HZ = (((uint16_t)u1RxBuf[16])<<8) | (uint16_t)u1RxBuf[17];
		PI = (((uint16_t)u1RxBuf[18])<<8) | (uint16_t)u1RxBuf[19];  //Pitch angle
		RO = (((uint16_t)u1RxBuf[20])<<8) | (uint16_t)u1RxBuf[21];  //Role angle
		YA = (((uint16_t)u1RxBuf[22])<<8) | (uint16_t)u1RxBuf[23];  //Yaw angle
		U1Q = 0;
	}
	d[0] = 0xFF;  //SYNC
	d[1] = 16;  //LEN
	d[2] = 0x0C;  //COM
	d[3] = sensSeq++;  //SEQ
	d[4] = (uint8_t)(AX>>8);
	d[5] = (uint8_t)(AX&0x00FF);
	d[6] = (uint8_t)(AY>>8);
	d[7] = (uint8_t)(AY&0x00FF);
	d[8] = (uint8_t)(AZ>>8);
	d[9] = (uint8_t)(AZ&0x00FF);
	d[10] = (uint8_t)(RO>>8);
	d[11] = (uint8_t)(RO&0x00FF);
	d[12] = (uint8_t)(PI>>8);
	d[13] = (uint8_t)(PI&0x00FF);
	d[14] = (uint8_t)(WZ>>8);
	d[15] = (uint8_t)(WZ&0x00FF);
	d[16] = 0x26; //DELIMITA
	for(i=0; i<17; i++){
		SUM += d[i];
	}
	d[17] = SUM;
	sendUart2Packet(d, 17);
  u2TxLedOff();
}

//unit degree
#define PI_MAX 20
#define PI_MIN -20
#define RO_MAX 20
#define RO_MIN -20


float mPITCH;
float mROLE;
//2022.11.10 add
bool isTiltStop(void){
	bool result = false;
	mPITCH = (((float)PI/32768)*180);  //formula is from WT901C Datasheet 6.3.3 Read Angle Output
	mROLE = ((float)RO/32768*180);
	if((mPITCH > PI_MAX) || (mPITCH < PI_MIN)){
		result = true;
	}
	if((mROLE > RO_MAX) || (mROLE < RO_MIN)){
		result = true;
	}

	return result;
}

bool isTiltSafe(void){
	bool result = false;
	mPITCH = ((float)PI/32768*180);  //formula is from WT901C Datasheet 6.3.3 Read Angle Output
	mROLE = ((float)RO/32768*180);
	if((mPITCH<PI_MAX-5) && (mPITCH>PI_MIN+5)){
		if((mROLE<RO_MAX-5) && (mROLE>RO_MIN+5)){
			result = true;
		}
	}
	return result;
}


