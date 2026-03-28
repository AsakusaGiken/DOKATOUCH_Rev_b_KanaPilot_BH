#include "main.h"

//#define U2_BUFFER_SIZE 200
#define U2_BUFFER_SIZE 500  //+++251104

extern uint32_t comTimeCnt;
void acmCommandCheck(uint8_t*);
void excuteEvent(uint8_t*);
void gamepadDrive(uint8_t*);


uint8_t u2RxBuf[U2_BUFFER_SIZE];  //ring buffer
uint32_t U2Q=0;

int32_t u2RxPos=0;
int32_t u2RxPrePos=0;
uint8_t u2RxCnt=0;
uint8_t u2RxLen=0;
uint8_t u2RxNum=0;
uint8_t u2RxSum=0;
uint8_t acmPacketBuf[50];
uint8_t acmPacketPos=0;
uint32_t eventNum=0;
uint32_t acmDio=0;

int32_t u2rxCnt=0;
uint8_t u2rxLen=0;
uint8_t u2CheckSum;
uint8_t u2LastByte;
//uint8_t u2RxCounter;
void U2RX_Callback(void){
	u2RxLedOn();
	uint8_t rxData = LL_USART_ReceiveData8(USART2);
	u2RxBuf[u2RxPos] = rxData;
	u2RxPos++;
	if(u2RxPos >= U2_BUFFER_SIZE) u2RxPos=0;
	u2RxLedOff();
}

//FY2025 Change to Direct drive format
extern bool isCom;  //for keepalive in wa30_main()
void checkAcmIncomming(void){
	int i,myPos;
	int32_t serchLength=0;
	if(u2RxPos>u2RxPrePos){
		serchLength = u2RxPos - u2RxPrePos;
	}else if(u2RxPos < u2RxPrePos){
		serchLength = (int32_t)(U2_BUFFER_SIZE) - u2RxPrePos + u2RxPos;
	}
	myPos = u2RxPrePos;
	u2RxPrePos = u2RxPos;
	if(serchLength > 0){
		resetWDT();
		//serch main loothine
		for(i=0; i<serchLength; i++){
			switch(U2Q){
				//idle
				case 0:
					if(u2RxBuf[myPos]==0xFF){
						u2RxSum=0xFF;
						U2Q=1;
					}break;
				case 1:
					u2RxLen = u2RxBuf[myPos];
				  u2RxSum += u2RxLen;
				  u2RxCnt = 0;
				  acmPacketPos=0;
				  U2Q=2;
				  break;
				case 2:
					acmPacketBuf[acmPacketPos] = u2RxBuf[myPos];
				  u2RxSum += acmPacketBuf[acmPacketPos];
				  acmPacketPos++;
					if(acmPacketPos>30){
						U2Q=0;
					}else{
						u2RxCnt++;
						if(u2RxCnt >= (u2RxLen-2)){  //by delimiter and sum
							U2Q=3;
						}
					}
					break;
				//delimiter
				case 3:
					if(u2RxBuf[myPos] == 0x26){
						u2RxSum += u2RxBuf[myPos];
						U2Q=4;
					}else{
						U2Q=0;
					}
					break;
				//check sum
				case 4:
					if(u2RxSum != u2RxBuf[myPos]){
						U2Q=0;
					}else{
						if(acmPacketBuf[0]==0x21){
							gamepadDrive(acmPacketBuf);
							pato2On();
							comTimeCnt=0;
							isCom=true;
						}else if(acmPacketBuf[0]==0x22){
							cat725DirectServoDrive(acmPacketBuf);
							pato2On();
							comTimeCnt=0;
							isCom=true;
						}else if(acmPacketBuf[0]==0x3A){  //Nop
							//do nothing. just turn the flags below
							pato2On();
							comTimeCnt=0;
							isCom=true;
						}
					}
					U2Q=0;
				break;
			}//switch end
			myPos++;
			if(myPos>=U2_BUFFER_SIZE) myPos=0;
		}//for end
	}
}

void acmCommandCheck(uint8_t* buf){
	uint8_t myCommand = buf[0];
	switch(myCommand){
		case 0x21:
			break;
		
	}
	
}

void gamepadDrive(uint8_t* inBuf){
	uint8_t d[30];
	d[0]=inBuf[2];
	d[1]=inBuf[3];
	d[2]=inBuf[4];
	//acmPacketBuf[0] as servo[0] Pos, same as others
	acmDio = (uint32_t)inBuf[19]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[20]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[21]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[22];
	cat725GamepadDrive(d, acmDio);
}



//FY2025 Direct Drive command add -> below comment out 
/*
extern bool isCom;
void checkAcmIncomming(void){
	int i,myPos;
	int32_t serchLength=0;
	if(u2RxPos>u2RxPrePos){
		serchLength = u2RxPos - u2RxPrePos;
	}else if(u2RxPos < u2RxPrePos){
		serchLength = (int32_t)(U2_BUFFER_SIZE) - u2RxPrePos + u2RxPos;
	}
	myPos = u2RxPrePos;
	u2RxPrePos = u2RxPos;
	if(serchLength > 0){
		resetWDT();
		//serch main loothine
		for(i=0; i<serchLength; i++){
			switch(U2Q){
				//idle
				case 0:
					if(u2RxBuf[myPos]==0xFF){
						u2RxSum=0xFF;
						U2Q=1;
					}break;
				case 1:
					u2RxNum = u2RxBuf[myPos];
				  u2RxSum += u2RxNum;
				  U2Q=2;
				  break;
				case 2:
					u2RxLen = u2RxBuf[myPos];
				  u2RxSum += u2RxLen;
				  u2RxCnt = 0;
				  acmPacketPos=0;
				  U2Q=3;
				  break;
				case 3:
					acmPacketBuf[acmPacketPos] = u2RxBuf[myPos];
				  u2RxSum += acmPacketBuf[acmPacketPos];
				  acmPacketPos++;
					if(acmPacketPos>30){
						U2Q=0;
					}else{
						u2RxCnt++;
						if(u2RxCnt >= (u2RxLen-2)){  //by delimiter and sum
							U2Q=4;
						}
					}
					break;
				//delimiter
				case 4:
					if(u2RxBuf[myPos] == 0x26){
						u2RxSum += u2RxBuf[myPos];
						U2Q=5;
					}else{
						U2Q=0;
					}
					break;
				//check sum
				case 5:
					if(u2RxSum != u2RxBuf[myPos]){
						U2Q=0;
					}else{
						if(u2RxLen==6){  //event command
							excuteEvent(acmPacketBuf);
						}else if(u2RxLen==26){//direct drive command
							excuteDirectDrive(acmPacketBuf);
						}
						pato2On();
						comTimeCnt=0;
						isCom=true;
					}
					U2Q=0;
				break;
			}//switch end
			myPos++;
			if(myPos>=U2_BUFFER_SIZE) myPos=0;
		}//for end
	}
}

void excuteEvent(uint8_t* inBuf){
	eventNum=(uint32_t)inBuf[0]; eventNum = eventNum<<8;
	eventNum+=(uint32_t)inBuf[1]; eventNum = eventNum<<8;
	eventNum+=(uint32_t)inBuf[2]; eventNum = eventNum<<8;
	eventNum+=(uint32_t)inBuf[3];
}



void excuteDirectDrive(uint8_t* inBuf){
	
	//acmPacketBuf[0] as servo[0] Pos, same as others
	acmDio = (uint32_t)inBuf[20]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[21]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[22]; acmDio = acmDio<<8;
	acmDio += (uint32_t)inBuf[23];
	cat725Drive(inBuf, acmDio);
	
}

*/

