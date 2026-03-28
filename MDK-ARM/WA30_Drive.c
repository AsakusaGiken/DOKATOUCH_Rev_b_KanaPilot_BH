/*
WA30 Drive
*/
#include "main.h"

#define POS_OFFSET 150
#define SVCNT 8  //servo count for backhoe type
#define P4CNT 4  //servo count for port4(U4)
#define P5CNT 5  //servo count for port5(U5)
#define RETURN_PACKET_SIZE 9  //retrun packet size from servo driver
#define U4BUF_SIZE 100
#define U5BUF_SIZE 100

//servo limit
//minus value is span value  ++20241108
#define AC_MAX (2000)  //+
#define AC_MIN (0)  //-
#define BL_MAX (22000)  //+
#define BL_MIN (0)  //-
#define DP_MAX (4500)  //+ DUMP
#define DP_MIN (3500)  //-
/*
#define B2_MAX (1000)  //+
#define B2_MIN (1000)  //-
#define B3_MAX (1000)  //+
#define B3_MIN (1000)  //-
*/

//key position  ++20241108
#define KY_OFF 0
#define KY_ON (-8000)
#define KY_ST (-12000)

//FR position  ++20241108
#define FR_P 0
#define FR_R 4500
#define FR_N 8000
#define FR_D 11000
#define FR_2 14000
#define FR_1 16800


//servo gain
#define ST_MUX 50
#define AC_MUX 50
#define BL_MUX 50
#define FR_MUX 50
#define B1_MUX 50
#define B2_MUX 50
#define B3_MUX 50

//servo gain
#define STA_MUX 687

//servo ID map
#define ST_ID 1  //steering
#define AC_ID 2  //accel
#define BL_ID 3  //blake
#define FR_ID 4  //FR lever
#define B1_ID 5  //backet up and down
#define B2_ID 6  //backet right left
#define B3_ID 7  //backet sidedump
#define KY_ID 8  //engine key

//UART port map (U1,U4,U5 are RS485 ports)
#define ST_PT 4  //steering
#define AC_PT 4  //accel
#define BL_PT 4  //blake
#define FR_PT 4  //FR lever
#define B1_PT 4  //backet up and down
#define B2_PT 4  //backet right left
#define B3_PT 4  //backet sidedump
#define KY_PT 4  //engine key

//data position in acmPacketBuf[]
#define ST_POS 0  //steering
#define AC_POS 1  //accel
#define BL_POS 2  //blake
#define FR_POS 3  //FR lever
#define B1_POS 4  //backet up and down
#define B2_POS 5  //backet right left
#define B3_POS 6  //backet sidedump
#define KY_POS 7  //engine key

//DIO mask
#define DIO_KEYON 0x00000001
#define DIO_KEYST 0x00000002


//error bit mask
#define ERR_BIT_MSK 0x80

//target step
int32_t ST;  //staling
int32_t AC;  //Accel
int32_t BL;  //Blake
int32_t FR;  //FNR lever
int32_t B1;  //Bucket Right
int32_t B2;  //Bucket Middle
int32_t B3;  //Bucket Left
int32_t KY;  //Engine key


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
wait: send data -> retrun data  1cycle length
*/
void packetWait(void){
	resetWDT();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
}

void U4RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART4);
	u4RxBuf[u4rxPos] = rxData;
	u4rxPos++;
}

void U5RX_Callback(void){
	uint8_t rxData = LL_USART_ReceiveData8(USART5);
	u5RxBuf[u5rxPos] = rxData;
	u5rxPos++;	
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
	else if((d>145)&& (d<155)){  //like soft filter
		result = 150;
	}
	else if(d>250){
		result = 250;
	}
	return result;
}
uint8_t spanChk2(uint8_t d){
	uint8_t result;
	result = d;
	if(d<50){
		result = 50;
	}
//	else if((d>147)&& (d<153)){  //like soft filter
//		result = 150;
//	}
	else if(d>250){
		result = 250;
	}
	return result;
}

uint8_t spanChk3(uint8_t d){
	uint8_t result;
	result = d;
	if(d<50){
		result = 50;
	}
	else if((d>148)&& (d<152)){  //like soft filter
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
//	queZhomeRequest(4,1); packetWait();  //stalling never go Z-home
	queZhomeRequest(4,2); packetWait();
	queZhomeRequest(4,3); packetWait();
	queZhomeRequest(4,4); packetWait();
	queZhomeRequest(4,5); packetWait();
	queZhomeRequest(4,6); packetWait();
	queZhomeRequest(4,7); packetWait();
	queZhomeRequest(4,8); packetWait();
	wait1s();wait1s();
}

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

bool isHandleLimit(void){
	//PD13=D0 as right limit sw, PD12=D1 as left limit sw.
	if(((GPIOD->IDR & GPIO_IDR_ID13) != 0) ||((GPIOD->IDR & GPIO_IDR_ID12) != 0)){
	    return true;
	}else{
	  return false;
	}
}

void wa30Drive(uint8_t *servoPos, uint32_t dio){
	int i;
	int32_t tempVal,tempVal2;
	resetWDT();
	
	//steering
	tempVal = (((int32_t)(spanChk2(servoPos[ST_POS]))) - POS_OFFSET);  //handle is relative control
	if(tempVal > 0){
		if((GPIOD->IDR & GPIO_IDR_ID12) != 0){  //Limit SW check
			ST = 0;
		}else{
			ST = tempVal*STA_MUX;
		}
	}else if(tempVal < 0){
		if((GPIOD->IDR & GPIO_IDR_ID13) != 0){  //Limit SW check
			ST = 0;
		}else{
			ST = tempVal*STA_MUX;
		}
	}else{
		ST = 0;
	}
	
	//backet
	tempVal = (((int32_t)(spanChk3(servoPos[B1_POS]))) - POS_OFFSET);  //abs to int with offset(-150..0..+150)
	B1 = convToStep(tempVal, B1_MAX, B1_MIN);
	tempVal = (((int32_t)(spanChk3(servoPos[B2_POS]))) - POS_OFFSET);  //abs to int with offset(-150..0..+150)
	B2 = convToStep(tempVal, B2_MAX, B2_MIN);
	
	//Accel
  uint8_t dTemp;
	dTemp = spanChk(servoPos[AC_POS]);
	if (dTemp<150) dTemp = 150;
  if(dTemp>150){
		AC = (((int32_t)(dTemp-150))*(AC_MAX/100));
	}else if(dTemp<150){
		AC = (((int32_t)(150-dTemp)))*(AC_MAX/100);
	}else{
		AC = 0;
	}
	
	//Blake
	dTemp= spanChk(servoPos[BL_POS]);
	if(dTemp<148){
		if(dTemp < 75){
			BL = BL_MAX;
		}else{
			tempVal2 = (150-(int32_t)(dTemp));
			BL = tempVal2 * (BL_MAX/80);
		}
	}
	else{
		BL = 0;
	}
	
	//key
	if((dio & DIO_KEYST) != 0){
		KY = KY_ST;
		//emgSubRelayOn();  //foce cellmotor enable
	}else if((dio & DIO_KEYON) != 0){
		KY = KY_ON;
		//emgSubRelayOff();  //normal situation
	}else{
		KY = KY_OFF;
		//emgSubRelayOff();  //normal situation
	}	
	
	//FNR
	dTemp= spanChk(servoPos[FR_POS]);
	if(dTemp < 75){
		FR = FR_F;
	}else if(dTemp > 225){
		FR = FR_R;
	}else{
		FR = FR_N;
	}
	
	//send position to servos
	rxBufferPositionReset();
	//#1
	setPosAndReadStatusByDefaultSetting3(ST_PT, ST_ID, ST);  //port4 task
	//port5 task
	packetWait();
	//#2
	setPosAndReadStatusByDefaultSetting(AC_PT, AC_ID, AC);  //port4 task
	//port5 task
	packetWait();
	//#3
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, BL);  //port4 task
	//port5 task
	packetWait();
	//#4
	setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR);  //port4 task
	//port5 task
	packetWait();
	//#5
	setPosAndReadStatusByDefaultSetting(B1_PT, B1_ID, B1);  //port4 task
	//port5 task
	packetWait();
	//#6
	setPosAndReadStatusByDefaultSetting(B2_PT, B2_ID, B2);  //port4 task
	//port5 task
	packetWait();
	//#7
	setPosAndReadStatusByDefaultSetting(B3_PT, B3_ID, B3);  //port4 task
	//port5 task
	packetWait();
	//#8
	setPosAndReadStatusByDefaultSetting(KY_PT, KY_ID, KY);  //port4 task
	//port5 task
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
}



void emgStopPosition(void){
	rxBufferPositionReset();
	BL=BL_MAX;
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, BL);  //port4	
  packetWait();
	AC=0;
	setPosAndReadStatusByDefaultSetting(AC_PT, AC_ID, AC);  //port4	
  packetWait();
	B1=0;
	setPosAndReadStatusByDefaultSetting(B1_PT, B1_ID, B1);  //port4
	packetWait();
	B2=0;
	setPosAndReadStatusByDefaultSetting(B2_PT, B2_ID, B2);  //port4
	packetWait();
	B3=0;
	setPosAndReadStatusByDefaultSetting(B3_PT, B3_ID, B3);  //port4
	packetWait();
	FR=FR_N;
	setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR);  //port4
	packetWait();
}

