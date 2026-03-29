/*
CAT725 Drive based on WA30 Drive
*/
#include "main.h"
#include <stdint.h>
#include <string.h>

#define POS_OFFSET 150
#define SVCNT 6  //servo count for CAT725					//+++251120 solved frequentry PCB resets
#define P4CNT 6  //servo count for port4(U4)			//+++251120 solved frequentry PCB resets
//#define P5CNT 5  //servo count for port5(U5)		//---251120 solved frequentry PCB resets
#define RETURN_PACKET_SIZE 9  //retrun packet size from servo driver
#define U4BUF_SIZE 256
//#define U5BUF_SIZE 256

//servo limit
//minus value is span value  ++20241108
#define AC_MAX (2000)  //+
#define AC_MIN (0)  //-
#define BL_MAX (22000)  //+
#define BL_HLF (10000)
#define BL_MIN (0)  //-
#define DP_MAX (4500)  //+ DUMP
#define DP_STP (0)
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
#define AC_MUX 50
#define BL_MUX 50
#define DP_MUX 50


//servo gain
#define STA_MUX 687

//servo ID map
#define ST_ID 1  //steering
#define AC_ID 2  //accel
#define BL_ID 3  //blake
#define FR_ID 4  //FR lever
#define DP_ID 5  //backet up and down
//#define B2_ID 6  //backet right left
//#define B3_ID 7  //backet sidedump
#define KY_ID 8  //engine key

//UART port map (U1,U4,U5 are RS485 ports)
#define ST_PT 4  //steering
#define AC_PT 4  //accel
#define BL_PT 4  //blake
#define FR_PT 4  //FR lever
#define DP_PT 4  //backet up and down
//#define B2_PT 4  //backet right left
//#define B3_PT 4  //backet sidedump
#define KY_PT 4  //engine key

//data position in acmPacketBuf[]
/*
#define ST_POS 0  //steering
#define AC_POS 1  //accel
#define BL_POS 2  //blake
#define FR_POS 3  //FR lever
#define B1_POS 4  //backet up and down
#define B2_POS 5  //backet right left
#define B3_POS 6  //backet sidedump
#define KY_POS 7  //engine key
*/
#define X_POS 0
#define Y_POS 1
#define DP_POS 2

//DIO mask
#define DIO_KEYON 0x00000002
#define DIO_KEYST 0x00000004


//error bit mask
#define ERR_BIT_MSK 0x80

//target step
int32_t ST;  //staling
int32_t AC;  //Accel
int32_t BL;  //Blake
int32_t FR;  //FNR lever
int32_t DP;  //Bucket Right
//int32_t B2;  //Bucket Middle
//int32_t B3;  //Bucket Left
int32_t KY;  //Engine key

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
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
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
//	queFreeRequest(4,6); packetWait();
//	queFreeRequest(4,7); packetWait();
	queFreeRequest(4,8); packetWait();
}

void allFreeOff(void){
	queWriteRegister(4, 1, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 2, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 3, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 4, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 5, REG_IO_INPUT, 0x0000); packetWait();
//	queWriteRegister(4, 6, REG_IO_INPUT, 0x0000); packetWait();
//	queWriteRegister(4, 7, REG_IO_INPUT, 0x0000); packetWait();
	queWriteRegister(4, 8, REG_IO_INPUT, 0x0000); packetWait();
}

//all Z-HOME
void allZhome(void){
//	queZhomeRequest(4,1); packetWait();  //stalling never go Z-home
	queZhomeRequest(4,2); packetWait();
	queZhomeRequest(4,3); packetWait();
	queZhomeRequest(4,4); packetWait();
	queZhomeRequest(4,5); packetWait();
//	queZhomeRequest(4,6); packetWait();
//	queZhomeRequest(4,7); packetWait();
//	queZhomeRequest(4,8); packetWait();
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
	/*
	//PD13=D0 as right limit sw, PD12=D1 as left limit sw.
	if(((GPIOD->IDR & GPIO_IDR_ID13) != 0) ||((GPIOD->IDR & GPIO_IDR_ID12) != 0)){
	    return true;
	}else{
	  return false;
	}
	*/
	return false;
}

void dumpUp(void){
	setPosAndReadStatusByDefaultSetting(DP_PT, DP_ID, DP_MAX);  //port4 task
	packetWait();
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, BL_HLF);  //port4 task
	packetWait();
	setPosAndReadStatusByDefaultSetting(AC_PT, AC_ID, AC_MAX);  //port4 task
	packetWait();
}




#define R_LIM_MSK 0x00000002
#define L_LIM_MSK 0x00000004
#define DEAD_BAND 1.0
#define GAP_BIG 8.0
#define GAP_MID 4.0
#define GAP_SML 2.0
//#define SPD_HI 100000  //the higher the hi motor current
//#define SPD_HI 80000  //the higher the hi motor current
//#define SPD_HI 70000  //the higher the hi motor current
#define SPD_HI 30000  //the higher the hi motor current
#define SPD_MD 10000
#define SPD_LO 6000
#define SPD_SL 3000
extern uint32_t DIO;  //CAT725 sensing DIO 0x02=RightLimit 0x04=LeftLimit
extern float sens_data[6];  //CAT725 sterring angle=sens_data[0]
float nowAngle;
float distAngle;
float dumpAngle;
float distDumpAngle;
int32_t thisDirection=0;
bool isDumped=false;
uint32_t DQ=0;


void cat725GamepadDrive(uint8_t *servoPos, uint32_t dio){
	int i;
//	int32_t tempVal,tempVal2;
	resetWDT();
	
	//---steering---
	nowAngle = sens_data[0];
	float leverPos = ((float)servoPos[0])-150;  //will be +100~-100
	//distAngle =  leverPos/3.5;  //max=28.5deg
	//distAngle =  leverPos/2.8;  //max=35.7deg
	distAngle =  leverPos/2.7;  //max=37.0deg
	//distAngle =  leverPos/2.6;  //max=38.4deg
	//direction
	if((nowAngle+DEAD_BAND)<distAngle){
		if((DIO & R_LIM_MSK) == 0){  //R-LIMIT check
		  thisDirection=1;
		}else{
			thisDirection=0;  //no move
		}
	}else if((nowAngle-DEAD_BAND)>distAngle){
		if((DIO & L_LIM_MSK) == 0){  //R-LIMIT check
		  thisDirection=-1;
		}else{
			thisDirection=0;  //no move
		}
	}else{
		thisDirection=0;  //no move
	}
	//speed
	if(((nowAngle+GAP_BIG)<distAngle) || ((nowAngle-GAP_BIG)>distAngle)){
		ST=SPD_HI;
	}else if(((nowAngle+GAP_MID)<distAngle) || ((nowAngle-GAP_MID)>distAngle)){
		ST=SPD_MD;
	}else if(((nowAngle+GAP_SML)<distAngle) || ((nowAngle-GAP_SML)>distAngle)){
		ST=SPD_LO;
	}else{
		ST=SPD_SL;
	}	
	int32_t myRelatibePos = 5000*thisDirection;
	
	//setRelativePosAndReadStatus(ST_PT, ST_ID, myRelatibePos, ST, ((ST)/2));
	setRelativePosAndReadStatus(ST_PT, ST_ID, myRelatibePos, ST, ((ST)*2));

  uint8_t dTemp;
	//FNR and Accel and Blake
	dTemp= spanChk(servoPos[1]);
	if(dTemp < 145){
		AC = (((int32_t)(150-dTemp)))*(AC_MAX/100)/3;
		FR = FR_R;
		BL = BL_MIN;
	}else if(dTemp > 155){
		AC = (((int32_t)(dTemp-150))*(AC_MAX/100))/2;
		FR = FR_D;
		BL = BL_MIN;
	}else{
		FR = FR_N;
		AC = 0;
		BL = BL_HLF;
	}
	
	//dump
	dTemp= servoPos[2];
	dumpAngle = sens_data[2];
	distDumpAngle = 24.6 + (float)dTemp*0.0136;

	if((FR!=FR_R)&&(FR!=FR_D)){
		if(dTemp > 155){
			DP = (dTemp-150)*30;
			AC = AC_MAX;
		}else if(dTemp < 145){
			DP = (dTemp-150)*30;
		}else{
			DP = DP_STP;
		}
	}else{
		DP = DP_STP;
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
	

	
	//send position to servos
	rxBufferPositionReset();
	//#1
//	setPosAndReadStatusByDefaultSetting3(ST_PT, ST_ID, ST);  //port4 task
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
	setPosAndReadStatusByDefaultSetting(DP_PT, DP_ID, DP);  //port4 task
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
	/*
	for(i=0; i<P5CNT; i++){
		if(((u5RxBuf[(i*RETURN_PACKET_SIZE)+6]) & ERR_BIT_MSK) != 0){
			queAlarmReset(5, u5RxBuf[i*RETURN_PACKET_SIZE]);
			packetWait();
		}
		svStatus[u5RxBuf[i*RETURN_PACKET_SIZE]] = u5RxBuf[(i*RETURN_PACKET_SIZE)+6];
	}
	*/
}



void emgStopPosition(void){
	rxBufferPositionReset();
	//BL=BL_MAX;
	BL=BL_HLF;
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, BL);  //port4	
  packetWait();
	AC=0;
	setPosAndReadStatusByDefaultSetting(AC_PT, AC_ID, AC);  //port4	
  packetWait();
	DP=0;
	setPosAndReadStatusByDefaultSetting(DP_PT, DP_ID, DP);  //port4
	packetWait();
	FR=FR_N;
	setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR);  //port4
	/*
	B2=0;
	setPosAndReadStatusByDefaultSetting(B2_PT, B2_ID, B2);  //port4
	packetWait();
	B3=0;
	setPosAndReadStatusByDefaultSetting(B3_PT, B3_ID, B3);  //port4
	packetWait();
	*/
	wait1s();wait1s();
	BL=BL_MAX;
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, BL);  //port4	
	wait1s();
	//FR=FR_P;
	FR=FR_N;
	setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR);  //port4
	packetWait();
}

void stopAllControlAndResetStatus(void){
	emgStopPosition();
	pato2Off();
}

//pos span 0-100%
void setBrake(uint8_t pos){
	uint32_t step=0;
	step=(BL_MAX/100)*pos;
	setPosAndReadStatusByDefaultSetting(BL_PT, BL_ID, step);
	packetWait();
}

//pos span 0-100%
void setAccel(uint8_t pos){
	uint32_t step=0;
	step=(AC_MAX/100)*pos;
	setPosAndReadStatusByDefaultSetting(AC_PT, AC_ID, step);
	packetWait();
}

//str span 'P','R','N','D','2','1'
void setGear(uint8_t str){
	switch(str){
		case 0x50:  //'P'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_P);
			packetWait();
			break;
		case 0x52:  //'R'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_R);
			packetWait();
			break;
		case 0x4E:  //'N'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_N);
			packetWait();
			break;
		case 0x44:  //'D'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_D);
			packetWait();
			break;
		case 0x32:  //'2'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_2);
			packetWait();
			break;
		case 0x31:  //'1'
			setPosAndReadStatusByDefaultSetting(FR_PT, FR_ID, FR_1);
			packetWait();
			break;
	}	
}

//val span -900deg..+900deg
#define MOTOR_DEG_PER_PULSE    0.0075f  //1 pulse = 0.075deg as handle move angle
#define STEER_ARTANGLE_RATE 20  //steering angle / 20 = articulate angle
int handleStep;
float nowArt;
float deltaArt;
float distArt;
float targetArt;
int testFlg=0;
void setHandle(float val){
	//articulate limit check
	nowArt = sens_data[0];  //as articulate angle
	//if(((nowArt>-35.0)&&(val>0))&&((nowArt<35.0)&&(val<0))){
	if(((nowArt<-35.0)&&(val<0))||((nowArt>35.0)&&(val>0))){
		//none - out of articulate
		testFlg++;
	}else{
		testFlg--;
		deltaArt = val/STEER_ARTANGLE_RATE;
		distArt = nowArt+deltaArt;
		if(distArt<-35.0){
			//targetArt = distArt + 35.0;
			targetArt = (-35.0) + (nowArt*(-1));
		}else if(distArt>35.0){
			//targetArt = distArt - 35.0;
			targetArt = 35.0 - nowArt;
		}else{
			targetArt = deltaArt;
		}
		
		float pulses_f = (targetArt*STEER_ARTANGLE_RATE) / MOTOR_DEG_PER_PULSE;
		handleStep = (int)pulses_f;
		setRelativePosAndReadStatus(ST_PT, ST_ID, handleStep, SPD_HI, ((SPD_HI)*2));
		packetWait();
	}

}


//val span -35.0deg..+35.0deg
void setArticulate(float val){
	//articulate limit check
	nowArt = sens_data[0];  //as articulate angle
	if(((nowArt<-35.0)&&(val<0))||((nowArt>35.0)&&(val>0))){
		//none - out of articulate
	}else{
		targetArt = val - nowArt;	
		float pulses_f = (targetArt) / MOTOR_DEG_PER_PULSE * STEER_ARTANGLE_RATE;
		handleStep = (int)pulses_f;
		setRelativePosAndReadStatus(ST_PT, ST_ID, handleStep, SPD_HI, ((SPD_HI)*2));
		packetWait(); 
	}		
	
	/*
	//---steering---
	nowAngle = sens_data[0];
	distAngle = val;
	if(distAngle>35.0) distAngle=35.0;
	else if(distAngle<-35.0) distAngle=-35.0;
	//direction
	if((nowAngle+DEAD_BAND)<distAngle){
	  thisDirection=1;
	}else if((nowAngle-DEAD_BAND)>distAngle){
		thisDirection=-1;
	}else{
		thisDirection=0;  //no move
	}
	//speed
	if(((nowAngle+GAP_BIG)<distAngle) || ((nowAngle-GAP_BIG)>distAngle)){
		ST=SPD_HI;
	}else if(((nowAngle+GAP_MID)<distAngle) || ((nowAngle-GAP_MID)>distAngle)){
		ST=SPD_MD;
	}else if(((nowAngle+GAP_SML)<distAngle) || ((nowAngle-GAP_SML)>distAngle)){
		ST=SPD_LO;
	}else{
		ST=SPD_SL;
	}	
	int32_t myRelatibePos = 5000*thisDirection;
	setRelativePosAndReadStatus(ST_PT, ST_ID, myRelatibePos, ST, ((ST)*2));
	*/
}


//val span -100%..0%..+100%
void setDump(float val){
	int step;
	if(val==0){
		step=DP_STP;
	}else if(val>0){
		step=(DP_MAX/100)*val;
	}else{
		step=(DP_MIN/100)*val;  //will be minus value
	}
	setPosAndReadStatusByDefaultSetting(DP_PT, DP_ID, step);
	packetWait();
}

uint8_t dBRAKE;
float dHANDLE;
uint8_t dACCEL;

float tempVal;
void cat725DirectServoDrive(uint8_t* inBuf){
	float val;
	rxBufferPositionReset();  //for servo error check receive   +++251120
	switch(inBuf[2]){
		case 1:  //brake set
			setBrake(inBuf[3]);
			break;
		case 2:  //handle set
			memcpy(&val, &inBuf[3], sizeof(float));
		  setHandle(val);
			break;
		case 3:  //accel set
			setAccel(inBuf[3]);
		case 4:  //gear set
			setGear(inBuf[3]);
			break;
		case 5:  //dump set
			memcpy(&val, &inBuf[3], sizeof(float));
			setDump(val);
			break;
		case 6:  //articulate set
			memcpy(&val, &inBuf[3], sizeof(float));
		  setArticulate(val);
			break;
		case 10:  //all set(handle control)
			//brake
			setBrake(inBuf[3]);
		  dBRAKE = inBuf[3];  //debug +++251104
		  //handle
			memcpy(&val, &inBuf[4], sizeof(float));
		  tempVal = val;
		  setHandle(val);
		  dHANDLE = val;   //debug +++251104
		  //accel
			setAccel(inBuf[8]);
		  dACCEL = inBuf[8];  //debug +++251104
			//gear
			setGear(inBuf[9]);
		  //backet
		  memcpy(&val, &inBuf[10], sizeof(float));
			setDump(val);
			break;
		case 12:  //all set(articulate control)
			//brake
			setBrake(inBuf[3]);
		  //handle
			memcpy(&val, &inBuf[4], sizeof(float));
		  setArticulate(val);
		  //accel
			setAccel(inBuf[8]);
			//gear
			setGear(inBuf[9]);
		  //backet
		  memcpy(&val, &inBuf[10], sizeof(float));
			setDump(val);
			break;
		case 13:  //all set(handle control)
			//brake
			setBrake(inBuf[3]);
		  dBRAKE = inBuf[3];  //debug +++251104
		  //handle
			memcpy(&val, &inBuf[4], sizeof(float));
		  tempVal = val;
		  setHandle(val);
		  dHANDLE = val;   //debug +++251104
		  //accel
			setAccel(inBuf[8]);
		  dACCEL = inBuf[8];  //debug +++251104
			//gear
			setGear(inBuf[9]);
		  //backet
		  memcpy(&val, &inBuf[10], sizeof(float));
			setDump(val);
			break;
	}
	//check servo error  +++251120
	int i;
	for(i=0; i<P4CNT; i++){
		if(((u4RxBuf[(i*RETURN_PACKET_SIZE)+6]) & ERR_BIT_MSK) != 0){
			
			queAlarmReset(4, u4RxBuf[i*RETURN_PACKET_SIZE]);
			packetWait();
		}
		svStatus[u4RxBuf[i*RETURN_PACKET_SIZE]] = u4RxBuf[(i*RETURN_PACKET_SIZE)+6];
	}
	
}

