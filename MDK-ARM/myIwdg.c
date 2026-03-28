#include "main.h"


//LSI 37kHz
void initIwdg(void){
	LL_RCC_LSI_Enable();
	while (LL_RCC_LSI_IsReady() != 1){}
	
	IWDG->KR = 0x0000CCCC;  //enable iwdt
	IWDG->KR = 0x00005555;  //enable setting
	IWDG->PR = 5;  //0:1/4 1:1/8 2:1/16 3:1/32 4:1/64 5:1/128 6:1/256 7:1/256 
//	IWDG->RLR = 0x00000FFF & (0x00000D8D);  //around 3Sec
//  IWDG->RLR = 0x00000FFF & (0x000006A4);  //around 1.5sec
	IWDG->RLR = 0x00000FFF & (0x00000241);  //around 0.5sec
	while(IWDG->SR){}	  //wait register update
	IWDG->KR = 0x0000AAAA;  //reset
}


void resetWDT(void){
	IWDG->KR = 0x0000AAAA;
}

