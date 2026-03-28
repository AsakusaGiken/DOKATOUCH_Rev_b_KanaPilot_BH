/*
code for wait 

at 32MHz

*/
#include "main.h"




void wait10us(void){
	int32_t dummy=0;
	int32_t cnt = 42;
	while(cnt>0){
		dummy++;
		cnt--;
	}
}

void wait100us(void){
	int32_t dummy=0;
	int32_t cnt = 525;
	while(cnt>0){
		dummy++;
		cnt--;
	}
}

void wait1ms(void){
	int32_t cnt = 4635;
	while(cnt>0){
		cnt--;
	}	
}

void wait_ms(uint32_t count){
	uint32_t j;
	for(j=0; j<count; j++){
		wait1ms();
	}
}

void wait10ms(void){
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
	wait1ms();wait1ms();wait1ms();wait1ms();wait1ms();
}

void wait100ms(void){
	wait10ms();wait10ms();wait10ms();wait10ms();wait10ms();
	wait10ms();wait10ms();wait10ms();wait10ms();wait10ms();
}

void wait1s(void){
	wait100ms();wait100ms();wait100ms();wait100ms();wait100ms();
	wait100ms();wait100ms();wait100ms();wait100ms();wait100ms();
}
