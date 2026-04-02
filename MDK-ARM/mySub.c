/*
code for AZD-KRD stepper driver
HM-60252
ModbusRTU
*/
#include "main.h"

void u1TxEnable(void){
	GPIOA->BSRR |= GPIO_BSRR_BS_8;
}

void u1TxDisable(void){
	GPIOA->BRR |= GPIO_BSRR_BS_8;
}

void u4TxEnable(void){
	GPIOH->BSRR |= GPIO_BSRR_BS_9;
}

void u4TxDisable(void){
	GPIOH->BRR |= GPIO_BSRR_BS_9;
}

void u5TxEnable(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_9;
}

void u5TxDisable(void){
	GPIOE->BRR |= GPIO_BSRR_BS_9;
}

void ledOn(void){
	GPIOD->BSRR |= GPIO_BSRR_BS_0;
}
void ledOff(void){
	GPIOD->BRR |= GPIO_BSRR_BS_0;
}

void blink(uint32_t d){
	int i;
	for(i=0; i<d; i++){
		ledOn();
		wait100ms();
		ledOff();
		wait100ms();
	}
}


void u1TxLedOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_2;
}
void u1TxLedOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_2;
}

void u1RxLedOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_3;
}
void u1RxLedOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_3;
}


void u2TxLedOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_4;
}
void u2TxLedOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_4;
}

void u2RxLedOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_5;
}
void u2RxLedOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_5;
}

void pato1On(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_13;
}
void pato1Off(void){
	GPIOB->BRR |= GPIO_BSRR_BS_13;
}
void pato2On(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_12;
}
void pato2Off(void){
	GPIOB->BRR |= GPIO_BSRR_BS_12;
}
void pato3On(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_15;
}
void pato3Off(void){
	GPIOE->BRR |= GPIO_BSRR_BS_15;
}

void hornOn(void){
	GPIOE->BSRR |= GPIO_BSRR_BS_14;
}
void hornOff(void){
	GPIOE->BRR |= GPIO_BSRR_BS_14;
}

