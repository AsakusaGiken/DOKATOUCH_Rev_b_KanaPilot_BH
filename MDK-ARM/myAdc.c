/*
code for ADC
*/
#include "main.h"

void initAdc(void){
	
	ADC1->CFGR1 &= 0x833E0200;  //clrar CFGR1 (clear without reserved bit)
	ADC1->CFGR2 &= 0x3FFFFC02;  //clrar CFGR2 (clear without reserved bit)
//	ADC1->SMPR = 0x05;  //sampring time 0:1.5 1:3.5 2:7.5 3:12.5 4:19.5 5:39.5 6:79.5 7:160.5 clock cycles
  ADC1->SMPR = 0x07;  //sampring time 0:1.5 1:3.5 2:7.5 3:12.5 4:19.5 5:39.5 6:79.5 7:160.5 clock cycles
	ADC1->CR &= 0x9FFFFFE7;  //clrar CR2 (clear without reserved bit)
	ADC1->CR |= ADC_CR_ADCAL;  //calibration start
	while(ADC1->CR & ADC_CR_ADCAL);  //wait calibration done
//	while(!(ADC1->ISR & ADC_ISR_EOCAL));  //wait calibration done
	wait10us();  //***
	ADC1->CR |= ADC_CR_ADEN;  //ADC enable
	wait10us();
}


uint16_t readAdc(uint32_t ch){
	uint32_t chMsk = 0x00000001;
	chMsk = chMsk << ch;
	ADC1->CHSELR = chMsk;
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOSEQ));   //wait end of sequence
	ADC1->ISR |= ADC_ISR_EOSEQ; //crear flg
	return ADC1->DR;
}

