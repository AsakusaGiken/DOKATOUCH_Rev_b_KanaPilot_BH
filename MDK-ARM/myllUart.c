/*
code for custom LL uart

*/
#include "main.h"

//***********************************	
//UART1 functions
//***********************************	
//send 1 byte
void sendUart1Byte(uint8_t data){
	while (!LL_USART_IsActiveFlag_TXE(USART1));
	LL_USART_TransmitData8(USART1, data);
	while (!LL_USART_IsActiveFlag_TC(USART1));
}
//send one packet
void sendUart1Packet(uint8_t *pData, uint16_t length){
	uint16_t lenCnt = length;
	uint16_t lenPos = 0;
	while(lenCnt>0){
		sendUart1Byte(pData[lenPos]);
		lenPos++;
		lenCnt--;
	}
}

void u1RxInterruptEnable(void){
		USART1->CR1 |= USART_CR1_RXNEIE; //UART1 RX interrupt enable
}

void u1RxInterruptDisable(void){
		USART1->CR1 &= ~USART_CR1_RXNEIE; //UART1 RX interrupt enable
}


//***********************************	
//UART2 functions
//***********************************	
//send 1 byte
void sendUart2Byte(uint8_t data){
	while (!LL_USART_IsActiveFlag_TXE(USART2));
	LL_USART_TransmitData8(USART2, data);
	while (!LL_USART_IsActiveFlag_TC(USART2));
}
//send one packet
void sendUart2Packet(uint8_t *pData, uint16_t length){
	uint16_t lenCnt = length;
	uint16_t lenPos = 0;
	while(lenCnt>0){
		sendUart2Byte(pData[lenPos]);
		lenPos++;
		lenCnt--;
	}
}


void u2RxInterruptEnable(void){
		USART2->CR1 |= USART_CR1_RXNEIE; //UART1 RX interrupt enable
}

void u2RxInterruptDisable(void){
		USART2->CR1 &= ~USART_CR1_RXNEIE; //UART1 RX interrupt enable
}



//***********************************	
//UART5 functions
//***********************************	
//send 1 byte
void sendUart5Byte(uint8_t data){
	while (!LL_USART_IsActiveFlag_TXE(USART5));
	LL_USART_TransmitData8(USART5, data);
	while (!LL_USART_IsActiveFlag_TC(USART5));
}
//send one packet
void sendUart5Packet(uint8_t *pData, uint16_t length){
	uint16_t lenCnt = length;
	uint16_t lenPos = 0;
	while(lenCnt>0){
		sendUart5Byte(pData[lenPos]);
		lenPos++;
		lenCnt--;
	}
}


void u5RxInterruptEnable(void){
		USART5->CR1 |= USART_CR1_RXNEIE; //UART1 RX interrupt enable
}

void u5RxInterruptDisable(void){
		USART5->CR1 &= ~USART_CR1_RXNEIE; //UART1 RX interrupt enable
}


//***********************************	
//UART4 functions
//***********************************	
//send 1 byte
void sendUart4Byte(uint8_t data){
	while (!LL_USART_IsActiveFlag_TXE(USART4));
	LL_USART_TransmitData8(USART4, data);
	while (!LL_USART_IsActiveFlag_TC(USART4));
}
//send one packet
void sendUart4Packet(uint8_t *pData, uint16_t length){
	uint16_t lenCnt = length;
	uint16_t lenPos = 0;
	while(lenCnt>0){
		sendUart4Byte(pData[lenPos]);
		lenPos++;
		lenCnt--;
	}
}

void u4RxInterruptEnable(void){
		USART4->CR1 |= USART_CR1_RXNEIE; //UART1 RX interrupt enable
}

void u4RxInterruptDisable(void){
		USART4->CR1 &= ~USART_CR1_RXNEIE; //UART1 RX interrupt enable
}

