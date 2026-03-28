/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2024 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_crc.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1 */

/* USER CODE BEGIN Private defines */
void wait10us(void);
void wait100us(void);
void wait1ms(void);
void wait10ms(void);
void wait100ms(void);
void wait1s(void);
void wait10s(void);
void ledOn(void);
void ledOff(void);
void blink(uint32_t);
//void initAdc(void);
//uint16_t readAdc(uint32_t);
void pato1On(void);
void pato1Off(void);
void pato2On(void);
void pato2Off(void);
void pato3On(void);
void pato3Off(void);
void u2TxLedOn(void);
void u2TxLedOff(void);
	
//uart
void sendUart1Byte(uint8_t);
void sendUart2Byte(uint8_t);
void sendUart5Byte(uint8_t);
void sendUart1Packet(uint8_t *, uint16_t);
void sendUart2Packet(uint8_t *, uint16_t);
void sendUart5Packet(uint8_t *, uint16_t);
//void U5RX_Callback(void);
//void u5RxInterruptEnable(void);
void u2RxInterruptEnable(void);
void u2RxInterruptDisable(void);
void u4RxInterruptEnable(void);
void u4RxInterruptDisable(void);
void u1RxInterruptEnable(void);
void u1RxInterruptDisable(void);
void sendUart4Packet(uint8_t *, uint16_t);
void U1RX_Callback(void);

void rs485TxEnable(void);
void rs485TxDisable(void);
void U4RX_Callback(void);
void U5RX_Callback(void);
void U2RX_Callback(void);
void u2RxLedOn(void);
void u2RxLedOff(void);
void ledOn(void);
void ledOff(void);
void blink(uint32_t);
void resetWDT(void);

//RS485
void u1TxEnable(void);
void u1TxDisable(void);
void u4TxEnable(void);
void u4TxDisable(void);
void u5TxEnable(void);
void u5TxDisable(void);

//modbus
uint16_t CRC16 (uint8_t*, uint16_t);
void queReadRegister(uint8_t, uint8_t, uint16_t, uint16_t);
void queWriteRegister(uint8_t, uint8_t, uint16_t, int16_t);
void queWriteRegisterSeq(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t*);
void queIoStatusRequest(uint8_t, uint8_t);
void queAlarmReset(uint8_t, uint8_t);
void queZhomeRequest(uint8_t, uint8_t);
void sendQuery(uint8_t, uint8_t *, uint8_t);
void setPositionByDefaultSetting(uint8_t, uint8_t, uint32_t);
void setPosAndReadStatusByDefaultSetting(uint8_t, uint8_t, uint32_t);
void queFreeRequest(uint8_t, uint8_t);
	
void modbusWrite(uint8_t, uint16_t, int16_t);
void modbusRead(uint8_t, uint16_t, int16_t);

void WT901_calibrateAcceleration(void);
void WT901_queReadData(void);
void sendSensorData(void);
void sensorQue(void);
void allServoDrive(uint8_t*);
	
//void initCrc(void);

void backhoe_1_main(void);
void stopAllControlAndResetStatus(void);
void backhoeInit(void);
void allFreeOn(void);
void allFreeOff(void);

void initIwdg(void);
void resetWDT(void);

void emgSubRelayOn(void);
void emgSubRelayOff(void);

bool isTiltStop(void);  //2022.11.10
bool isTiltSafe(void);  //2022.11.10

void wa30_main(void);
void wa30Init(void);
void initTimers(void);
void T2_Callback(void);
void checkAcmIncomming(void);

void cat725GamepadDrive(uint8_t *, uint32_t);
void cat725DirectServoDrive(uint8_t*);
void setPosAndReadStatusByDefaultSetting3(uint8_t, uint8_t, uint32_t);
void u1TxLedOn(void);
void u1TxLedOff(void);
void u1RxLedOn(void);
void u1RxLedOff(void);
void sensorsRead(void);
void setContinuousSpeed(uint8_t, uint8_t, uint32_t);
void continuousDrive(uint8_t, uint8_t, int32_t);
void setRelativePosAndReadStatus(uint8_t, uint8_t, int32_t, uint32_t, uint32_t);

#define REG_TORQUE 0x00D6
#define REG_IO_INPUT 0x007D
#define REG_IO_STATUS 0x007E
#define REG_ALM_RESET 0x0180

#define CONT_SPEED 20000

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
