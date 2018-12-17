/*
 * HeadTail.cpp
 *
 *  Created on: Oct 19, 2018
 *      Author: kbisland
 */

#include "HeadTail.h"

HeadTail::HeadTail(SPI_HandleTypeDef* _hspi, GPIO_TypeDef* GPIOx, uint16_t Pin) {
	hspi = _hspi;
	GPIO = GPIOx;
	GPIO_Pin = Pin;
}

void HeadTail::SetPosPID(uint8_t* motors, uint8_t* kp, uint8_t* ki,
		uint8_t* kd) {
	*TransData = {0};
	uint16_t CMDS[4] = {0x47};
	for(int i = 0;i<sizeof(motors)/sizeof(uint8_t);i++){
		TransData[10+i*5] = CMDS[i];
		TransData[10+i*5+1] = motors[i];
		TransData[10+i*5+2] = kp[i];
		TransData[10+i*5+3] = ki[i];
		TransData[10+i*5+4] = kd[i];

	}
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	//HAL_SPI_Transmit(hspi, TransData, 29, 290);
	GPIO->ODR |= GPIO_Pin; //set cs pin high

}

void HeadTail::SetVelPID(uint8_t* motors, uint8_t* kp, uint8_t* ki,
		uint8_t* kd) {
	*TransData = {0};
	uint16_t CMDS[4] = {0x48};
	for(int i = 0;i<sizeof(motors)/sizeof(uint8_t);i++){
		TransData[10+i*5] = CMDS[i];
		TransData[10+i*5+1] = motors[i];
		TransData[10+i*5+2] = kp[i];
		TransData[10+i*5+3] = ki[i];
		TransData[10+i*5+4] = kd[i];

	}
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	//HAL_SPI_Transmit(hspi, TransData, 29, 290);
	GPIO->ODR |= GPIO_Pin; //set cs pin high

}

void HeadTail::SetTorquePID(uint8_t* motors, uint8_t* kp, uint8_t* ki,
		uint8_t* kd) {
	*TransData = {0};
	uint16_t CMDS[4] = {0x49};
	for(int i = 0;i<sizeof(motors)/sizeof(uint8_t);i++){
		TransData[i*5] = CMDS[i];
		TransData[i*5+1] = motors[i];
		TransData[i*5+2] = kp[i];
		TransData[i*5+3] = ki[i];
		TransData[i*5+4] = kd[i];

	}
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	//HAL_SPI_Transmit(hspi, TransData, 29, 290);
	GPIO->ODR |= GPIO_Pin; //set cs pin high

}

void HeadTail::updateLeg(uint16_t* Motor1, uint16_t* Motor2) {
	*TransData = {0};
		for(int i = 0; i<5;i++){
			TransData[i] = Motor1[i];
			TransData[i+5] = Motor2[i];
		}
		GPIO->ODR &= ~GPIO_Pin; //set cs pin low
		//HAL_SPI_TransmitReceive(hspi, TransData, retData,  29, 290);//send setpints and get prev data
		GPIO->ODR |= GPIO_Pin; //set cs pin high
}

void HeadTail::autoSetID() {
}
