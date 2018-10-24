/*
 * Leg.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#include "Leg.h"
#include "stdint.h"
Leg::Leg(SPI_HandleTypeDef *_hspi, GPIO_TypeDef* GPIOx, uint16_t Pin){
	hspi = _hspi; //SPI instance
	GPIO = GPIOx; //CS pin port
	GPIO_Pin = Pin; //CS pin
}

void Leg::AutoAssignID() {
	TransData[0] = 0x87;//set id command
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	HAL_SPI_Transmit(hspi, TransData, 29, 290);
	GPIO->ODR |= GPIO_Pin; //set cs pin high
}

void Leg::getMotorData() {
	for(int i = 0;i<5;i++){
		MotorData[0][i] = retData[i];
		MotorData[1][i] = retData[i+5];
		MotorData[2][i] = retData[i+10];
		MotorData[3][i] = retData[i+15];
	}
}

void Leg::getFootPressureData() {
	for(int i = 0;i<9;i++){
		Pressures[i] = retData[21+i];
	}
}

void Leg::updateLeg(uint16_t* Motor1, uint16_t* Motor2, uint16_t* Motor3,
		uint16_t* Motor4, uint16_t* FootSensor) {
	*TransData = {0};
	for(int i = 0;i<9;i++){
		TransData[i] = FootSensor[i];
	}
	for(int i = 0; i<5;i++){
		TransData[10+i] = Motor1[i];
		TransData[10+i+5] = Motor2[i];
		TransData[10+i+10] = Motor3[i];
		TransData[10+i+15] = Motor4[i];
	}
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	HAL_SPI_TransmitReceive(hspi, TransData, retData,  29, 290);//send setpints and get prev data
	GPIO->ODR |= GPIO_Pin; //set cs pin high
}

void Leg::SetPosPID(uint8_t* motors, uint8_t* kp, uint8_t* ki, uint8_t* kd) {
	*TransData = {0};
	uint16_t CMDS[4] = {0x47};
	for(int i = 0;i<sizeof(motors);i++){
		TransData[10+i*5] = CMDS[i];
		TransData[10+i*5+1] = motors[i];
		TransData[10+i*5+2] = kp[i];
		TransData[10+i*5+3] = ki[i];
		TransData[10+i*5+4] = kd[i];

	}
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
		HAL_SPI_Transmit(hspi, TransData, 29, 290);
		GPIO->ODR |= GPIO_Pin; //set cs pin high

}

void Leg::SetVelPID(uint8_t* motors, uint8_t* kp, uint8_t* ki, uint8_t* kd) {
	*TransData = {0};
		uint16_t CMDS[4] = {0x48};
		for(int i = 0;i<sizeof(motors);i++){
			TransData[10+i*5] = CMDS[i];
			TransData[10+i*5+1] = motors[i];
			TransData[10+i*5+2] = kp[i];
			TransData[10+i*5+3] = ki[i];
			TransData[10+i*5+4] = kd[i];

		}
		GPIO->ODR &= ~GPIO_Pin; //set cs pin low
			HAL_SPI_Transmit(hspi, TransData, 29, 290);
			GPIO->ODR |= GPIO_Pin; //set cs pin high

}

void Leg::SetTorquePID(uint8_t* motors, uint8_t* kp, uint8_t* ki, uint8_t* kd) {
	*TransData = {0};
		uint16_t CMDS[4] = {0x49};
		for(int i = 0;i<sizeof(motors);i++){
			TransData[10+i*5] = CMDS[i];
			TransData[10+i*5+1] = motors[i];
			TransData[10+i*5+2] = kp[i];
			TransData[10+i*5+3] = ki[i];
			TransData[10+i*5+4] = kd[i];

		}
		GPIO->ODR &= ~GPIO_Pin; //set cs pin low
			HAL_SPI_Transmit(hspi, TransData, 29, 290);
			GPIO->ODR |= GPIO_Pin; //set cs pin high

}
