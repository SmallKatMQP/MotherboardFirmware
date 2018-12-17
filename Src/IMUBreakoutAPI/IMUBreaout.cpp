/*
 * IMUBreaout.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#include "IMUBreaout.h"
#include "stdint.h"
IMUBreaout::IMUBreaout(SPI_HandleTypeDef* _hspi, GPIO_TypeDef* GPIOx,
		uint16_t Pin) {
	GPIO = GPIOx;
	GPIO_Pin = Pin;
	hspi = _hspi;

}

void IMUBreaout::setID(uint8_t _ID) {
	uint16_t TransData[] = {0x22,ID,0x00,0x01,0x00};
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low
	HAL_SPI_Transmit(hspi, (uint8_t*)TransData, 5, 50);
	GPIO->ODR |= GPIO_Pin;
}

uint16_t* IMUBreaout::getAccelerometer() {
	uint16_t TransData[] = {0x37,ID,0x00,0x00,0x00};
	uint16_t AccelerometerData[3];
	*retData = {0};
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low

	HAL_SPI_Transmit(hspi, (uint8_t*)TransData, 5, 50);
	HAL_Delay(2);
	HAL_SPI_TransmitReceive(hspi, (uint8_t*)0x00, (uint8_t*)retData, 5, 50);
	GPIO->ODR |= GPIO_Pin;
	for(int i = 0;i<3;i++){
		AccelerometerData[i] = retData[2+i];
	}
	return AccelerometerData;
}

uint16_t* IMUBreaout::getGyroscope() {
	uint16_t TransData[] = {0x38,ID,0x00,0x00,0x00};
	uint16_t GyroData[3];
	*retData = {0};
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low

	HAL_SPI_Transmit(hspi, (uint8_t*)TransData, 5, 50);
	HAL_Delay(2);
	//HAL_SPI_TransmitReceive(hspi, (uint8_t*)0x00, retData, 5, 50);
	GPIO->ODR |= GPIO_Pin;
	for(int i = 0;i<3;i++){
		GyroData[i] = retData[2+i];
	}
	return GyroData;
}

uint16_t* IMUBreaout::getMagnetometer() {
	uint16_t TransData[] = {0x39,ID,0x00,0x00,0x00};
	uint16_t MagData[3];
	*retData = {0};
	GPIO->ODR &= ~GPIO_Pin; //set cs pin low

	HAL_SPI_Transmit(hspi, (uint8_t*)TransData, 5, 50);
	HAL_Delay(2);
	//HAL_SPI_TransmitReceive(hspi, (uint8_t*)0x00, retData, 5, 50);
	GPIO->ODR |= GPIO_Pin;
	for(int i = 0;i<3;i++){
		MagData[i] = retData[2+i];
	}
	return MagData;
}
