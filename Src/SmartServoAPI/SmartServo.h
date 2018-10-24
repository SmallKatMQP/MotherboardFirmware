/*
 * SmartServo.h
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#ifndef SMARTSERVOAPI_SMARTSERVO_H_
#define SMARTSERVOAPI_SMARTSERVO_H_
#include "stm32h7xx_hal.h"
class SmartServo {
public:
	SmartServo(uint8_t ID, SPI_HandleTypeDef * _hspi);
	virtual ~SmartServo();
	void setPosition(uint16_t);
	void setVelocity(uint16_t);
	void settorque(uint16_t);
	uint16_t getPosition();
	float getVelocity();
	float getTorque();
	void SetmotorID(uint8_t);
	void SetPosPID(uint8_t, uint8_t, uint8_t);
	void SetVelPID(uint8_t, uint8_t, uint8_t);
	void SetPTorquePID(uint8_t, uint8_t, uint8_t);
	void updateID(uint8_t);




private:
	uint8_t id;
	SPI_HandleTypeDef * hspi;
};

#endif /* SMARTSERVOAPI_SMARTSERVO_H_ */
