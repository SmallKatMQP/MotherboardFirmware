/*
 * Leg.h
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#ifndef LEGAPI_LEG_H_
#define LEGAPI_LEG_H_

#include "../SmartServoAPI/SmartServo.h"
#include "../FootSensorAPI/FootSensor.h"
#include "stdint.h"
class Leg {
public:
	Leg(SPI_HandleTypeDef *_hspi, GPIO_TypeDef* GPIOx, uint16_t Pin);
	void AutoAssignID();
	void getMotorData();
	void getFootPressureData();
	void getLegData();
	void updateLeg(uint16_t * Motor1, uint16_t * Motor2, uint16_t * Motor3, uint16_t * Motor4, uint16_t * FootSensor );
	void SetPosPID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);
	void SetVelPID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);
	void SetTorquePID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);



private:
	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
	uint16_t* Position = 0;
	uint16_t* Velocity = 0;
	uint16_t* Torque = 0;
	uint16_t** MotorData = 0;
	uint16_t* Pressures = 0;
	uint16_t retData[29] = {0}; //order: M1 4 bytes, M2 4 bytes, M3 4 bytes, M4 4 bytes F7 9 bytes
	uint16_t TransData[29] = {0}; //order: FS 7bytes, M1 4 bytes, M2 4 bytes, M3 4 bytes, M4 4 bytes

};

#endif /* LEGAPI_LEG_H_ */
