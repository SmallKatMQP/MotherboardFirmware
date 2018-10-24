/*
 * HeadTail.h
 *
 *  Created on: Oct 19, 2018
 *      Author: kbisland
 */

#ifndef HEADTAILAPI_HEADTAIL_H_
#define HEADTAILAPI_HEADTAIL_H_
#define NUMMOTORS 2
#include "stm32h7xx_hal.h"
class HeadTail {
public:
	HeadTail(SPI_HandleTypeDef *_hspi, GPIO_TypeDef* GPIOx, uint16_t Pin);
	void SetPosPID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);
	void SetVelPID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);
	void SetTorquePID(uint8_t * motors, uint8_t * kp, uint8_t * ki, uint8_t * kd);
	void updateLeg(uint16_t * Motor1, uint16_t * Motor2);
	void autoSetID();


private:
	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
	uint16_t retData[NUMMOTORS*5] = {0};
	uint16_t TransData[NUMMOTORS*5] = {0};
};

#endif /* HEADTAILAPI_HEADTAIL_H_ */
