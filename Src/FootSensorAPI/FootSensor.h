/*
 * FootSensor.h
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#ifndef FOOTSENSORAPI_FOOTSENSOR_H_
#define FOOTSENSORAPI_FOOTSENSOR_H_
#include<stdint.h>
#include "stm32h7xx_hal.h"
class FootSensor {
public:
	FootSensor(SPI_HandleTypeDef *_hspi);
	virtual ~FootSensor();
	void getPressures();
	void setID(uint8_t);
	void updateID(uint8_t);

private:
	uint32_t Pressures[7] = {0};
	uint8_t ID = 0;
	SPI_HandleTypeDef * hspi;

};

#endif /* FOOTSENSORAPI_FOOTSENSOR_H_ */
