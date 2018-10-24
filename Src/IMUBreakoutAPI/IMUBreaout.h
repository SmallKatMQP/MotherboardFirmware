/*
 * IMUBreaout.h
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#ifndef IMUBREAKOUTAPI_IMUBREAOUT_H_
#define IMUBREAKOUTAPI_IMUBREAOUT_H_
#include "stm32h7xx_hal.h"
#include"stdint.h"
class IMUBreaout {
public:
	IMUBreaout(SPI_HandleTypeDef *_hspi, GPIO_TypeDef* GPIOx, uint16_t Pin);
	void setID(uint8_t _ID);
	uint16_t* getAccelerometer();
	uint16_t* getGyroscope();
	uint16_t* getMagnetometer();

private:
	uint8_t ID = 0;
	//uint16_t TransData[5] = {0};
	uint16_t retData[5] = {0};
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef * GPIO;
	uint16_t GPIO_Pin;
};

#endif /* IMUBREAKOUTAPI_IMUBREAOUT_H_ */
