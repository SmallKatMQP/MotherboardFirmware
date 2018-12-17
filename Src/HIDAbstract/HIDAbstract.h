/*
 * HIDAbstract.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef HIDABSTRACT_HIDABSTRACT_H_
#define HIDABSTRACT_HIDABSTRACT_H_
#include "stm32h7xx_hal.h"
#include "main.h"
class HIDAbstract {
public:
	HIDAbstract(uint8_t id);
	virtual void event(uint8_t *buffer)=0;
	uint8_t getId(){
		return ID;
	}
private:
uint8_t ID;
};

#endif /* HIDABSTRACT_HIDABSTRACT_H_ */
