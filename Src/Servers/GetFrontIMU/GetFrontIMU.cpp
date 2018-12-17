/*
 * GetIMUData.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#include "GetFrontIMU.h"

void GetFrontIMU::event(uint8_t * buffer){
	//float accel[3] = IMU->getAccelerometer();
	IMU->getGyroscope();
	IMU->getMagnetometer();


}
