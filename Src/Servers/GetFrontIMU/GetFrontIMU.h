/*
 * GetIMUData.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_GETFRONTIMU_GETFRONTIMU_H_
#define SERVERS_GETFRONTIMU_GETFRONTIMU_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../IMUBreakoutAPI/IMUBreaout.h"
#define ID 32
class GetFrontIMU: public HIDAbstract{
public:
	GetFrontIMU(IMUBreaout *imu):HIDAbstract(ID){
		IMU = imu;
	}
	void event(uint8_t * buffer);
private:
	IMUBreaout * IMU;
};

#endif /* SERVERS_GETFRONTIMU_GETFRONTIMU_H_ */
