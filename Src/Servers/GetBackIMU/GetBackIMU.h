/*
 * GetIMUData.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_GETFRONTIMU_GETIMUDATA_H_
#define SERVERS_GETFRONTIMU_GETIMUDATA_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../IMUBreakoutAPI/IMUBreaout.h"
#define ID 32
class GetBackIMU: public HIDAbstract{
public:
	GetBackIMU(IMUBreaout *imu):HIDAbstract(ID){
		IMU = imu;
	}
	void event(uint8_t * buffer);
private:
	IMUBreaout * IMU;
};

#endif /* SERVERS_GETFRONTIMU_GETIMUDATA_H_ */
