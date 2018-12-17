/*
 * GetFootSensorData.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_GETFOOTSENSORDATA_GETFOOTSENSORDATA_H_
#define SERVERS_GETFOOTSENSORDATA_GETFOOTSENSORDATA_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"
#define ID 33
class GetFootSensorData: public HIDAbstract {
public:
	GetFootSensorData(Leg * legs):HIDAbstract(ID){
		for(int i = 0;i<4;i++){
			*FootSensors[i] = legs[i];
		}
	}
	void event(uint8_t * buffer);
private:
	Leg *  FootSensors[4];
};

#endif /* SERVERS_GETFOOTSENSORDATA_GETFOOTSENSORDATA_H_ */
