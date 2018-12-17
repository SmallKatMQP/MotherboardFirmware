/*
 * VelocityServer.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_VELOCITYSERVER_H_
#define SERVERS_VELOCITYSERVER_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"

#define ID 33
class VelocityServer: public HIDAbstract{
public:
	VelocityServer(Leg *legs):HIDAbstract(ID){
		for(int i = 0;i<4;i++){
			*RobotLegs[i] = legs[i];
		}
	}
	void event(uint8_t * buffer);
private:
	Leg * RobotLegs[4];
};

#endif /* SERVERS_VELOCITYSERVER_H_ */
