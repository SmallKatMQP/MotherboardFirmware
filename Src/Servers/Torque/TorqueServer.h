/*
 * TorqueServer.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_TORQUESERVER_H_
#define SERVERS_TORQUESERVER_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"

#define ID 29

class TorqueServer: public HIDAbstract {
public:
	TorqueServer(Leg *legs):HIDAbstract(ID){
		for(int i = 0;i<4;i++){
			*RobotLegs[i] = legs[i];
		}
	}
	void event(uint8_t * buffer);
private:
	Leg * RobotLegs[4];
};

#endif /* SERVERS_TORQUESERVER_H_ */
