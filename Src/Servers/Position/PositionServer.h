/*
 * PositionServer.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_POSITIONSERVER_H_
#define SERVERS_POSITIONSERVER_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"

#define ID 28
class PositionServer: public HIDAbstract{
private:
public:
	PositionServer(Leg *legs):HIDAbstract(ID){
		for(int i = 0;i<4;i++){
			*RobotLegs[i] = legs[i];
		}
	}
	void event(uint8_t * buffer);
private:
	Leg * RobotLegs[4];
};

#endif /* SERVERS_POSITIONSERVER_H_ */
