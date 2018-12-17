/*
 * UpdatePositionPID.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_UPDATEPOSITIONPID_H_
#define SERVERS_UPDATEPOSITIONPID_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"
#define ID 30
class UpdatePositionPID: public HIDAbstract{
public:
	UpdatePositionPID(Leg *leg):HIDAbstract(ID){
		UpdateLeg = leg;
	}
	void event(uint8_t * buffer);
private:
	Leg *UpdateLeg;
};

#endif /* SERVERS_UPDATEPOSITIONPID_H_ */
