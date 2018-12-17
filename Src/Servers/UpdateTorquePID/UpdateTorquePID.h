/*
 * UpdateTorquePID.h
 *
 *  Created on: Nov 16, 2018
 *      Author: kbisland
 */

#ifndef SERVERS_UPDATETORQUEPID_H_
#define SERVERS_UPDATETORQUEPID_H_
#include "../../HIDAbstract/HIDAbstract.h"
#include "../../LegAPI/Leg.h"

#define ID 31
class UpdateTorquePID: public HIDAbstract {
public:
	UpdateTorquePID(Leg *leg):HIDAbstract(ID){
		UpdateLeg = leg;

	}
	void event(uint8_t * buffer);
private:
	Leg *UpdateLeg;
};

#endif /* SERVERS_UPDATETORQUEPID_H_ */
