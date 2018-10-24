/*
 * FootSensor.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#include "FootSensor.h"

FootSensor::FootSensor(SPI_HandleTypeDef* _hspi) {
	hspi = _hspi;
}

FootSensor::~FootSensor() {

}

void FootSensor::getPressures(){

}

void FootSensor::setID(uint8_t _ID) {

}


void FootSensor::updateID(uint8_t _ID) {

}
