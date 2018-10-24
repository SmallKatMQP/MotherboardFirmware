/*
 * SmartServo.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: kbisland
 */

#include "SmartServo.h"

SmartServo::SmartServo(uint8_t ID, SPI_HandleTypeDef * _hspi)
{
	hspi = _hspi;
	id = ID;
}

SmartServo::~SmartServo() {
	// TODO Auto-generated destructor stub
}

void SmartServo::setPosition(uint16_t unsignedShortInt) {
}

void SmartServo::setVelocity(uint16_t unsignedShortInt) {
}

void SmartServo::settorque(uint16_t unsignedShortInt) {
}

uint16_t SmartServo::getPosition() {
	return 0;
}

float SmartServo::getVelocity() {
	return 0;
}

float SmartServo::getTorque() {
	return 0;
}

void SmartServo::SetmotorID(uint8_t unsignedChar) {
}

void SmartServo::SetPosPID(uint8_t unsignedChar, uint8_t unsignedChar1,
		uint8_t unsignedChar2) {
}

void SmartServo::SetVelPID(uint8_t unsignedChar, uint8_t unsignedChar1,
		uint8_t unsignedChar2) {
}

void SmartServo::SetPTorquePID(uint8_t unsignedChar, uint8_t unsignedChar1,
		uint8_t unsignedChar2) {
}

void SmartServo::updateID(uint8_t unsignedChar) {
}
