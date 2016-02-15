#pragma once
#include <string>
#include "constants.h"
#include "WPILib.h"

#ifndef SRC_INTERFACES_H_
#define SRC_INTERFACES_H_

class SolenoidInterface
{
public:
	virtual void Fwd() = 0;
	virtual void Rev() = 0;
};

class Joystick;
class Solenoid;
struct TESTER_PARAMETERS
{
	TESTER_PARAMETERS(  Joystick* stick,
						SolenoidInterface* shiftsolenoid)
						: stick(stick),
						shiftsolenoid(shiftsolenoid){}
	Joystick* stick;
	SolenoidInterface* shiftsolenoid;

};




#endif /* SRC_INTERFACES_H_ */
