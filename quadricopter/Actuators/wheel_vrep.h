//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#ifndef PIONEER_WHEEL_VREP_H_
#define PIONEER_WHEEL_VREP_H_

#include <Actuators/rotary_motor.h>
#include <Knowledge/knowledge.h>
#include "../Connection/connection.h"

class WheelVREP : public RotaryMotor
{
private:
	int handle;
	Connection &connection;
  int rotorNumber;
public:
	WheelVREP(int name_, Connection &connection_);

	void setSpeed(double speed);
	void setSpeeds(double speed1, double speed2, double speed3, double speed4);
};

#endif // PIONEER_WHEEL_VREP_H_
