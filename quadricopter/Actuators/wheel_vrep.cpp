//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#include <include/v_repConst.h>

#include <utility>
#include <iostream>
#include <Knowledge/knowledge.h>

#include "wheel_vrep.h"

WheelVREP::WheelVREP(int name_, Connection &connection_)
	:RotaryMotor("motor"), rotorNumber(name_), handle(-1), connection(connection_)
{
  setSpeed(50.2);
}

void WheelVREP::setSpeed(double speed)
{
  float speedParam[]={(float)speed};
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"setRotorVelocity",1,&rotorNumber,1,speedParam,0,NULL,0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
}

void WheelVREP::setSpeeds(double speed1,double speed2,double speed3,double speed4)
{
  float speedParam[]={(float)speed1,(float)speed2,(float)speed3,(float)speed4};
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"setRotorVelocities",0,NULL,4,speedParam,0,NULL,0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
}
