//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
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
  // setSpeed(0);

  /*
	int id = connection.getClientId();

	if (simxGetObjectHandle(id, (const simxChar *) name.c_str(), (simxInt *) &handle,
	                        (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
		std::cout << "Motor handle " << name << " not found! " << " Handle:" << handle << std::endl;
	else
	{
		std::cout << "Connected to motor " << name << " Handle:" << handle << std::endl;
		setSpeed(0);
	}
  //*/
}

float WheelVREP::getVelocity(){
  int *len; // If gives an error, malloc, but unused.
  float **matrix=(float**)calloc(1,sizeof(float*)); //=(float*)calloc(12,sizeof(float));
  //std::cout << matrix << std::endl;
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"getVelocity", 0, NULL, 0, NULL,0,NULL,0,NULL,NULL,NULL,len,matrix,NULL,NULL,NULL,NULL,simx_opmode_blocking);
  if(*matrix==0){
    std::cout << "velocity zero!" << std::endl;
    return 0;
  }
  return (*matrix)[2];
}

float* WheelVREP::getMatrix(){
  int *len; // If gives an error, malloc, but unused.
  float **matrix=(float**)calloc(1,sizeof(float*)); //=(float*)calloc(12,sizeof(float));
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"getQuadMatrix", 0, NULL, 0, NULL,0,NULL,0,NULL,NULL,NULL,len,matrix,NULL,NULL,NULL,NULL,simx_opmode_blocking);
  return *matrix;
}

void WheelVREP::setSpeed(double speed)
{
	// simxSetJointTargetVelocity(connection.getClientId(), handle, (simxFloat) speed, simx_opmode_streaming);
  float speedParam[]={(float)speed};
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"setRotorVelocity",1,&rotorNumber,1,speedParam,0,NULL,0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
}

void WheelVREP::setSpeeds(double speed1,double speed2,double speed3,double speed4)
{
	// simxSetJointTargetVelocity(connection.getClientId(), handle, (simxFloat) speed, simx_opmode_streaming);
  float speedParam[]={(float)speed1,(float)speed2,(float)speed3,(float)speed4};
  simxCallScriptFunction(connection.getClientId(),"Quadricopter",sim_scripttype_childscript,"setRotorVelocities",0,NULL,4,speedParam,0,NULL,0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
}
