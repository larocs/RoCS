//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#include <include/v_repConst.h>
#include <remoteApi/extApi.h>
#include <iostream>
#include <thread>
#include "accelerometer_vrep_sensor.h"
#include "../Knowledge/robotnik_model.h"

AccelerometerVREPSensor::AccelerometerVREPSensor(Connection &connection_)
	:Sensor("AccelerometerSensor"), connection(connection_), handle(connection_.getRobotHandle())
{
}

void AccelerometerVREPSensor::connect()
{
  simxGetObjectHandle(connection.getClientId(), "Accelerometer_forceSensor", (simxInt *) &handle,
	                        (simxInt) simx_opmode_oneshot_wait);
	float angle[3] = {0, 0, 0};
  
  simxReadForceSensor(connection.getClientId(), handle, NULL, angle, NULL, simx_opmode_streaming);
  while(simxReadForceSensor(connection.getClientId(), handle, NULL, angle, NULL, simx_opmode_buffer))
	{
//		std::cout << "Waiting to connect to " << name << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(15));
	}
	std::cout << "Connected to Accelerometer\n";
}

bool AccelerometerVREPSensor::getData(Orientation &value)
{
	float angle[3];
  angle[1]=0;
  // std::cout << "handler " << handle << "\n";
	
  if (simx_return_ok == simxReadForceSensor(connection.getClientId(), handle, NULL, angle, NULL, simx_opmode_buffer))
	{
	  // std::cout << "GetDataAccelerometer: " << angle[0] << " " <<  angle[1]<< " " <<  angle[2] <<std::endl;
    
		value.setOrientation(angle[0], angle[1], angle[2]);
		return true;
	}
	value.setValid(false);
	return false;
}


