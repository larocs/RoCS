//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#include "accelerometer_vrep_monitor.h"

AccelerometerVREPMonitor::AccelerometerVREPMonitor(Knowledge &knowledge)
	:Monitor(knowledge)
{
}


AccelerometerVREPMonitor::AccelerometerVREPMonitor(Knowledge knowledge_, Sensor<Orientation> *acceleration_sensor)
	:Monitor(knowledge_, acceleration_sensor)
{
}

Orientation AccelerometerVREPMonitor::interpret(Orientation raw)
{
//	std::cout << raw << "\n";
	return raw;
}
