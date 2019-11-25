//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#ifndef PIONEER_ACCELERATION_VREP_MONITOR_H
#define PIONEER_ACCELERATION_VREP_MONITOR_H


#include <Util/orientation.h>
#include <Monitor/monitor.h>

class AccelerometerVREPMonitor : public Monitor<Orientation, Orientation>
{
public:
	explicit AccelerometerVREPMonitor(Knowledge &knowledge);

	AccelerometerVREPMonitor(Knowledge knowledge_, Sensor<Orientation> *orientation_sensor);

	Orientation interpret(Orientation raw) override;
};


#endif //PIONEER_ACCELERATION_VREP_MONITOR_H
