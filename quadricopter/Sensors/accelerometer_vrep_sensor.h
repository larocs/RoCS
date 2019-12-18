//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#ifndef PIONEER_ACCELEROMETER_SENSOR_H_
#define PIONEER_ACCELEROMETER_SENSOR_H_


#include <Sensors/sensor.h>
#include <Knowledge/robot_model.h>
#include <Util/orientation.h>
#include "../Connection/connection.h"

class AccelerometerVREPSensor : public Sensor<Orientation>
{
public:
	Connection &connection;
	int handle;

public:
	explicit AccelerometerVREPSensor(Connection &connection_);

	void connect();

	bool getData(Orientation &value) override;

};


#endif // PIONEER_ACCELERATION_SENSOR_H_
