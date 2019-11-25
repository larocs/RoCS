//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
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
