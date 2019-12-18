//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#include "robotnik.h"
#include "../Knowledge/robotnik_knowledge.h"

extern "C"
{
#include <include/v_repConst.h>
}

Robotnik::Robotnik(int runType)
	:Robot("Quadricopter"), connection(knowledge.getConnection()), /*range_sensors(),*/ orientation_sensor(connection),
   acceleration_sensor(connection), position_sensor(connection), p3dx_position_sensor(connection), wheels(),
	 wheel_ptrs(), /*range_monitor(knowledge),*/ position_monitor(knowledge), orientation_monitor(knowledge),
   accelerometer_monitor(knowledge), p3dx_position_monitor(knowledge), /*range_analyze(knowledge),*/ position_analyze(knowledge),
	 orientation_analyze(knowledge), accelerometer_analyze(knowledge), p3dx_position_analyze(knowledge),
   planner(knowledge, position_analyze, orientation_analyze, accelerometer_analyze, p3dx_position_analyze, runType), file_visualizer{knowledge}, lifetime(300)
{
	verifyConnection();

	int robot_handle = -1;
	if (simxGetObjectHandle(connection.getClientId(), (const simxChar *) name.c_str(), (simxInt *) &robot_handle,
	                        (simxInt) simx_opmode_oneshot_wait) == simx_return_ok)
	{
		std::cout << "Connected to robot: " << name << " handle: " << robot_handle << std::endl;
		connection.setRobotHandle(robot_handle);
		setSensors();
		setActuators();

		setMonitors();
		setAnalyzes();
		setPlan();
		setExecute();

		setKnowledge();

		setVisualizers();
	}
	else
	{
		std::cout << "Could not connect to robot: " << name << std::endl;
		exit(1);
	}
}


void Robotnik::run()
{
	// TODO range_monitor.startThread();
	position_monitor.startThread();
	orientation_monitor.startThread();
	accelerometer_monitor.startThread();
	p3dx_position_monitor.startThread();

	// TODO range_analyze.startThread();
	position_analyze.startThread();
	orientation_analyze.startThread();
	accelerometer_analyze.startThread();
	p3dx_position_analyze.startThread();

	execute.startThread();

	knowledge.getAvoidWallModel().startThread();

	planner.startThread();

	file_visualizer.startThread();

	std::cout << "Threads running...\n";

	std::this_thread::sleep_for(lifetime);

}

void Robotnik::verifyConnection()
{
	if (connection.isConnected())
	{
		std::cout << "Connected to client: " << connection.getClientId() << std::endl;
	}
	else
	{
		std::cout << "Could not connect to client" << std::endl;
		exit(1);
	}
}



void Robotnik::connectToProximitySensors()
{
	for (int num_sonars = 0; num_sonars <= 2; ++num_sonars)
	{
		std::string sonar_name = "prox_sensor" + std::to_string(num_sonars);
		range_sensors.emplace_back(sonar_name, connection);

	}
}


void Robotnik::connectToOrientationSensor()
{
	orientation_sensor.connect(connection.getRobotHandle());
	Orientation robot_orientation;
	orientation_sensor.getData(robot_orientation);
//	std::cout << "First Ori: " << robot_orientation << std::endl;
}

void Robotnik::connectToAccelerometerSensor()
{
	acceleration_sensor.connect();
	Orientation robot_acceleration;
	acceleration_sensor.getData(robot_acceleration);
//	std::cout << "First Ori: " << robot_orientation << std::endl;
}

void Robotnik::connectToPositionSensor()
{
	position_sensor.connect(connection.getRobotHandle());
	Position robot_position;
	position_sensor.getData(robot_position);
//	std::cout << "First Pos: " << robot_position << std::endl;
}

void Robotnik::connectToP3DXPositionSensor()
{
	std::string robot_to_follow_name = "Pioneer_p3dx";
//	std::string robot_to_follow_name = "broder";
	int robot_to_follow_handle = -1;
	if (simxGetObjectHandle(connection.getClientId(), (const simxChar *) robot_to_follow_name.c_str(), (simxInt *) &robot_to_follow_handle,
							(simxInt) simx_opmode_oneshot_wait) == simx_return_ok) {
		p3dx_position_sensor.connect(robot_to_follow_handle);
		Position robot_position;
		p3dx_position_sensor.getData(robot_position);
		std::cout << "First Pos: " << robot_position << std::endl;
	}
}

void Robotnik::connectToWheels()
{
	int front_left_wheel_name = 1;
	int front_right_wheel_name = 4;
	int back_right_wheel_name = 3;
	int back_left_wheel_name = 2;

	wheels.emplace_back(front_left_wheel_name, connection);
	wheels.emplace_back(back_left_wheel_name, connection);
	wheels.emplace_back(back_right_wheel_name, connection);
	wheels.emplace_back(front_right_wheel_name, connection);

}

void Robotnik::setSensors()
{
	// TODO connectToProximitySensors();
	connectToOrientationSensor();
  connectToAccelerometerSensor();
	connectToP3DXPositionSensor();
	connectToPositionSensor();
}


void Robotnik::setActuators()
{
	connectToWheels();
	wheel_ptrs.push_back(&wheels[0]);
	wheel_ptrs.push_back(&wheels[1]);
	wheel_ptrs.push_back(&wheels[2]);
	wheel_ptrs.push_back(&wheels[3]);
	actuators.push_back(wheel_ptrs);

}

void Robotnik::setMonitors()
{
  /*
	for (RangeVREPSensor &s : range_sensors)
	{
		range_monitor.insertSensor(&s);
	}
  //*/
	position_monitor.insertSensor(&position_sensor);
	orientation_monitor.insertSensor(&orientation_sensor);
	accelerometer_monitor.insertSensor(&acceleration_sensor);
	p3dx_position_monitor.insertSensor(&p3dx_position_sensor);
}

void Robotnik::setAnalyzes()
{
	// TODO range_monitor.attach(&range_analyze);
	position_monitor.attach(&position_analyze);
	orientation_monitor.attach(&orientation_analyze);
  accelerometer_monitor.attach(&accelerometer_analyze);
	p3dx_position_monitor.attach(&p3dx_position_analyze);

}

void Robotnik::setVisualizers()
{
	orientation_monitor.attach( &(knowledge.getRobotModel()));
	accelerometer_monitor.attach( &(knowledge.getRobotModel()));
	position_monitor.attach( &(knowledge.getRobotModel()));
//	file_visualizer.setPipeline(&pipeline);
}

void Robotnik::setPlan()
{
	planner.setPipeline(&execute.getPipeline());
}

void Robotnik::setExecute()
{
	execute.setActuators(actuators);
}

void Robotnik::setKnowledge()
{
	setReactiveModels();
	knowledge.getAvoidWallModel().setPipeline(&execute.getPipeline());
	knowledge.setPipeline(&execute.getPipeline());
}

void Robotnik::setReactiveModels()
{
	// range_monitor.attach(&knowledge.getAvoidWallModel());
}
