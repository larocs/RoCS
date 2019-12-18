//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#include "pass_vrep_accelerometer.h"

PassVREPAccelerometer::PassVREPAccelerometer(Knowledge &knowledge_):Analyze(knowledge_)
{
}

std::vector<Orientation> PassVREPAccelerometer::mergeAndProcess(std::vector<Orientation> ov)
{
	return ov;
}
