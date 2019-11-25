//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#include "pass_vrep_accelerometer.h"

PassVREPAccelerometer::PassVREPAccelerometer(Knowledge &knowledge_):Analyze(knowledge_)
{
}

std::vector<Orientation> PassVREPAccelerometer::mergeAndProcess(std::vector<Orientation> ov)
{
	return ov;
}
