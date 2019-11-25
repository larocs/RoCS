//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#ifndef PIONEER_PASS_VREP_ACCELERATION_H
#define PIONEER_PASS_VREP_ACCELERATION_H


#include <vector>
#include <Analyze/analyze.h>
#include <Util/orientation.h>

class PassVREPAccelerometer : public Analyze<std::vector<Orientation>, std::vector<Orientation>>
{
public:
	explicit PassVREPAccelerometer(Knowledge &knowledge_);

	std::vector<Orientation> mergeAndProcess(std::vector<Orientation> ov) override;
};


#endif //PIONEER_PASS_VREP_ACCELERATION_H
