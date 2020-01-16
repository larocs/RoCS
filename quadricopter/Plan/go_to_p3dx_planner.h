//
// Laboratory of Robotics and Cognitive Science
// Created by:  Alex Lucchesi de Oliveira
// Github:      https://github.com/larocs/RoCS
//

#ifndef PIONEER_GO_TO_ORIGIN_PLANNER_H
#define PIONEER_GO_TO_ORIGIN_PLANNER_H


#include <Plan/planner.h>
#include "../Analyze/pass_robotnik_position.h"
#include "../Analyze/pass_vrep_position.h"
#include "../Analyze/pass_vrep_orientation.h"
#include "../Analyze/pass_vrep_accelerometer.h"

class GoToP3DXPlanner : public Planner
{
private:
	PassVREPPosition &position_analyze;
	PassVREPOrientation &orientation_analyze;
	PassVREPAccelerometer &accelerometer_analyze;
	PassRobotnikPosition &p3dx_positon_analyze;

  int runType;
	int state;
	Position destination;

public:
	GoToP3DXPlanner(Knowledge &knowledge, PassVREPPosition &position_analyze, PassVREPOrientation &orientation_analyze,
      PassVREPAccelerometer &accelerometer_analyze, PassRobotnikPosition &p3dx_positon_analyze_, int runType);

	void planIteration() override;

  struct Lasts{
    float pAlphaE = 0;
    float pBetaE = 0;
    float pAlphaI = 0;
    float pBetaI = 0;

    float prevEuler=0;
    Position prevPos=Position();
  }lasts;
};


#endif //PIONEER_GO_TO_ORIGIN_PLANNER_H
