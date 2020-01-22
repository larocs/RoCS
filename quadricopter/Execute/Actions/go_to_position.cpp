//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#include <cmath>

#include "go_to_position.h"
#include "set_wheels_speed.h"

GoToPosition::GoToPosition(const std::string &name_, double value_, Pipeline *pipeline_,
                           Position position_, Orientation orientation_, Position destination_,
                           GoToP3DXPlanner::Lasts *lasts_, Orientation acceleration_)
	:Action(name_, value_, pipeline_), left_wheel(nullptr), right_wheel(nullptr), position(position_),
	 orientation(orientation_), acceleration(acceleration_), destination(destination_), angle_threshold(0.1), distance_threshold(1.2), lasts(lasts_)
{

}

void GoToPosition::setActuators(std::vector<std::vector<Actuator *> > &actuators)
{
	Action::actuators = actuators;
	left_wheel = (WheelVREP *) (actuators[0][0]);
	right_wheel = (WheelVREP *) (actuators[0][1]);
}

void GoToPosition::act()
{
  if (pipeline){
    
    // Vertical control:
    float e=(0.5 - position.getZ());
    float timeStep=(float)std::chrono::duration_cast<std::chrono::milliseconds>(
            position.getTime() - lasts->prevPos.getTime()
          ).count() / 1000;
    float vel = (position.getZ() - lasts->prevPos.getZ()) / timeStep;
    float thrust = 50.2 + e * 2 + vel * -2;
    //std::cout << e << " " << vel << " " <<  timeStep << std::endl;

    // Horizontal control: 
		Position res = destination - position;
    float alphaE=orientation.getAlpha();
    float betaE=orientation.getBeta();
    lasts->pAlphaI = lasts->pAlphaI + alphaE * timeStep;
    lasts->pBetaI  = lasts->pBetaI  + betaE  * timeStep;

    // Stabilization
    float alphaCorr = 0.017 * alphaE + 0.01 * (alphaE - lasts->pAlphaE) / timeStep + lasts->pAlphaI * 0.000005;
    float betaCorr  = 0.017 * betaE  + 0.01 * (betaE  - lasts->pBetaE ) / timeStep + lasts->pBetaI  * 0.000005;
    //std::cout << alphaE << " " << (alphaE - lasts->pAlphaE) / timeStep << " " <<  timeStep << std::endl;
    
    // Go to target
    alphaCorr = alphaCorr + res.getY() * 0.00003 - 0.0003 * (position.getY() - lasts->prevPos.getY()) / timeStep;
    betaCorr  = betaCorr  - res.getX() * 0.00003 + 0.0003 * (position.getX() - lasts->prevPos.getX()) / timeStep;
    
    lasts->pAlphaE=alphaE;
    lasts->pBetaE=betaE;
    
    // Rotational control:
    float euler = orientation.getGamma();
    float rotCorr = euler * 0.000003 + 0.000004 * (euler - lasts->prevEuler);
    lasts->prevEuler = euler;

    lasts->prevPos.setPosition(position.getX(), position.getY(), position.getZ(), position.getTime());

	  std::shared_ptr<Action> go_ahead(new SetWheelsSpeed("GoAhead", 1, pipeline, 
          thrust*(1-alphaCorr+betaCorr+rotCorr),
          thrust*(1-alphaCorr-betaCorr-rotCorr),
          thrust*(1+alphaCorr-betaCorr+rotCorr),
          thrust*(1+alphaCorr+betaCorr-rotCorr)
        ));
		pipeline->push(go_ahead);
	}
	else
	{
		std::cout << "Pipeline null\n";
	}
}
