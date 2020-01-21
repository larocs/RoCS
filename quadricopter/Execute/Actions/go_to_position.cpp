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
	:Action(name_, value_, pipeline_), left_wheel(nullptr), right_wheel(nullptr), position(position_), lasts(lasts_),
	 orientation(orientation_), destination(destination_), angle_threshold(0.1), distance_threshold(1.2), acceleration(acceleration_)
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
    float e=(0.7 - position.getZ());
    float timeStep=(float)std::chrono::duration_cast<std::chrono::milliseconds>(
            position.getTime() - lasts->prevPos.getTime()
          ).count() / 1000;
    float vel = (position.getZ() - lasts->prevPos.getZ()) / timeStep;
    float thrust = 50.2 + e * 2 + vel * -2;
    //std::cout << e << " " << vel << " " <<  timeStep << std::endl;
//*
    // Horizontal control: 
		Position res = destination - position;
    float alphaE=orientation.getAlpha();
    float betaE=orientation.getBeta();
    lasts->pAlphaI = lasts->pAlphaI + alphaE * timeStep;
    lasts->pBetaI  = lasts->pBetaI  + betaE  * timeStep;

 //   if(lasts->pAlphaE==0) // TODO tirar
 //     lasts->pAlphaE=alphaE;
 //   if(lasts->pBetaE==0)
 //     lasts->pBetaE=betaE;
    
    // Stabilization
    float alphaCorr = 0.017 * alphaE + 0.01 * (alphaE - lasts->pAlphaE) / timeStep + lasts->pAlphaI * 0.000005;
    float betaCorr  = 0.017 * betaE  + 0.01 * (betaE  - lasts->pBetaE ) / timeStep + lasts->pBetaI  * 0.000005;
    std::cout << alphaE << " " << (alphaE - lasts->pAlphaE) / timeStep << " " <<  timeStep << std::endl;
    
    // Go to target
    alphaCorr = alphaCorr + res.getY() * 0.00004 - 0.0006 * (position.getY() - lasts->prevPos.getY()) / timeStep;
    betaCorr  = betaCorr  - res.getX() * 0.00004 + 0.0006 * (position.getX() - lasts->prevPos.getX()) / timeStep;
    
    //alphaCorr=alphaCorr>0.02 || alphaCorr<-0.02 ?0.02:alphaCorr; TODO tirar
    //betaCorr=betaCorr>0.02 || betaCorr<-0.02 ?0.02:betaCorr;

    lasts->pAlphaE=alphaE;
    lasts->pBetaE=betaE;
    
    // Rotational control:
    float euler = orientation.getGamma();
    float rotCorr = euler * 0.00003 + 0.00004 * (euler - lasts->prevEuler);
    lasts->prevEuler = euler; // 0.3 + 4.2

    lasts->prevPos.setPosition(position.getX(), position.getY(), position.getZ(), position.getTime());
   /* 
    float alphaCorr=0,
          betaCorr =0,
          rotCorr  =0;
    */
    // NEW CONTROL (with parameters used inside V-REP script)
    /* // Vertical control:
    float e=(0.5 - position.getZ());
    float timeStep=(float)std::chrono::duration_cast<std::chrono::microseconds>(
            position.getTime() - lasts->prevPos.getTime()
          ).count(); // TODO Acho que tem que mudar este divisor XXX Tirei o divisor
    float vel = (position.getZ() - lasts->prevPos.getZ()) / timeStep;
    float thrust = 50.3 + e * 50 + vel * -20;

    // Horizontal control: 
		Position res = destination - position;
    float alphaE=orientation.getAlpha();
    float betaE=orientation.getBeta();
    lasts->pAlphaI = lasts->pAlphaI + alphaE * timeStep;
    lasts->pBetaI  = lasts->pBetaI  + betaE  * timeStep;

 //   if(lasts->pAlphaE==0) // TODO tirar
 //     lasts->pAlphaE=alphaE;
 //   if(lasts->pBetaE==0)
 //     lasts->pBetaE=betaE;
    
    // Stabilization
    float alphaCorr = 0.09 * alphaE + 0.2 * (alphaE - lasts->pAlphaE) / timeStep + lasts->pAlphaI * 0.05;
    float betaCorr  = 0.09 * betaE  + 0.2 * (betaE  - lasts->pBetaE ) / timeStep + lasts->pBetaI  * 0.05;
    
    // Go to target
    alphaCorr = alphaCorr + res.getY() * 0.02 - 0.03 * (position.getY() - lasts->prevPos.getY()) / timeStep;
    betaCorr  = betaCorr  - res.getX() * 0.02 + 0.03 * (position.getX() - lasts->prevPos.getX()) / timeStep;
    
    //alphaCorr=alphaCorr>0.02 || alphaCorr<-0.02 ?0.02:alphaCorr; TODO tirar
    //betaCorr=betaCorr>0.02 || betaCorr<-0.02 ?0.02:betaCorr;

    lasts->pAlphaE=alphaE;
    lasts->pBetaE=betaE;
    
    // Rotational control:
    float euler = orientation.getGamma();
    float rotCorr = euler * 0.3 + 4.2 * (euler - lasts->prevEuler);
    lasts->prevEuler = euler;
    lasts->prevPos.setPosition(position.getX(), position.getY(), position.getZ(), position.getTime());
 //*/
 
    // OLD control:
    /*
    float pv=(0.7-position.getZ())*2; // (0.51) distance to desired vertical position / pParam=2
    float timeStep=(float)std::chrono::duration_cast<std::chrono::microseconds>(
            position.getTime()-lasts->prevPos.getTime()
          ).count()/1000000;
    float vel=(position.getZ()-lasts->prevPos.getZ())/timeStep;
    float thrust=50.2+pv+vel*(-2); // vParam = -2
    lasts->prevPos.setPosition(position.getX(), position.getY(), position.getZ(), position.getTime());

    // Horizontal control: 
		Position res = destination - position;
    float alphaE=orientation.getAlpha(); //(matrix[9]-matrix[11]);
    if(lasts->pAlphaE==0)
      lasts->pAlphaE=alphaE;
    float alphaCorr=0.07*alphaE+0.25*(alphaE-lasts->pAlphaE); // TODO 0.1 and 0.5 with other drone
    
    float betaE=orientation.getBeta(); //(matrix[8]-matrix[11]);
    if(lasts->pBetaE==0)
      lasts->pBetaE=betaE;
    float betaCorr=0.07*betaE+0.25*(betaE-lasts->pBetaE);
    //float betaCorr=-0.25*betaE-2.1*(betaE-lasts->pBetaE);
    
    lasts->pAlphaE=alphaE;
    lasts->pBetaE=betaE;

    // alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    alphaCorr=alphaCorr+res.getY()*0.001+0.03*(res.getY()-lasts->psp2); // TODO 0.0005 and 0.02 with other drone
    betaCorr=betaCorr-res.getX()*0.001-0.03*(res.getX()-lasts->psp1);
    std::cout << alphaCorr << std::endl;
    alphaCorr=alphaCorr>0.02 || alphaCorr<-0.02 ?0.02:alphaCorr;
    betaCorr=betaCorr>0.02 || betaCorr<-0.02 ?0.02:betaCorr;
    
    lasts->psp2=res.getY();
    lasts->psp1=res.getX();
    // Position e Matrix
    
    // Rotational control:
    float rotCorr=0; // TODO here
    /*euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3] */

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
