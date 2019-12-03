//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

#include <cmath>

#include "go_to_position.h"
#include "set_wheels_speed.h"

GoToPosition::GoToPosition(const std::string &name_, double value_, Pipeline *pipeline_,
                           Position position_, Orientation orientation_, Position destination_, GoToP3DXPlanner::Lasts *lasts_)
	:Action(name_, value_, pipeline_), left_wheel(nullptr), right_wheel(nullptr), position(position_), lasts(lasts_),
	 orientation(orientation_), destination(destination_), angle_threshold(0.1), distance_threshold(1.2)
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
    float *matrix=left_wheel->getMatrix();
    if(!matrix)
      return;
    //float vel=left_wheel->getVelocity(); // TODO if use, check
		Position res = destination - position;
    
    // Vertical control:
    float pv=(0.7-position.getZ())*2; // (0.51) distance to desired vertical position / pParam=2
    float timeStep=(float)std::chrono::duration_cast<std::chrono::microseconds>(
            position.getTime()-lasts->prevPos.getTime()
          ).count()/1000000;
    float vel=(position.getZ()-lasts->prevPos.getZ())/timeStep;
    float thrust=50.2+pv+vel*(-2); // vParam = -2
    lasts->prevPos.setPosition(position.getX(), position.getY(), position.getZ(), position.getTime());

    // Horizontal control: 
    //sp=sim.getObjectPosition(targetObj,d)
    //m=sim.getObjectMatrix(d,-1)
    //vx={1,0,0}
    //vx=sim.multiplyVector(m,vx)
    //vy={0,1,0}
    //vy=sim.multiplyVector(m,vy)
    //alphaE=(vy[3]-m[12])
    float alphaE=(matrix[9]-matrix[11]);
    float alphaCorr=0.25*alphaE+2.1*(alphaE-lasts->pAlphaE);
    float betaE=(matrix[8]-matrix[11]);
    float betaCorr=-0.25*betaE-2.1*(betaE-lasts->pBetaE);
    lasts->pAlphaE=alphaE;
    lasts->pBetaE=betaE;
    // alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    alphaCorr=alphaCorr+position.getY()*0.005+1*(position.getY()-lasts->psp2);
    betaCorr=betaCorr-position.getX()*0.005-1*(position.getX()-lasts->psp1);
    lasts->psp2=position.getY();
    lasts->psp1=position.getX();
    // Position e Matrix
    
    // Rotational control:
    float rotCorr=0;
    /*euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3] */
    // orientation 3
    
    // Decide of the motor velocities:
    /*
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)

*/


//		std::cout << destination << "\n"; // TODO destination kkk
//		std::cout << position << "\n";

    /*
		if (res.abs() < distance_threshold)
		{
//			std::cout << "stop\n";
			std::shared_ptr<Action> stop(new SetWheelsSpeed("Stop", 1, pipeline, 0, 0));
			pipeline->push(stop);
		}
		else
		{
    //*/
//			std::cout << "go ahead\n";
    //alphaCorr=betaCorr=rotCorr=0; // TODO retirar
    float fator=10;
    alphaCorr/=fator;
    betaCorr/=fator;
    rotCorr/=fator;
	  std::shared_ptr<Action> go_ahead(new SetWheelsSpeed("GoAhead", 1, pipeline, 
          thrust*(1-alphaCorr+betaCorr+rotCorr),
          thrust*(1-alphaCorr-betaCorr-rotCorr),
          thrust*(1+alphaCorr-betaCorr+rotCorr),
          thrust*(1+alphaCorr+betaCorr-rotCorr)
          //thrust*(1-alphaCorr+betaCorr+rotCorr),
          //thrust*(1-alphaCorr-betaCorr-rotCorr),
          //thrust*(1+alphaCorr-betaCorr+rotCorr)
        ));
		pipeline->push(go_ahead);
    //*/
	}
	else
	{
		std::cout << "Pipeline null\n";
	}
}