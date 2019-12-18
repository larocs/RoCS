#include <iostream>
#include "Robot/robotnik.h"

int main(int argc, char **argv){

	Robotnik robot(argc-1);
	robot.run();

	return 0;
}
