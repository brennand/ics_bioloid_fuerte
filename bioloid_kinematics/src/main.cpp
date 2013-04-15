/*
 * main.cpp
 *
 *  Created on: Nov 16, 2012
 *      Author: Donato Sciarra
 */

#include "bioloid_kinematics/bioloid_kinematics.hpp"

int main(int argc, char** argv){

	ros::init(argc,argv,"bioloid_kinematics");
	Bioloid_Kinematics k;
	ros::spin();

	return 0;
}


