/*
 *      Author: Donato Sciarra
 */


#include "ros/ros.h"
#include "bioloid_interactive_markers/interactive_markers.hpp"

int main(int argc, char** argv){

	ros::init(argc,argv,"bioloid_interactive_markers");

	BioloidInteractiveMarkers bim;

	ros::spin();
	return 0;
}

