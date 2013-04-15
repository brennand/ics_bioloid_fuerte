/*
 * Bioloid_kinematics.cpp
 *
 *  Created on: Nov 16, 2012
 *
 *      Author: Donato Sciarra (refactoring)
 *
 *      Original Author: Ross Kidson - Karol Hausman
 */

#ifndef BIOLOID_KINEMATICS_HPP_
#define BIOLOID_KINEMATICS_HPP_

#include <string>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include "bioloid_kinematics/ForwardKinematics.h"
#include "bioloid_kinematics/InverseKinematics.h"

class Bioloid_Kinematics{
public:
	Bioloid_Kinematics();
	virtual ~Bioloid_Kinematics();

	/*
	 * Service callback methods
	 */
	bool forwardKinematics(bioloid_kinematics::ForwardKinematics::Request& request, bioloid_kinematics::ForwardKinematics::Response& response);
	bool inverseKinematics(bioloid_kinematics::InverseKinematics::Request& request, bioloid_kinematics::InverseKinematics::Response& response);

	void robotStateCallback(const sensor_msgs::JointState::ConstPtr msg);

private:
	ros::NodeHandle n_;
	ros::ServiceServer fk, ik;
	ros::Subscriber robotFeedback;

	sensor_msgs::JointState::ConstPtr robotState_;

	/*
	 * Check if RobotStateActual has been set
	 */
	bool isRobotStateAvailable();

	int getIdxFromJointName(const std::vector<std::basic_string<char> > & nameVector, const std::basic_string<char> qryJoint);

	/*
	 * Forward Kinematics
	 * @param part:			one of the 4 Bioloid's parts
	 * @param joint_pos:	joint variables
	 * @param result:		the positions of the joints of "part"
	 *
	 * @return: true on success, false otherwise
	 */
	bool getFk(const std::string part, const sensor_msgs::JointState joint_pos, std::vector<geometry_msgs::PoseStamped> &result);

	/*
	 * Inverse Kinematics
	 * @param part:			one of the 4 Bioloid's parts
	 * @param req_pose:		desired pose of the part's endeffector
	 * @param result:		joint variables of "part"
	 *
	 * @return: true on success, false otherwise
	 */
	bool getIk(const std::string part, const geometry_msgs::PoseStamped req_pose, sensor_msgs::JointState &result);
};


#endif /* BIOLOID_KINEMATICS_HPP_ */
