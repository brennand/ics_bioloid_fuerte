/*
 * Bioloid_kinematics.cpp
 *
 *  Created on: Nov 16, 2012
 *
 *      Author: Donato Sciarra (refactoring)
 *
 *      Original Author: Ross Kidson - Karol Hausman
 */


#include "bioloid_kinematics/bioloid_kinematics.hpp"

#include "kinematics_msgs/GetKinematicSolverInfo.h"
#include "kinematics_msgs/GetPositionFK.h"
#include "kinematics_msgs/GetPositionIK.h"
#include "arm_navigation_msgs/convert_messages.h"

Bioloid_Kinematics::Bioloid_Kinematics():
	n_("~")
{

	fk = n_.advertiseService("bioloid_kinematics/FK", &Bioloid_Kinematics::forwardKinematics, this);
	ik = n_.advertiseService("bioloid_kinematics/IK", &Bioloid_Kinematics::inverseKinematics, this);

	robotFeedback = n_.subscribe("/bioloid_interface/state", 1000, &Bioloid_Kinematics::robotStateCallback, this);

}

Bioloid_Kinematics::~Bioloid_Kinematics(){}

void Bioloid_Kinematics::robotStateCallback(const sensor_msgs::JointState::ConstPtr msg) {
	robotState_ = msg;
}

bool Bioloid_Kinematics::isRobotStateAvailable(){
	if(!robotState_){
		ROS_WARN("Please start the package bioloid_interface before invoking the service.");
		return false;
	}
	return true;
}

int Bioloid_Kinematics::getIdxFromJointName(const std::vector<std::basic_string<char> > & nameVector, const std::basic_string<char> qryJoint) {
	return (int) (std::find(nameVector.begin(), nameVector.end(), qryJoint) - nameVector.begin());
}

bool Bioloid_Kinematics::getIk(const std::string part, const geometry_msgs::PoseStamped req_pose, sensor_msgs::JointState& result){

	if(!isRobotStateAvailable())
		return false;

	const std::string ik_solver_info("/" + part + "/get_ik_solver_info");
	const std::string ik("/" + part + "/get_ik");

	ros::service::waitForService(ik_solver_info);
	ros::service::waitForService(ik);

	ROS_DEBUG_STREAM("The service: \n" << ik << "\nis available and is about to process the request.");

	ros::ServiceClient query_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> ( ik_solver_info );
	ros::ServiceClient ik_client = n_.serviceClient<kinematics_msgs::GetPositionIK> ( ik );

	// define the service messages
	kinematics_msgs::GetKinematicSolverInfo::Request request;
	kinematics_msgs::GetKinematicSolverInfo::Response response;

	if (query_client.call(request, response)) {
		for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
			ROS_DEBUG("BioloidMaster::getIk: Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str());
		}
	} else {
		ROS_ERROR("Could not retrieve joint names from kinematic solver info service");
		return false;
	}

	// define the service messages
	kinematics_msgs::GetPositionIK::Request gpik_req;
	kinematics_msgs::GetPositionIK::Response gpik_res;

	gpik_req.timeout = ros::Duration(1,0);


	gpik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
	gpik_req.ik_request.pose_stamped.pose.position = req_pose.pose.position;
	gpik_req.ik_request.pose_stamped.pose.orientation = req_pose.pose.orientation;
	gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

	// initiate the seed (starting configuration) with the current robot pose
	for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
		int stateIdx = getIdxFromJointName(robotState_->name, response.kinematic_solver_info.joint_names[i]);
		gpik_req.ik_request.ik_seed_state.joint_state.position[i] = robotState_->position[stateIdx];
	}

	ROS_DEBUG_STREAM("BioloidMaster::getIk seed state: " << gpik_req.ik_request.ik_seed_state);

	// make the IK call here
	if (ik_client.call(gpik_req, gpik_res)) {
		if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS) {
			result = gpik_res.solution.joint_state;
		} else {
			ROS_DEBUG("Inverse kinematics failed, trying different seed....");
			for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
				gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position) / 2.0;
			}
			// second try with new seed
			ROS_DEBUG_STREAM("BioloidMaster::getIk NEW seed state: " << gpik_req.ik_request.ik_seed_state);
			ik_client.call(gpik_req, gpik_res);
			if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS) {
				ROS_DEBUG("Inverse kinematics successful");
				result = gpik_res.solution.joint_state;
			} else {
				ROS_ERROR("Inverse kinematics failed");
				ROS_ERROR_STREAM("Value of error code: " << arm_navigation_msgs::armNavigationErrorCodeToString(gpik_res.error_code));
				return false;
			}
		}
	} else {
		ROS_ERROR("Inverse kinematics service call failed");
		return false;
	}
	return true;
}

bool Bioloid_Kinematics::getFk(const std::string part, const sensor_msgs::JointState joint_pos, std::vector<geometry_msgs::PoseStamped>& result){

	if(!isRobotStateAvailable())
		return false;

	const std::string fk_solver_info("/" + part + "/get_fk_solver_info");
	const std::string fk("/" + part + "/get_fk");

	ros::service::waitForService( fk_solver_info );
	ros::service::waitForService( fk );

	ros::ServiceClient query_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> ( fk_solver_info );
	ros::ServiceClient fk_client = n_.serviceClient<kinematics_msgs::GetPositionFK> ( fk );

	// define the service messages
	kinematics_msgs::GetKinematicSolverInfo::Request request;
	kinematics_msgs::GetKinematicSolverInfo::Response response;
	if (query_client.call(request, response)) {
		for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
			ROS_DEBUG("BioloidMaster::getFk: Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str());
		}
	} else {
		ROS_ERROR("Could not call query service");
	}

	// define the service messages
	kinematics_msgs::GetPositionFK::Request fk_request;
	kinematics_msgs::GetPositionFK::Response fk_response;
	fk_request.header.frame_id = "base_link";
	fk_request.fk_link_names = response.kinematic_solver_info.link_names;
	fk_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
	fk_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;
	for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
		fk_request.robot_state.joint_state.position[i] = joint_pos.position[i];
	}

	if (fk_client.call(fk_request, fk_response)) {
		if (fk_response.error_code.val == fk_response.error_code.SUCCESS) {
			for (unsigned int i = 0; i < fk_response.pose_stamped.size(); i++) {
				ROS_DEBUG_STREAM("BioloidMaster::getFk: Link    : " << fk_response.fk_link_names[i].c_str());
				ROS_DEBUG_STREAM("BioloidMaster::getFk: Position: "
						<< fk_response.pose_stamped[i].pose.position.x
						<< "," << fk_response.pose_stamped[i].pose.position.y
						<< "," << fk_response.pose_stamped[i].pose.position.z);
				ROS_DEBUG("BioloidMaster::getFk: Orientation: %f %f %f %f",
						fk_response.pose_stamped[i].pose.orientation.x,
						fk_response.pose_stamped[i].pose.orientation.y,
						fk_response.pose_stamped[i].pose.orientation.z,
						fk_response.pose_stamped[i].pose.orientation.w);
				result.push_back(fk_response.pose_stamped[i]);
			}
		} else {
			ROS_ERROR("Forward kinematics failed");
			return false;
		}
	} else {
		ROS_ERROR("Forward kinematics service call failed");
		return false;
	}
	return true;
}

bool Bioloid_Kinematics::forwardKinematics(bioloid_kinematics::ForwardKinematics::Request& request, bioloid_kinematics::ForwardKinematics::Response& response){

	if(getFk(request.part,request.joint_pos,response.joint_position)){
		return true;
	}
	return false;
}

bool Bioloid_Kinematics::inverseKinematics(bioloid_kinematics::InverseKinematics::Request& request, bioloid_kinematics::InverseKinematics::Response& response){

	if(getIk(request.part,request.req_pose,response.joint_pos)){
		return true;
	}
	return false;
}
