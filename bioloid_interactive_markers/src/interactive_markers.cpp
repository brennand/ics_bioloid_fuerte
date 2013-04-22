/*
 *      Author: Donato Sciarra
 */

#include "bioloid_interactive_markers/interactive_markers.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "bioloid_kinematics/InverseKinematics.h"

BioloidInteractiveMarkers::~BioloidInteractiveMarkers(){}

BioloidInteractiveMarkers::BioloidInteractiveMarkers():
		nh_("~"),
		server_("Bioloid_Interactive_Marker_Sever")
	{

	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/bioloid_interface/command",1000);
//	robot_state_ = nh_.subscribe("/bioloid_interface/state", 1000, &BioloidInteractiveMarkers::robotStateCallback, this);

	//Create a visual marker (objects, xyz (Position), reference frame, name, description)

	Feedback_cb left_arm_cb (boost::bind(&BioloidInteractiveMarkers::processLeftArm,this,_1));
	createMarker(left_arm, -0.23, 0.0, 0.02, "left_arm_marker", "Left Arm", left_arm_cb);

	Feedback_cb right_arm_cb (boost::bind(&BioloidInteractiveMarkers::processRightArm,this,_1));
	createMarker(right_arm, 0.23, 0.0, 0.02, "right_arm_marker", "Right Arm", right_arm_cb);

	Feedback_cb left_leg_cb (boost::bind(&BioloidInteractiveMarkers::processLeftLeg,this,_1));
	createMarker(left_leg, -0.03, 0.0, -0.21, "left_leg_marker", "Left Foot",left_leg_cb);

	Feedback_cb right_leg_cb (boost::bind(&BioloidInteractiveMarkers::processRightLeg,this,_1));
	createMarker(right_leg, 0.03, 0.0, -0.21, "right_leg_marker", "Right Foot",right_leg_cb);

	// Apply changes made since the last call to this method & broadcast an update to all clients.
	server_.applyChanges();

}

//void BioloidInteractiveMarkers::robotStateCallback(const sensor_msgs::JointState::ConstPtr msg){
//	robotState_ = msg;
//}

void BioloidInteractiveMarkers::processLeftLeg(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

	std::string part = "L_leg";
	Pose *pose_ptr = &(marker_pose_.left_leg);

	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
	{
		if(attemptModification(feedback,part,pose_ptr)){
			server_.applyChanges();
		}
	}

}

void BioloidInteractiveMarkers::processRightLeg(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback) {

	std::string part = "R_leg";
	Pose *pose_ptr = &(marker_pose_.right_leg);

	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
	{
		if(attemptModification(feedback,part,pose_ptr)){
			server_.applyChanges();
		}
	}

}

void BioloidInteractiveMarkers::processLeftArm(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback) {

	std::string part = "L_arm";
	Pose *pose_ptr = &(marker_pose_.left_arm);

	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
	{
		if(attemptModification(feedback,part,pose_ptr)){
			server_.applyChanges();
		}
	}

}


void BioloidInteractiveMarkers::processRightArm(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback) {

	std::string part = "R_arm";
	Pose *pose_ptr = &(marker_pose_.right_arm);

	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
	{
		if(attemptModification(feedback,part,pose_ptr)){
			server_.applyChanges();
		}
	}

}

bool BioloidInteractiveMarkers::attemptModification(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback, std::string part_name, Pose *pose_ptr){
	//First check if there is an inverse kinematics available for the requested position
	//if so send commands to bioloid and update the marker postition in rviz

	static const std::string service_name = "/bioloid_kinematics/bioloid_kinematics/IK";

	ros::ServiceClient query_client = nh_.serviceClient<bioloid_kinematics::InverseKinematics> ( service_name );
	if(!ros::service::waitForService(service_name,ros::Duration(5,0))){
		ROS_ERROR_STREAM("The service /bioloid_kinematics/bioloid_kinematics/IK is not available.");
		return false;
	}

	bioloid_kinematics::InverseKinematicsRequest request;
	bioloid_kinematics::InverseKinematicsResponse response;

	request.req_pose.header.stamp = ros::Time::now();
	request.req_pose.pose.orientation.x = feedback->pose.orientation.x;
	request.req_pose.pose.orientation.y = feedback->pose.orientation.y;
	request.req_pose.pose.orientation.z = feedback->pose.orientation.z;
	request.req_pose.pose.orientation.w = feedback->pose.orientation.w;
	request.req_pose.pose.position.x = feedback->pose.position.x;
	request.req_pose.pose.position.y = feedback->pose.position.y;
	request.req_pose.pose.position.z = feedback->pose.position.z;
	request.part = part_name;

	ROS_DEBUG_STREAM("Chain " << part_name << " : " << feedback->pose.orientation.x << " " <<
	feedback->pose.orientation.y << " " << feedback->pose.orientation.z << " " << feedback->pose.orientation.w << " " <<
	feedback->pose.position.x << " " << feedback->pose.position.y << " " << feedback->pose.position.z);

	if(query_client.call(request,response)){
		//apply changes iff there exist an inverse kinematic solution
		updateMarkerPose(feedback,pose_ptr);

		//send joint state to the bioloid
		joint_state_pub_.publish(response);
		return true;
	}

	ROS_ERROR_STREAM("Couldn't convert position of " << part_name << " into Joint values.");
	return false;

}

void BioloidInteractiveMarkers::updateMarkerPose(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback, Pose *pose_ptr){

	pose_ptr->position.x = feedback->pose.position.x;
	pose_ptr->position.y = feedback->pose.position.y;
	pose_ptr->position.z = feedback->pose.position.z;
	pose_ptr->orientation.x = feedback->pose.orientation.x;
	pose_ptr->orientation.y = feedback->pose.orientation.y;
	pose_ptr->orientation.z = feedback->pose.orientation.z;
	pose_ptr->orientation.w = feedback->pose.orientation.w;

}

void BioloidInteractiveMarkers::createMarker(
		visualization_msgs::InteractiveMarker& marker,
		double x,
		double y,
		double z,
		std::string const& name,
		std::string const& description,
		Feedback_cb feedback_cb,
		double scale)
{

	// General specification of the marker
	marker.scale = scale;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.header.frame_id = "/base_link";
	marker.name = name;
	marker.description = description;

	// Create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.02;
	box_marker.scale.y = 0.02;
	box_marker.scale.z = 0.02;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;

	// Create a non-interactive box_control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.markers.push_back(box_marker);

	// Add the box_control to the interactive marker
	marker.controls.push_back(box_control);

	visualization_msgs::InteractiveMarkerControl control;

	createControl(control, 1.0, 1.0, 0.0, 0.0, marker, "move_x", "rotate_x");
	createControl(control, 1.0, 0.0, 0.0, 1.0, marker, "move_y", "rotate_y");
	createControl(control, 1.0, 0.0, 1.0, 0.0, marker, "move_z", "rotate_z");

	server_.insert(marker);
	server_.setCallback(marker.name, feedback_cb);
}

void BioloidInteractiveMarkers::createControl(
		visualization_msgs::InteractiveMarkerControl& control,
		double w,
		double x,
		double y,
		double z,
		visualization_msgs::InteractiveMarker& marker,
		std::string const& move,
		std::string const& rotate) {

	control.orientation.w = w;
	control.orientation.x = x;
	control.orientation.y = y;
	control.orientation.z = z;

	control.name = rotate;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	marker.controls.push_back(control);

	control.name = move;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(control);

}
