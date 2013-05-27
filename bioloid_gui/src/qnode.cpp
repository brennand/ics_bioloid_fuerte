/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/bioloid_gui/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bioloid_gui {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void QNode::jointsRawCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	if (change == false) {
		pos_raw = msg->position;
		vel_raw = msg->velocity;
		eff_raw = msg->effort;
		emit jointUpdate();
	}
}

void QNode::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	if (change == false) {
		pos = msg->position;
		vel = msg->velocity;
		eff = msg->effort;
		emit jointUpdate();
	}
}

QNode::QNode(int argc, char** argv) :
	init_argc(argc), init_argv(argv) {
	for (int i = 0; i < 18; i++) {
		pos.push_back(0.0);
		vel.push_back(5.0);
		eff.push_back(0.0);
		des_pos.push_back(0.0);
		des_vel.push_back(0.0);
		des_eff.push_back(0.0);
		pos_raw.push_back(0.0);
		vel_raw.push_back(0.0);
		eff_raw.push_back(0.0);
	}
	change = false;
	test = false;
	userInteraction = false;
}

QNode::~QNode() {
	if (ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();
}

bool QNode::init() {
	ros::init(init_argc, init_argv, "bioloid_gui");
	if (!ros::master::check()) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	counter = 0;

	getParamVector<std::string>(n, "/bioloid/joints/name", &name);

	getParamVector<int>(n, "/bioloid/joints/servo_number", &servo_number);
	getParamVector<int>(n, "/bioloid/joints/joint_encoder_offset", &joint_encoder_offset);

	getParamVector<double>(n, "/bioloid/joints/angle_max", &angle_max);
	getParamVector<double>(n, "/bioloid/joints/angle_min", &angle_min);

	// Add your ros communications here.

	chatter_publisher = n.advertise<sensor_msgs::JointState> ("/bioloid_interface/command", 1000);
	chatter_sub1 = n.subscribe("/bioloid_interface/state", 1000, &QNode::jointsCallback, this);
	chatter_sub2 = n.subscribe("/bioloid_interface/raw_state", 1000, &QNode::jointsRawCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string, std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings, "bioloid_gui");
	if (!ros::master::check()) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	counter = 0;

	getParamVector<std::string>(n, "/bioloid/joints/name", &name);

	getParamVector<int>(n, "/bioloid/joints/servo_number", &servo_number);
	getParamVector<int>(n, "/bioloid/joints/joint_encoder_offset", &joint_encoder_offset);

	getParamVector<double>(n, "/bioloid/joints/angle_max", &angle_max);
	getParamVector<double>(n, "/bioloid/joints/angle_min", &angle_min);

	// Add your ros communications here.

	chatter_publisher = n.advertise<std_msgs::String> ("/bioloid_interface/command", 1000);
	chatter_sub1 = n.subscribe("/bioloid_interface/state", 1000, &QNode::jointsCallback, this);
	chatter_sub2 = n.subscribe("/bioloid_interface/raw_state", 1000, &QNode::jointsRawCallback, this);
	start();
	return true;
}

bool QNode::saveConfigurationToFile(std::string filepath){

	return true;

}

void QNode::run() {
	ros::Rate loop_rate(1);
	while (ros::ok()) {

		counter++;
		sensor_msgs::JointState js;
		js.header.seq = counter;
		js.header.stamp = ros::Time::now();
		js.header.frame_id = "/base_link";

		js.name = name;

		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "hello world " << count;
		//msg.data = ss.str();
		if (change == true) {
			js.position = des_pos;
			js.velocity = des_vel;
			js.effort = des_eff;
			chatter_publisher.publish(js);
			change = false;
			//usleep(100);
		}
		if (test) {
			for (int i = 0; i < 18; i++) {
				des_pos[i] = 0;
			}
			for (int i = 0; i < 18; i++) {
				des_pos[i] = 3.15;
				if (i == 8 || i == 16) {
					des_pos[i + 1] = 3.15;
				}
				if (i == 0 || i == 2 || i == 4 || i == 10 || i == 12 || i == 14) {
					des_pos[i + 1] = -3.15;
				}
				js.position = des_pos;
				js.velocity = des_vel;
				js.effort = des_eff;
				chatter_publisher.publish(js);
				ros::spinOnce();
				loop_rate.sleep();
				usleep(500);

				des_pos[i] = -3.15;
				if (i == 8 || i == 16) {
					des_pos[i + 1] = -3.15;
					i++;
				}
				if (i == 0 || i == 2 || i == 4 || i == 10 || i == 12 || i == 14) {
					des_pos[i + 1] = 3.15;
					i++;
				}
				js.position = des_pos;
				js.velocity = des_vel;
				js.effort = des_eff;
				chatter_publisher.publish(js);
				ros::spinOnce();
				loop_rate.sleep();
				usleep(500);

				for (int i = 0; i < 18; i++) {
					des_pos[i] = 0;
				}
				js.position = des_pos;
				js.velocity = des_vel;
				js.effort = des_eff;
				chatter_publisher.publish(js);
				ros::spinOnce();
				loop_rate.sleep();
				usleep(500);
			}
			test = false;
			//usleep(100);
		}/*
		 else{
		 js.position = pos;
		 js.velocity = vel;
		 js.effort = eff;
		 chatter_publisher.publish(js);
		 }*/

		//log(Info,std::string("I sent: ")+msg.data);

		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::updateJoints() {
	change = true;
}

void QNode::testJoints() {
	test = true;
}

void QNode::log(const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(), 1);
	std::stringstream logging_model_msg;
	switch (level) {
	case (Debug): {
		ROS_DEBUG_STREAM(msg);
		logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	case (Info): {
		ROS_INFO_STREAM(msg);
		logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	case (Warn): {
		ROS_WARN_STREAM(msg);
		logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	case (Error): {
		ROS_ERROR_STREAM(msg);
		logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	case (Fatal): {
		ROS_FATAL_STREAM(msg);
		logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
			new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}

template <typename T> void QNode::getParamVector (ros::NodeHandle n, const std::string Var, std::vector<T>* const Vec){
	XmlRpc::XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpc::XmlRpcValue::TypeArray);

	//create a dummy object for sanity check
	T* dummy = new T;
	XmlRpc::XmlRpcValue obj(*dummy);

	for (int index = 0; index < gainList.size(); index++) {
		ROS_ASSERT(gainList[index].getType() == obj.getType());
		Vec->push_back(static_cast<T> (gainList[index]));
	}

	//clean-up
	delete dummy;
}

//void QNode::getParamVector_string(ros::NodeHandle n, string Var, vector<std::string> *Vec) {
//
//	XmlRpc::XmlRpcValue gainList;
//	n.getParam(Var, gainList);
//	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);
//
//	for (int index = 0; index < gainList.size(); index++) {
//		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeString);
//		Vec->push_back(static_cast<std::string> (gainList[index]));
//	}
//
//}
//
//void QNode::getParamVector_int(ros::NodeHandle n, string Var, vector<int> *Vec) {
//
//	XmlRpc::XmlRpcValue gainList;
//	n.getParam(Var, gainList);
//	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);
//
//	for (int index = 0; index < gainList.size(); index++) {
//		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeInt);
//		Vec->push_back(static_cast<int> (gainList[index]));
//	}
//}
//
//void QNode::getParamVector_double(ros::NodeHandle n, string Var,
//		vector<double> *Vec) {
//
//	XmlRpc::XmlRpcValue List;
//	n.getParam(Var, List);
//	ROS_ASSERT(List.getType() == XmlRpcValue::TypeArray);
//
//	for (int index = 0; index < List.size(); index++) {
//		ROS_ASSERT(List[index].getType() == XmlRpcValue::TypeDouble);
//		Vec->push_back(static_cast<double> (List[index]));
//	}
//}

} // namespace bioloid_gui
