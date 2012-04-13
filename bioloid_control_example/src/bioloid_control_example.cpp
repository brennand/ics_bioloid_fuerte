/*
 * Example
 * Copyright (C) 201
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


#include <string>
#include <vector>
#include <math.h>

#include <XmlRpcValue.h>

#include "time.h"

using namespace XmlRpc;
using namespace std;

//These functions get the values from the parameter server and puts them into the vector
void getParamVector_int(ros::NodeHandle, string,vector<int> *);
void getParamVector_double(ros::NodeHandle, string,vector<double> *);
void getParamVector_string(ros::NodeHandle, string,vector<std::string> *);

//Values from the para server.
std::vector<std::string> name;   //Stores joint names

//Vales to recieved to the bioloid:
std::vector<double> pos;   
std::vector<double> vel;  
std::vector<double> eff;  

//Vales to send to the bioloid:
std::vector<double> des_pos;   
std::vector<double> des_vel;  
std::vector<double> des_eff; 

int number_of_joints;

	/******************************************************************
	*
  * 			This function is called when we recieve a message with joint position in rad or rad/s
  *
  ******************************************************************/

void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg){

    pos = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

	/******************************************************************
	*
  * 			Main loop.
  *
  ******************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bioloid_control_example");
  ros::NodeHandle n;
  
  //This is the msg listener and the msg caster.
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("/bioloid_interface/command", 1000);
  ros::Subscriber sub = n.subscribe("/bioloid_interface/state", 1000, desiredCallback);


	/******************************************************************
  * 			Get all the variables from the parameter server.
  ******************************************************************/

  getParamVector_string(n,"/bioloid/joints/name",&name);

	number_of_joints = name.size();
    
	/******************************************************************
  * 			Setup ROS
  ******************************************************************/    
    
  ROS_INFO("Number of joints: %d", number_of_joints);
  ROS_INFO("head controller started");

  //Want to communicate at 1Khz
  ros::Rate loop_rate(1000);
  //Initialice the time.
  ros::Time::init();

  int counter = 0;

	/******************************************************************
  * 			Initialize all the vectors so that number of joints = vector size.
  ******************************************************************/

	des_pos.clear();   
	des_vel.clear();  
	des_eff.clear(); 

	for (unsigned int i = 0 ; i < number_of_joints ; ++i){
		des_pos.push_back(0.0);
		des_vel.push_back(0.0);  
		des_eff.push_back(0.0);
	
	}

	/******************************************************************
	*
  * 				Main control loop.
  *
  ******************************************************************/

  while (ros::ok())
  {
    //Send Joint states
    counter ++;
    sensor_msgs::JointState js;
    js.header.seq = counter;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "/world";

    js.name = name;

		/******************************************************************
	  * 				Create a sine wave to the joints
	  ******************************************************************/

    for (unsigned int i = 0 ; i < number_of_joints ; ++i){
			des_pos.at(i) = (((cos(counter*0.003)))/5);
    }

		/******************************************************************
	  * 				Publish the desired positon on the joints
	  ******************************************************************/

    js.position = des_pos;
    js.velocity = des_vel;
    js.effort = des_eff;

    js_pub.publish(js);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

	/******************************************************************
  *
  * 				Next three functions returns the paramenter server value.
  *
  ******************************************************************/

void getParamVector_string(ros::NodeHandle n, string Var,vector<std::string> *Vec){
 
  	XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < gainList.size(); index++)
	{
		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeString);
		Vec->push_back(static_cast<std::string>(gainList[index]));
	}
    
}

void getParamVector_int(ros::NodeHandle n, string Var,vector<int> *Vec){
 
  	XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < gainList.size(); index++)
	{
		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeInt);
		Vec->push_back(static_cast<int>(gainList[index]));
	}
    
}


void getParamVector_double(ros::NodeHandle n, string Var,vector<double> *Vec){
 
  	XmlRpcValue List;
	n.getParam(Var, List);
	ROS_ASSERT(List.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < List.size(); index++)
	{
		ROS_ASSERT(List[index].getType() == XmlRpcValue::TypeDouble);
		Vec->push_back(static_cast<double>(List[index]));
	}
    
}


