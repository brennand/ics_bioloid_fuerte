/*
 * Copyright (c) 2012, Brennand Pierce
 * Bren@tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


extern "C" {
	#include <dynamixel_SDK/dynamixel.h>
}


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

bool init_dynamixel();

//definition of the bits set in the dynamixels ( see Robotis Dynamixel Wiki )

#define P_ID   								3
#define P_TORQUE_ENABLED			24
#define P_GOAL_POSITION_L   	30
#define P_GOAL_POSITION_H   	31
#define P_GOAL_SPEED_L				32
#define P_GOAL_SPEED_H				33
#define P_GOAL_TORQUELIMIT_L	34
#define P_GOAL_TORQUELIMIT_H	35
#define P_PRESENT_POSITION_L  36
#define P_PRESENT_POSITION_H  37
#define P_PRESENT_SPEED_L    	38
#define P_PRESENT_SPEED_H    	39
#define P_PRESENT_LOAD_L    	40
#define P_PRESENT_LOAD_H    	41
#define P_MOVING       				46

//defines to calculate from tick to SI units
#define TICK2RAD				*300*2*M_PI/360/1023 // convert Ticks to Radiant
#define RAD2TICK				/300/2/M_PI*360*1023 // convert Radiant to Ticks
#define TICKSPSEC2RADPSEC		*2*M_PI/60*0.111 //convert Ticks/sec to Radiant/sec
#define RADPSEC2TICKSPSEC		/2/M_PI*60/0.111 // convert Radiant/sec to Ticks/sec

//Values from the para server.
std::vector<std::string> name;					//Stores joint names
std::vector<int> servo_number;					//The servo number, used to communicate with servo
std::vector<int> joint_encoder_offset;	//The offset of the servo to make the standard home poistion
std::vector<double> angle_max;					//Max angle, this coulde be used to limit the range of motor
std::vector<double> angle_min;					//Min angle, this coulde be used to limit the range of motor
//Vales to send to the bioloid:
std::vector<int> des_pos; 	//rad
std::vector<int> des_vel;	//rad/s
//Values to publish:
std::vector<double> pos;	//rad
std::vector<double> vel;	//rad/s
std::vector<double> eff;	//not used yet, could get motor current.

std::vector<double> raw_pos;	//tick
std::vector<double> raw_vel;	//tick/s
std::vector<double> raw_eff;	//not used yet, could get motor current.

bool torque = true; // this turns the motors on and off;  
int number_of_joints;

	/******************************************************************
	*
  * 			This function is called when we recieve a desired message, in 
  *				the case of the bioloid this is rad or rad/s
  *
  ******************************************************************/

void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg){

	
	for (int i = 0 ; i < number_of_joints; i++){

		/******************************************************************
	  * 				Check to see if the desired rad is in the max - min range
	  ******************************************************************/

		double pos_rad = msg->position.at(i);

		if(pos_rad > angle_max.at(i)){
			pos_rad = angle_max.at(i);
		}else if(pos_rad < angle_min.at(i)){
			pos_rad = angle_min.at(i);
		}

		/******************************************************************
	  * 				Convert the real numbers into ticks for servos.
	  ******************************************************************/
	
		int v = (int)(msg->velocity.at(i) RADPSEC2TICKSPSEC);	//We recieve rad/s, we convert to tick/s.
		int p = (int)(pos_rad RAD2TICK);					//We recieve rad, we convert to ticks.

		/******************************************************************
	  * 				Make sure It's in the range of the servo.
	  ******************************************************************/
		
		//now make sure it's in range.
		if(p > 1023){
			des_pos.at(i) = 1023;
		}else if(p < 0){
			des_pos.at(i) = 0;
		}else{
			des_pos.at(i) = p;
		}

		//now make sure it's in range.
		if(v > 1023){
			des_vel.at(i) = 1023;
		}else if(v < 0){
			des_vel.at(i) = 0;
		}else{
			des_vel.at(i) = v;
		}

	}

}

	/******************************************************************
	*
  * 			Main loop.
  *
  ******************************************************************/


int main(int argc, char **argv)
{  
  ros::init(argc, argv, "bioloid_interface");
  ros::NodeHandle n("~");
  
  //This is the msg listener and the msg caster.
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("state", 1000);
  ros::Publisher raw_js_pub = n.advertise<sensor_msgs::JointState>("raw_state", 1000);
  ros::Subscriber sub = n.subscribe("command", 1000, desiredCallback);

	/******************************************************************
  * 			Get all the variables from the parameter server.
  ******************************************************************/

  getParamVector_string(n,"/bioloid/joints/name",&name);
    
  getParamVector_int(n,"/bioloid/joints/servo_number",&servo_number);
  getParamVector_int(n,"/bioloid/joints/joint_encoder_offset",&joint_encoder_offset);

  getParamVector_double(n,"/bioloid/joints/angle_max",&angle_max);     
  getParamVector_double(n,"/bioloid/joints/angle_min",&angle_min); 
    
	// Make the code more readable by using number_of_joints for loops.
	number_of_joints = name.size(); 
    
	/******************************************************************
  * 			Connect to bioloid
  ******************************************************************/

	if(init_dynamixel() == true){
			ROS_INFO("connected to bioloid");
	} else {
			ROS_INFO("falied to connected bioloid");
			return 0;
	}

	/******************************************************************
  * 			Setup ROS
  ******************************************************************/

  //Communicate at 200hz
  ros::Rate loop_rate(200);
  //Initialice the time.
  ros::Time::init();
	//ROS counter
  int counter = 0;

  ROS_INFO("Number of joints: %d", int(number_of_joints));
  ROS_INFO("bioloid interface started.");

	/******************************************************************
  * 			Initialize all the vectors so that number of joints = vector size.
  ******************************************************************/

	pos.clear();
 	vel.clear();
 	eff.clear();

	raw_pos.clear();
 	raw_vel.clear();
 	raw_eff.clear();

	des_pos.clear();
	des_vel.clear();
	
	for (int i = 0 ; i < number_of_joints ; i++){
		pos.push_back(0.0);
		vel.push_back(0.0);
		eff.push_back(0.0);
		
		raw_pos.push_back(0.0);
		raw_vel.push_back(0.0);
		raw_eff.push_back(0.0);
		
		des_pos.push_back(0);
		des_vel.push_back(0);
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

		/******************************************************************
	  * 				Get Dynamixel position and speed
	  ******************************************************************/

		for (int i = 0 ; i < number_of_joints  ; i++){
		
			//Get raw position from dynamixel
			do{
				raw_pos.at(i) = (double)dxl_read_word( servo_number.at(i), P_PRESENT_POSITION_L);
			}while(dxl_get_result( ) != COMM_RXSUCCESS );
			
			//get raw velocity from dynamixel
			do{
				raw_vel.at(i) = (double)dxl_read_word(servo_number.at(i), P_PRESENT_SPEED_L);
			}while(dxl_get_result( ) != COMM_RXSUCCESS );
				
			//Work out if vel direction, 
			//If a value is in the rage of 0~1023, it means that the motor rotates to the CCW direction.
			//If a value is in the rage of 1024~2047, it means that the motor rotates to the CW direction.

			if(raw_vel.at(i)  > 1024.0) {
				raw_vel.at(i) = -(raw_vel.at(i)-1024);
			}
	
			//convert the raw into real number.
			vel.at(i) = (raw_vel.at(i) TICKSPSEC2RADPSEC );		
			pos.at(i) = ((raw_pos.at(i)-joint_encoder_offset.at(i)) TICK2RAD );
		
		}

		/******************************************************************
	  * 				Send the desired position to the Dynamixel 
	  ******************************************************************/

		if(torque) {
			for (int i = 0 ; i < number_of_joints  ; i++){
	
				do{
					dxl_write_word( servo_number.at(i), P_GOAL_SPEED_L,(unsigned)des_vel.at(i));
				}while(dxl_get_result( ) != COMM_RXSUCCESS );
	       
				do{
					dxl_write_word( servo_number.at(i), P_GOAL_POSITION_L, (unsigned)(des_pos.at(i)+joint_encoder_offset.at(i)));
				}while(dxl_get_result( ) != COMM_RXSUCCESS );
	
			}
		}
	
		/******************************************************************
	  * 				Publish the raw and real positon on the joints
	  ******************************************************************/

		js.name = name;  
    js.position = pos;
    js.velocity = vel;
    js.effort   = eff;

    js_pub.publish(js);

    js.position = raw_pos;
    js.velocity = raw_vel;
    js.effort   = raw_eff;

    raw_js_pub.publish(js);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
	/******************************************************************
  *
  * 				Before exit turn everything off.
  *
  ******************************************************************/

	//turn the torque of the servo off
	do{
		dxl_write_byte( 254, P_TORQUE_ENABLED, 0);
	}while(dxl_get_result( ) != COMM_RXSUCCESS );


	//clode the dynamical port.
	dxl_terminate();

  return 0;
}


	/******************************************************************
  *
  * 				try and connect to the bioloid if succesful return true, other wish return false.
  *
  ******************************************************************/

bool init_dynamixel(){
	
		// setup initial data for the Dynamixels
	int baudnum = 1;

	for (unsigned int deviceIndex = 0 ; deviceIndex < 10 ; deviceIndex++){
		if(dxl_initialize(deviceIndex, baudnum) == 0){
			// not possible to connect to dynamixels
			ROS_INFO( "[initializeDynamixel] Failed to open USB2Dynamixel on /dev/ttyUSB%d", deviceIndex );
			
		} else {
			// connection succesfull
					ROS_INFO( "[initializeDynamixel] Succeed to open USB2Dynamixel on /dev/ttyUSB%d", deviceIndex );
					return true;
		}
		
		
		if(deviceIndex == 9){
			return false;
		}
	}
	
	return false;
	
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
