/**
 * @file /include/bioloid_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef bioloid_gui_QNODE_HPP_
#define bioloid_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <string>
#include <QThread>
#include <QStringListModel>
#include <vector>
#include <XmlRpcValue.h>
#include <ros/network.h>
#include <sstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace XmlRpc;
namespace bioloid_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	std::vector<std::string> name;					//Stores joint names
	std::vector<int> servo_number;					//The servo number, used to communicate with servo
	std::vector<int> joint_encoder_offset;	//The offset of the servo to make the standard home poistion
	std::vector<double> angle_max;					//Max angle, this coulde be used to limit the range of motor
	std::vector<double> angle_min;					//Min angle, this coulde be used to limit the range of motor
	//Vales recieved from the bioloid:
	std::vector<double> pos;
	std::vector<double> vel;
	std::vector<double> eff;

	//Vales to send to the bioloid:
	std::vector<double> des_pos;
	std::vector<double> des_vel;
	std::vector<double> des_eff;
	
	//Vales recieved from the bioloid:
	std::vector<double> pos_raw;
	std::vector<double> vel_raw;
	std::vector<double> eff_raw;
	
	int counter;
	bool change;
	bool test;
	bool userInteraction;
	
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool saveConfigurationToFile(std::string filepath);
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	void jointsRawCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);
public slots:
	void updateJoints();
	void testJoints();
signals:
	void jointUpdate();
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Subscriber chatter_sub1;
	ros::Subscriber chatter_sub2;
    QStringListModel logging_model;
    
	template <typename T> void getParamVector (ros::NodeHandle n, const std::string Var, std::vector<T>* const Vec);
};

}  // namespace bioloid_gui

#endif /* bioloid_gui_QNODE_HPP_ */
