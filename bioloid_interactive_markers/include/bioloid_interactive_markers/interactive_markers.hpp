/*
 *      Author: Donato Sciarra
 */

#ifndef INTERACTIVE_MARKERS_HPP_
#define INTERACTIVE_MARKERS_HPP_

#include "interactive_markers/interactive_marker_server.h"
#include "bioloid_interactive_markers/types.h"

#include "visualization_msgs/InteractiveMarker.h"
#include "sensor_msgs/JointState.h"

#define L_arm 1
#define R_arm 2
#define L_leg 3
#define R_leg 4

typedef boost::function< void ( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& ) > Feedback_cb;

class BioloidInteractiveMarkers {

public:
	BioloidInteractiveMarkers();
	virtual ~BioloidInteractiveMarkers();


	void robotStateCallback(const sensor_msgs::JointState::ConstPtr msg);

	/*
	 * Callback functions for processing the information of each Bioloid's marker
	 */
	void processLeftArm(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
	void processRightArm(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback);
	void processLeftLeg(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback);
	void processRightLeg(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback);

private:

	ros::NodeHandle nh_;
//	ros::Subscriber robot_state_;
	ros::Publisher joint_state_pub_;

//	sensor_msgs::JointState::ConstPtr robotState_;
	MarkerPose marker_pose_;

	//Interactive marker server
	interactive_markers::InteractiveMarkerServer server_;

	//Bioloid's markers
	visualization_msgs::InteractiveMarker left_arm;
	visualization_msgs::InteractiveMarker right_arm;
	visualization_msgs::InteractiveMarker left_leg;
	visualization_msgs::InteractiveMarker right_leg;

	void createControl(visualization_msgs::InteractiveMarkerControl& control, double w, double x, double y,
			double z, visualization_msgs::InteractiveMarker& marker, std::string const& move, std::string const& rotate);

	void createMarker(visualization_msgs::InteractiveMarker& marker, double x, double y, double z,
			std::string const& name, std::string const& description, Feedback_cb feedback_cb, double scale = 0.1);

	void updateMarkerPose(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback, Pose *pose_ptr);
	bool attemptModification(visualization_msgs::InteractiveMarkerFeedbackConstPtr const& feedback, std::string part_name, Pose *pose_ptr);

};


#endif /* INTERACTIVE_MARKERS_HPP_ */
