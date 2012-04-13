This is the interface designed by Brennand Pierce for controlling the bioloid in ROS.

To configure the bioloid, please configure joint offsets and max, min angles and joint names and servo numbers using the para launch files.  The main loop reads this file and then uses it to control the bioloid. So if you want to only control one servo just have one joint in the para launch file.  please see the readme.txt in the launch folder.

To communicate to the bioloid please us sensor_msgs/JointSate.h, to make it easier you can read the values in your own control program from the parameter server to get the number of joints and the joint names.

Please see bioloid_control_example package in this stack for a simple example of using this interface.
