// Author : Ashish Rana
// Date   : 1/11/2018

#ifndef _BEHAVIOUR_H
#define _BEHAVIOUR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "seeker/enable.h"
#include <vector>


class Behaviour
{
private:
	//Ros method access variables
	ros::NodeHandle m_n;
	ros::Publisher m_pub1;
	ros::Publisher m_pub2;
	ros::Subscriber m_sub;
	ros::ServiceServer m_srv;

	// Callbacks, Services
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	bool getData(seeker::enable::Request &req, seeker::enable::Response &res);
	void sendDisp();

public:
	// Tunable Parameters
	double m_omega = 0.2;						// Rotation Speed
	const double m_v = 0.2;						// Movement Speed
	const double m_multiplier = 1.95;			// Mathematical expression variable
	const int m_cone = 30;						// 30 Degree cone to lock in on the ball
	const int m_offset= 359-(m_cone/2);					// Stopping point for lock in cone.
	const int m_lockin_range = 10;				// Lock in range offset from center point of kinect


	// Empty and uniform variables/vectors
	double m_distance;						// store max range to check if any objects are in range
	int m_fsm;									// Finite state machine key
	bool m_chk;									// Service toggle
	std::vector<double> m_ranges;				// store ranges in vector because vectors are nicer than old school arrays

	// Constructor & Destructor
	Behaviour();
	~Behaviour() = default;

	// Member Functions
	void fsm();
	void seekObject();				
	void moveForward();
	void stopMoving();
	bool lockedIn();
	void printInfo();

};

#endif
