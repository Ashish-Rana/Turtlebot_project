// Author : Ashish Rana
// Date   : 1/11/2018

#include "behaviour.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"


// Route subscriber and service to their callback functions
// creates 2 publishers,
Behaviour::Behaviour(): m_ranges(m_cone, 0), m_fsm(0), m_distance(0)
{

	m_pub1 = m_n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 5);
	m_sub = m_n.subscribe("/scan", 5, &Behaviour::scanCallback, this);
	m_srv = m_n.advertiseService("enable", &Behaviour::getData, this); 
	m_pub2 = m_n.advertise<geometry_msgs::Vector3>("displacement", 5);
}

// Slices the ranges[] array and passes values to a vector.
// Filters all nan values and converts them to zero
void Behaviour::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for(int i = 0; i < m_ranges.size(); ++i)
	{
		double temp = msg->ranges[i+m_offset];
		m_ranges[i] = temp == temp? temp : 0;
	}
}

// Gets bool value from service, response is giberish
bool Behaviour::getData(seeker::enable::Request &req,
						seeker::enable::Response &res)
{

	m_chk = static_cast<bool>(req.data);
	std::cout << "Service call received!\n";
	return "Request Received";
}

// Publishes measured distance at the center point
void Behaviour::sendDisp()
{
	geometry_msgs::Vector3 msg;
	m_distance = m_ranges.at(m_cone/2);
	msg.x = m_distance;
	m_pub2.publish(msg);
}

// Finite state machine, this drives the robot
void Behaviour::fsm()
{


	ros::Rate rate(30);
	std::cout << "Entering Finite State Machine\n";
	while(ros::ok())
	{
		sendDisp();
		if(m_chk == true)
		{
			
			//printInfo(); // For debuging
			switch(m_fsm)
			{
				case 0:
					seekObject();
					break;

				case 1:
					moveForward();
					break;

				default:
					stopMoving();
					break;
			}
		}

	ros::spinOnce();
	rate.sleep();
	}
}


// Rotates untill an object is detected
void Behaviour::seekObject()
{
	std::cout << "\nSearching...";
	geometry_msgs::Twist msg;

	msg.angular.z = m_omega;
	m_pub1.publish(msg);	
	
	if(!lockedIn())
		m_fsm = 0;
	else
		m_fsm = 1;
}

// Moves robot forward untill kinect faces away and starts record zero
void Behaviour::moveForward()
{
	std::cout << "\nMoving Forward...";
	geometry_msgs::Twist msg;
	msg.linear.x = m_v;
	m_pub1.publish(msg);

	if(!lockedIn() && m_distance > 0.5)
		m_fsm = 0;
	else
		m_fsm = 1;

}

// Stops robot motion, primarily for debuging and experimentation
void Behaviour::stopMoving()
{
	std::cout << "\nSomething went wrong";
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	m_pub1.publish(msg);

}

// Checks whether a object is being detected and is within a certain angle range
bool Behaviour::lockedIn()
{
	int begin = m_ranges.at((m_cone/2) - m_lockin_range);
	for(int i = begin; i < begin+(m_lockin_range*2); ++i)
	{
		if(m_ranges.at(i) == 0)
		{
			std::cout << "Target lost!\n"; 
			return false;
		}
		else
		{
			std::cout << "Locked in on target!\n";
			return true;
		}
	}
}

// Function for debugging, prints scan values from vector
void Behaviour::printInfo()
{
		std::cout<< "[ ";
		for(auto i = m_ranges.begin(); i != m_ranges.end(); ++i)
			std::cout << *i << " ";
		std::cout << " ]\n";
}
