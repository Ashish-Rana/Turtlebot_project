
// Author : Ashish Rana
// Date   : 1/11/2018

#include "behaviour.h"
#include "seeker/enable.h"


int main(int argc, char** argv)
{

	ros::init(argc, argv, "seeker_node");

	Behaviour turtlebot;
	turtlebot.fsm();

}
