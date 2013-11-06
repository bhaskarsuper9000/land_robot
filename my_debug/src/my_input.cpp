#include "ros/ros.h"
#include "std_msgs/String.h"
#include<my_debug/input.h>
#include<iostream>
#include <sstream>
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");            // "talker" is the name of the node
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<my_debug::input>("chatter", 1000);
 	ros::Rate loop_rate(10);
  	int count = 0;
  	while (ros::ok())
  	{
  		my_debug::input msg;
		msg.x = 10;
		std::stringstream ss;
		ROS_INFO("Hello World! %d %f", count,msg.x);
 		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	    ++count;
  	}
 	return 0;
}
