#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int16.h"


int main(int argc, char **argv){
  
  ros::init(argc, argv, "rinput");
  ros::NodeHandle n;
  ros::Publisher pwm_pub = n.advertise<std_msgs::Int16>("rpwm", 1000);

ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
	 
    
    std_msgs::Int16 msg; 	
    int rpwm;
    std::cin>>rpwm;
		msg.data=rpwm;
        pwm_pub.publish(msg);
        std::cout<<msg.data<<" ";

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
