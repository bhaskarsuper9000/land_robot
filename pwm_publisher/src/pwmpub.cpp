#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "setup.h"
#include <sstream>
#include <string>
#include <unistd.h>             /*This both header file is used for the*/ 
#include <termios.h>            /*continous input(without enter or space)*/


int main(int argc, char **argv){
  
ros::init(argc, argv, "input");
ros::NodeHandle n;
ros::Publisher pwm_pub = n.advertise<std_msgs::Char>("pwm", 100);
ros::Rate loop_rate(5);

	//int count = 0;
	setup();                        /* To change the mode of input of terminal to take continous input without waiting for enter*/
	while (ros::ok()){
	 
    
    std_msgs::Char msg; 	
    unsigned char pwm, pwm1, pwm2;
                                    
    pwm=getchar();                     
		if(pwm==27){              
			pwm1=getchar();
			pwm2=getchar();
			pwm=pwm2;        
	}
		msg.data=pwm;
        pwm_pub.publish(msg);
        std::cout<<msg.data<<" ";

    ros::spinOnce();

    //loop_rate.sleep();
    //++count;
  }
                                                         
	tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);  /* restore the former settings */


  return 0;
}
