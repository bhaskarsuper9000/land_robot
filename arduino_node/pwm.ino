/*this node will assign a pwm to the motors as it receive a message from the publisher on topic "/pwm" */

#include <ros.h>
#include <std_msgs/Char.h>

ros::NodeHandle  nh;
void messageCb( const std_msgs::Char& pwm){
   if(int(pwm.data)==32) {       //Break
      analogWrite(5,127);
      analogWrite(6,127);
    }
 
    else if(int(pwm.data)==65) {       //forward  (up key)
      analogWrite(5, 142);
      analogWrite(6, 142);
      //analogWrite(10, 255);
      //analogWrite(11, 255);
    }
    else if(int(pwm.data)==66){        //Backward (down key)
      analogWrite(5, 112);
      analogWrite(6, 112);
      //analogWrite(10, 0);
      //analogWrite(11, 0);
   }
  
    else if(int(pwm.data)==67){        //right (right key)
      analogWrite(5, 142);
      analogWrite(6, 112);
      //analogWrite(10, 127);
      //analogWrite(11, 127);
   }
   else if(int(pwm.data)==68){        //left (left key)
    analogWrite(5, 112);
      analogWrite(6, 142);
      //analogWrite(10, 255);
      //analogWrite(11, 255);
   }
}

ros::Subscriber<std_msgs::Char> sub("pwm", &messageCb );


void setup()
{ 
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  analogWrite(5, 127);
  analogWrite(6, 127);
  
  
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

