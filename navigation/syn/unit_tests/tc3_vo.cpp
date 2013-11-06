#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <time.h>

#define NOISE_MAX (10.0*M_PI/180)

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("vo", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 1.0*M_PI/180;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);

  srand (time(NULL));	//for random noise

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //Add noise
    //th += rand() % 10 + 1; 

    //Will generate a random number between 0 and X (a float value)
    //float noise = (float)rand()/((float)RAND_MAX/NOISE_MAX);
    float noise = NOISE_MAX;
    th += noise;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "vo";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "vo";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
 
    for(int i=0; i< 36; i++)    
	odom.pose.covariance[i] = 1e-12;
    
    
    for(int i=0; i< 6; i++)
	for(int j=0; j<6; j++)	
	    if(i==j)
		odom.twist.covariance[6*i + j] = NOISE_MAX;
	    else
		odom.twist.covariance[6*i + j] = 1e-12;

//    odom.pose.covariance = {2.9,1,2,2,1,3,
//				1,4,2,2,1,1,
//				2,5,2,2,2,2,
//				2,2,1,3,4,23,
//				2,6,2,1,3,1,
//				2,4,1,2,1,3};
//    odom.twist.covariance = {2.9,1,2,2,1,3,
//				1,2,2,2,1,1,
//				2,2,2,2,2,2,
//				2,2,1,3,4,23,
//				2,6,2,1,3,1,
//				2,4,1,2,1,3};

    //publish the message
    odom_pub.publish(odom);

    //remove noise :P
    th -= noise;

    last_time = current_time;
    r.sleep();
  }
}
