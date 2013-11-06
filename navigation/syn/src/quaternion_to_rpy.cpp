#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "auv_msgs/Orientation.h"
#include <cmath>

ros::Publisher pubEkf;
ros::Publisher pubImu;
ros::Publisher pubVo;

void IMUDataCallBack(const sensor_msgs::Imu &msg){
	float w = msg.orientation.w;
	float x = msg.orientation.x;
	float y = msg.orientation.y;
	float z = msg.orientation.z;
	float yaw,pitch,roll;
//
	auv_msgs::Orientation currentmsg;
//
	float q0 = x;
	float q1 =y;
	float q2 = z;
	float q3 =w;


	yaw = atan2(2 *(q0*q1 + q3*q2) ,(q3*q3 - q2*q2 - q1*q1 + q0*q0)) * 180 / M_PI;
	pitch = asin(-2* (q1*q2 + q0*q3) /(q3*q3 + q2*q2 - q1*q1 - q0*q0)) * 180/ M_PI;
	roll = atan2( 2* (q1*q2 + q0*q3) , (q3*q3 + q2*q2 - q1*q1 -q0*q0)) * 180/ M_PI;

	currentmsg.yaw=yaw;
	currentmsg.pitch=pitch;
	currentmsg.roll=roll;

	pubImu.publish(currentmsg);

}

void VoDataCallBack(const nav_msgs::Odometry &msg2){
	geometry_msgs::Pose msg = msg2.pose.pose;
	float w = msg.orientation.w;
	float x = msg.orientation.x;
	float y = msg.orientation.y;
	float z = msg.orientation.z;
	float yaw,pitch,roll;
//
	auv_msgs::Orientation currentmsg;
//
	float q0 = x;
	float q1 =y;
	float q2 = z;
	float q3 =w;


	yaw = atan2(2 *(q0*q1 + q3*q2) ,(q3*q3 - q2*q2 - q1*q1 + q0*q0)) * 180 / M_PI;
	pitch = asin(-2* (q1*q2 + q0*q3) /(q3*q3 + q2*q2 - q1*q1 - q0*q0)) * 180/ M_PI;
	roll = atan2( 2* (q1*q2 + q0*q3) , (q3*q3 + q2*q2 - q1*q1 -q0*q0)) * 180/ M_PI;

	currentmsg.yaw=yaw;
	currentmsg.pitch=pitch;
	currentmsg.roll=roll;

	pubVo.publish(currentmsg);

}
void IMUDataEkfCallBack(const geometry_msgs::PoseWithCovarianceStamped &msg2){
	geometry_msgs::Pose msg = msg2.pose.pose;
	float w = msg.orientation.w;
	float x = msg.orientation.x;
	float y = msg.orientation.y;
	float z = msg.orientation.z;
	float yaw,pitch,roll;
//
	auv_msgs::Orientation currentmsg;
//
	float q0 = x;
	float q1 =y;
	float q2 = z;
	float q3 =w;


	yaw = atan2(2 *(q0*q1 + q3*q2) ,(q3*q3 - q2*q2 - q1*q1 + q0*q0)) * 180 / M_PI;
	pitch = asin(-2* (q1*q2 + q0*q3) /(q3*q3 + q2*q2 - q1*q1 - q0*q0)) * 180/ M_PI;
	roll = atan2( 2* (q1*q2 + q0*q3) , (q3*q3 + q2*q2 - q1*q1 -q0*q0)) * 180/ M_PI;

	currentmsg.yaw=yaw;
	currentmsg.pitch=pitch;
	currentmsg.roll=roll;

	pubEkf.publish(currentmsg);

}


int main(int argc,char **argv){
	ros::init(argc,argv,"quaternion_to_rpy");
	ros::NodeHandle n;

	ros::Subscriber subEkf=n.subscribe("robot_pose_ekf/odom_combined",1,IMUDataEkfCallBack);
	ros::Subscriber subSim=n.subscribe("/imu_data",1,IMUDataCallBack);
	ros::Subscriber subVo=n.subscribe("/vo",1,VoDataCallBack);

	pubEkf= n.advertise<auv_msgs::Orientation>("imu/data_ekf",1);
	pubVo= n.advertise<auv_msgs::Orientation>("imu/data_vo",1);
	pubImu= n.advertise<auv_msgs::Orientation>("imu/data_imu",1);

	ros::spin();		
	return 0;

}

