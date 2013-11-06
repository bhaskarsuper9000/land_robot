/* quaternion_to_rpy node has been implemented in this file
 * this node subscribes to /imu/data (which publishes the 
 * quaternion values) and publishes on /imu/data_rpy (which 
 * contains the roll pitch yaw angles in degrees)
 */

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include "auv_msgs/IMUFilter.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "auv_msgs/Orientation.h"
#include <cmath>


class QtoRpy {
	public:
		QtoRpy();
		void spin();
	private:
		ros::NodeHandle _nh;
		ros::ServiceServer _srvFilter;

		ros::Publisher _pubRuggedRpy;
		ros::Publisher _pubBoardRpy;
		ros::Publisher _pubFinalRpy;
		ros::Publisher _pubFinalRawRpy;

		ros::Subscriber _subRuggedQuaternion;
		ros::Subscriber _subBoardQuaternion;

		sensor_msgs::Imu _ruggedImuMsg;
		sensor_msgs::Imu _boardImuMsg;

		auv_msgs::Orientation _ruggedRpyMsg;
		auv_msgs::Orientation _boardRpyMsg;
		auv_msgs::Orientation _finalRpyMsg;
		auv_msgs::Orientation _finalRawRpyMsg;

		bool _enableFilter;
		bool _callBackFilter(auv_msgs::IMUFilter::Request &req, auv_msgs::IMUFilter::Response &res);
		void _callBackRuggedIMUData(const sensor_msgs::Imu &msg);
		void _callBackBoardIMUData(const sensor_msgs::Imu &msg);
		void _processIMUData(void);
};

QtoRpy::QtoRpy() {
	_subRuggedQuaternion = _nh.subscribe("/imu/rugged/data",1000,&QtoRpy::_callBackRuggedIMUData,this);
	_subBoardQuaternion = _nh.subscribe("/imu/board/data",1000,&QtoRpy::_callBackBoardIMUData,this);
	_pubRuggedRpy = _nh.advertise<auv_msgs::Orientation>("/imu/rugged/data_rpy",1000);
	_pubBoardRpy = _nh.advertise<auv_msgs::Orientation>("/imu/board/data_rpy",1000);
	_pubFinalRpy = _nh.advertise<auv_msgs::Orientation>("/imu/data_rpy",1000);
	_pubFinalRawRpy = _nh.advertise<auv_msgs::Orientation>("/imu/raw_data_rpy",1000);

	_srvFilter = _nh.advertiseService("/imu/peak_filter",&QtoRpy::_callBackFilter,this);
	_enableFilter = false;
}

bool QtoRpy::_callBackFilter(auv_msgs::IMUFilter::Request& req, auv_msgs::IMUFilter::Response& res) {
	
	_enableFilter = req.command;
	if(_enableFilter == true) {
		ROS_INFO("IMU: changing filter status to :: True");
	}
	else {
		ROS_INFO("IMU: changing filter status to :: False");
	}

	return true;
}

void QtoRpy::_callBackRuggedIMUData(const sensor_msgs::Imu &msg){
	_ruggedImuMsg = msg;
}

void QtoRpy::_callBackBoardIMUData(const sensor_msgs::Imu &msg){
	_boardImuMsg = msg;
	_processIMUData();
}

void QtoRpy::_processIMUData() {
	static bool firstCallBack = true; // whether this function is being called for the first time

	int rugged_yaw,rugged_pitch,rugged_roll;
	int board_yaw,board_pitch,board_roll;
	int final_yaw,final_pitch,final_roll;
	static int final_prev_yaw, final_prev_pitch, final_prev_roll;

	float rugged_q0 = _ruggedImuMsg.orientation.x;
	float rugged_q1 = _ruggedImuMsg.orientation.y;
	float rugged_q2 = _ruggedImuMsg.orientation.z;
	float rugged_q3 = _ruggedImuMsg.orientation.w;

	float board_q0 = _boardImuMsg.orientation.x;
	float board_q1 = _boardImuMsg.orientation.y;
	float board_q2 = _boardImuMsg.orientation.z;
	float board_q3 = _boardImuMsg.orientation.w;

	rugged_yaw     = (int) (180*atan2(2 *(rugged_q0*rugged_q1 + rugged_q3*rugged_q2) ,(rugged_q3*rugged_q3 - rugged_q2*rugged_q2 - rugged_q1*rugged_q1 + rugged_q0*rugged_q0))/3.14);
	rugged_roll    = (int) (180*asin(-2* (rugged_q0*rugged_q2 - rugged_q1*rugged_q3))/3.14);
	rugged_pitch   = (int) (180*atan2( 2* (rugged_q1*rugged_q2 + rugged_q0*rugged_q3) , (rugged_q3*rugged_q3 + rugged_q2*rugged_q2 - rugged_q1*rugged_q1 -rugged_q0*rugged_q0))/3.14);

	_ruggedRpyMsg.yaw = rugged_yaw;
	_ruggedRpyMsg.pitch = rugged_pitch;
	_ruggedRpyMsg.roll = rugged_roll;

	board_yaw     = (int) (180*atan2(2 *(board_q0*board_q1 + board_q3*board_q2) ,(board_q3*board_q3 - board_q2*board_q2 - board_q1*board_q1 + board_q0*board_q0))/3.14);
	board_roll    = (int) (180*asin(-2* (board_q0*board_q2 - board_q1*board_q3))/3.14);
	board_pitch   = (int) (180*atan2( 2* (board_q1*board_q2 + board_q0*board_q3) , (board_q3*board_q3 + board_q2*board_q2 - board_q1*board_q1 -board_q0*board_q0))/3.14);

	_boardRpyMsg.yaw = board_yaw;
	_boardRpyMsg.pitch = board_pitch;
	_boardRpyMsg.roll = board_roll;
	
	final_yaw = (_ruggedRpyMsg.yaw + _boardRpyMsg.yaw)/2;
	final_pitch = (_ruggedRpyMsg.pitch + _boardRpyMsg.pitch)/2;
	final_roll = (_ruggedRpyMsg.roll + _boardRpyMsg.roll)/2;

	_finalRawRpyMsg.yaw = final_yaw;
	_finalRawRpyMsg.pitch = final_pitch;
	_finalRawRpyMsg.roll = final_roll;

	/* filter the RPY data to remove random noisy peask */

	if (_enableFilter) {
		if(!firstCallBack) {
			if(((final_prev_yaw - final_yaw) > 30) || (final_prev_yaw - final_yaw) < -30) {
				final_yaw = final_prev_yaw;
			}
			if(((final_prev_pitch - final_pitch) > 30) || (final_prev_pitch - final_pitch) < -30) {
				final_pitch = final_prev_pitch;
			}
			if(((final_prev_roll - final_roll) > 30) || (final_prev_roll - final_roll) < -30) {
				final_roll = final_prev_roll;
			}
		}
		final_prev_yaw = final_yaw;
		final_prev_pitch = final_pitch;
		final_prev_roll = final_roll;
	}
	else {
		final_yaw = 0;
		final_prev_yaw = 0;
	}



	_finalRpyMsg.yaw = final_yaw;
	_finalRpyMsg.pitch = final_pitch;
	_finalRpyMsg.roll = final_roll;

	_pubRuggedRpy.publish(_ruggedRpyMsg);
	_pubBoardRpy.publish(_boardRpyMsg);
	_pubFinalRpy.publish(_finalRpyMsg);
	_pubFinalRawRpy.publish(_finalRawRpyMsg);

	firstCallBack = false;
}

void QtoRpy::spin() {
	ros::spin();
}



int main(int argc,char **argv){
	ros::init(argc,argv,"quaternion_to_rpy");
	QtoRpy theQtoRpy;
	theQtoRpy.spin();
	return 0;
}
