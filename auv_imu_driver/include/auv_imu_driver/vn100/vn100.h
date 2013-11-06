#include <SerialStream.h>
#include <SerialPort.h>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Empty.h>
#include <cstring>
#include <ros/ros.h>
#include <string.h>
#include <std_msgs/Bool.h>

#include <auv_imu_driver/vn100/imu.h>

//#define DEBUG 1
class VN100{
  private:
    ros::NodeHandle _nh;
    bool _calibrateState;
    SerialPort _mySerialPort;
    ros::Publisher _calibratePub;
    ros::Publisher _imuPub;
    ros::ServiceServer _calibrateService;  
	bool _calibrateImu(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  public:
    VN100(std::string port, std::string type);
    void publishImuData();
    IMU getValues(std::string s);
	std::string imuType;
};
