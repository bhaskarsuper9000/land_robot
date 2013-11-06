
#include <auv_imu_driver/vn100/vn100.h>


using namespace LibSerial;
using namespace std;

VN100::VN100(std::string port, std::string type) : _mySerialPort(port)
{
	imuType = type;
	_calibrateState=false;
	_mySerialPort.Open(SerialPort::BAUD_9600 , SerialPort::CHAR_SIZE_8) ;
	std::string dataTopicName = "/imu/" + imuType + "/data";
	std::string calTopicName = "/imu/" + imuType + "/is_calibrated";
	std::string calServiceName = "/imu/" + imuType + "/calibrate";
	_imuPub =   _nh.advertise<sensor_msgs::Imu>(dataTopicName.c_str(),1);
	_calibrateService = _nh.advertiseService(calServiceName.c_str(), &VN100::_calibrateImu, this);
	_calibratePub = _nh.advertise<std_msgs::Bool>(calTopicName.c_str(),1);
	//std::cout<<_mySerialPort.IsOpen()<<std::endl;

}

void VN100::publishImuData()
{
	ros::Rate loop_rate(10);
	if(_mySerialPort.IsOpen()){
		while(ros::ok()){
			std_msgs::Bool calibrate_msg;
			std::string data = _mySerialPort.ReadLine(0);
			int length = data.length();
#ifdef DEBUG
			std::cout<<data<<std::endl; 
#endif
			if(length<108){
				continue;
			}
			IMU currentImu = getValues(data);

			sensor_msgs::Imu currentmsg ;

			currentmsg.orientation.x=currentImu.orientation.x;
			currentmsg.orientation.y=currentImu.orientation.y;
			currentmsg.orientation.z=currentImu.orientation.z;
			currentmsg.orientation.w=currentImu.orientation.w;

			currentmsg.angular_velocity.x=currentImu.angularVelocity.x;
			currentmsg.angular_velocity.y=currentImu.angularVelocity.y;
			currentmsg.angular_velocity.z=currentImu.angularVelocity.z;

			currentmsg.linear_acceleration.x=currentImu.linearAcceleration.x;
			currentmsg.linear_acceleration.y=currentImu.linearAcceleration.y;
			currentmsg.linear_acceleration.z=currentImu.linearAcceleration.z;
#ifdef DEBUG
			currentImu.print();
#endif
			calibrate_msg.data = _calibrateState;
			_calibratePub.publish(calibrate_msg);
			_imuPub.publish(currentmsg);
			ros::spinOnce();
		}
		_mySerialPort.Close();
	}else{
		std::cout<<"The port is not open"<<std::endl;
	}
}

IMU VN100 ::getValues(string s){
	string str=s;
	float values[10];

	for(int i=0;i<str.length();i++){
		if(str[i]=='*'){
			str[i]=',';
		}
	}
	string word;
	stringstream stream(str);
	int count =0;
	while( getline(stream, word, ',') ){
		if(count>=1 && count<=10){
			values[count-1] = atof(word.c_str());
		}
		count++;
	}
	IMU imu;
	imu.setParameters(values);
	return imu;
}

bool VN100:: _calibrateImu(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//ROS_INFO("Calibrating IMU");
	if(_mySerialPort.IsOpen()){
		std::string output_buffer = "$VNTAR*5F\n";
		//  _mySerialPort.Write("$VNTAR*5F\n");//this command to do the tare operation.
		_mySerialPort.Write( output_buffer) ;
		std::string data = _mySerialPort.ReadLine(0);
		//ROS_INFO("Read a line from port %s",data.c_str());
		char output_char[data.length()];
		strcpy(output_char,data.c_str());
		//std::cout<<output_char<<std::endl;
		/*  if(tare=="$VNSGB*4E"){
			msg.data =true;
			}
			else {
			msg.data=false;
			} */

		_calibrateState=  true;
		ROS_INFO("VN100: %s : IMU calibrated",imuType.c_str());   
	}
	else{
		ROS_INFO("VN100: %s : could not caliberate IMU",imuType.c_str());     
		_calibrateState=false;
	}
	return true;
}

