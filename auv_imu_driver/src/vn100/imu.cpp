#include <iostream>
#include <auv_imu_driver/vn100/imu.h>
void IMU:: setParameters(float* values){
	orientation.x=values[0];
	orientation.y=values[1];
	orientation.z=values[2];
	orientation.w=values[3];
	linearAcceleration.x=values[4];
	linearAcceleration.y=values[5];
	linearAcceleration.z=values[6];
	angularVelocity.x=values[7];
	angularVelocity.y=values[8];
	angularVelocity.z=values[9];
}

IMU::IMU(){
	
	

}

void IMU :: print(){
	std::cout<<"Quaternian-orientation :" <<std::endl;
	
	std::cout<<"\t"<< orientation.x<<std::endl;
	std::cout<<"\t"<< orientation.y<<std::endl;
	std::cout<<"\t"<< orientation.z<<std::endl;
	std::cout<<"\t"<< orientation.w<<std::endl;
	
	
	std::cout<<"Linear Acceleration :" <<std::endl;	
	
	std::cout<<"\t"<< linearAcceleration.x<<std::endl;
	std::cout<<"\t"<< linearAcceleration.y<<std::endl;
	std::cout<<"\t"<< linearAcceleration.z<<std::endl;
	
	std::cout<<"Angular Acceleration :" <<std::endl;
		
	std::cout<<"\t"<< angularVelocity.x<<std::endl;
	std::cout<<"\t"<< angularVelocity.y<<std::endl;
	std::cout<<"\t"<< angularVelocity.z<<std::endl;

}
void IMU::operator=(IMU imu){

			orientation.x=imu.orientation.x;
			orientation.y=imu.orientation.y;
			orientation.z=imu.orientation.z;
			orientation.w=imu.orientation.w;
	

			linearAcceleration.x=imu.linearAcceleration.x;
			linearAcceleration.y=imu.linearAcceleration.y;
			linearAcceleration.z=imu.linearAcceleration.z;

			angularVelocity.x=imu.angularVelocity.x;
			angularVelocity.y=imu.angularVelocity.y;
			angularVelocity.z=imu.angularVelocity.z;
				
}

//this function needs changes
float IMU::calculateOrienation(){
	return orientation.x;

}
