#ifndef IMU_H
#define IMU_H

struct Quaternion{
	float x;
	float y;
	float z;
	float w;
};
struct Vector3{
	float x;
	float y;
	float z;
};

class IMU{
		//use the Imu where it publishes quaternion,linear acceleration , angular rates.
	private:
				
	public:
		Quaternion orientation;
		Vector3 linearAcceleration; //x,y,z;
		Vector3 angularVelocity; //x,y,z;
		

		IMU();
		void print();
		void setParameters(float* value);
		float calculateOrienation();	
		void operator=(IMU i);
};

#endif

