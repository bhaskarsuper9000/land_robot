#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher");

  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 50);
  tf::TransformBroadcaster imu_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 1.0*M_PI/180;

  double lax = 0.0;
  double lay = 0.0;
  double laz = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute dometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    double delta_ax = 0.0;
    double delta_ay = 0.0;
    double delta_az = 0.0;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    lax += delta_ax;
    lay += delta_ay;
    laz += delta_az;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped imu_trans;
    imu_trans.header.stamp = current_time;
    imu_trans.header.frame_id = "imu";
    imu_trans.child_frame_id = "base_footprint";

    imu_trans.transform.translation.x = 0.0;
    imu_trans.transform.translation.y = 0.0;
    imu_trans.transform.translation.z = 0.0;
    imu_trans.transform.rotation = imu_quat;

    //send the transform
    imu_broadcaster.sendTransform(imu_trans);

    //next, we'll publish the imu message over ROS
    sensor_msgs::Imu imu;
    imu.header.stamp = current_time;
    imu.header.frame_id = "imu";

    //set the orientation
    imu.orientation = imu_quat;

    //set the angular velocity
    imu.angular_velocity.x = vx;
    imu.angular_velocity.y = vy;
    imu.angular_velocity.z = vth;

    //set the linear acceleration
    imu.angular_velocity.x = lax;
    imu.angular_velocity.y = lay;
    imu.angular_velocity.z = laz;


    for(int i=0; i < 3; i++)
	for(int j=0; j<3; j++)
	    if(i==j)
		imu.orientation_covariance[3*i+j] = M_PI/180*1;
	    else
		imu.orientation_covariance[3*i+j] = 1e-12;
	    
    
    for(int i=0; i < 3; i++)
	for(int j=0; j<3; j++)
//	    if(i==j)
		imu.angular_velocity_covariance[3*i+j] = 1e-12;

    for(int i=0; i < 3; i++)
	for(int j=0; j<3; j++)
//	    if(i==j)
		imu.linear_acceleration_covariance[3*i+j] = 1e-12;
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
    imu_pub.publish(imu);

    last_time = current_time;
    r.sleep();
  }
}
