#include <auv_imu_driver/vn100/vn100.h>
#include <string>


using namespace LibSerial;
using namespace std;


int main(int argc,char **argv){
	std::string imuType = argv[2];
	std::string nodeName = "vn100_" + imuType;
    ros::init(argc,argv,nodeName.c_str());
    std::string port = argv[1];
    ROS_INFO("Port %s opened",port.c_str());
    VN100 theImu(port,imuType);
    theImu.publishImuData();
    return 0;
}
