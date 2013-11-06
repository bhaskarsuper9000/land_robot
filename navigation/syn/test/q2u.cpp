#include <stdio.h>
#include <cmath>

//#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>
//#include <nav_msgs/Odometry.h>

using namespace std;

#define PI 3.1417
void QuaternionToEuler(float x, float y, float z, float w);

int main(int argc, char** argv)
{   
    float x,y,z,w;
//    int n=0;

//    scanf("%d\n",&n);

    //fflush(stdin);

    //std::cin>>"x: ">>"y: ">>y>>"z: ">>z>>"w: ">>w;
//    for(int i=0; i<n; i++){
	//fflush(stdin);
	scanf("x: %f\ny: %f\nz: %f\nw: %f", &x,&y,&z,&w);
        QuaternionToEuler(x,y,z,w);
//    }

//    tf::Quaternion rotation(x, y, z, w);
//    tf::Vector3 vector(1, 1, 1);
//    tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

//    QuaternionToEuler(x,y,z,w);
    return 0;
}

void QuaternionToEuler(float x, float y, float z, float w)
{
    float a,b,c;
    
    double sqw = w*w;    
    double sqx = x*x;    
    double sqy = y*y;    
    double sqz = z*z; 

    a =  (atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw)) * (180.0f/PI));
    b =  (atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw)) * (180.0f/PI));          
    c =  (asin(-2.0 * (x*z - y*w)) * (180.0f/PI));


    printf("%f : %f : %f\n", a, b, c);
}
