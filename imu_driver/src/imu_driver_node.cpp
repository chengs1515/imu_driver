#include <ros/ros.h>
#include "imu_driver/ImuDriver.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"imu_driver_node");
    ros::NodeHandle nh("~");
    
    imu_driver::ImuDriver imuDriver(nh);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(!imuDriver.start())
            break;
        ros::spinOnce();
    }

    return 0;
}