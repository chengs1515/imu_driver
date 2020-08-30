#include "imu_driver/ImuDriver.h"
#include "imu_driver/Newton2Parase.h"

namespace imu_driver{

ImuDriver::ImuDriver(ros::NodeHandle& nh):nh_(nh)
{
    imuTopic_ = "/raw_imu";
    poseTopic_ = "/gnss_pose";
    frame_ = "Gnss";

    imuPublisher_ = nh_.advertise<sensor_msgs::Imu>(imuTopic_,2);
    posePublisher_ = nh_.advertise<geometry_msgs::Pose>(poseTopic_,2);

    par_ = new imu_driver::Newton2Parser(imuPublisher_,posePublisher_,frame_);
}

bool ImuDriver::start()
{
    if(!par_->readPort())
        return false;
    return true;
}

}