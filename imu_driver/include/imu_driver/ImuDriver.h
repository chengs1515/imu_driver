#pragma once

#include "Newton2Parase.h"
#include <string>
#include <ros/ros.h>

namespace imu_driver {

class ImuDriver
{
public:

    ImuDriver(ros::NodeHandle& nh);
    bool start();

private:
    
    ros::Publisher imuPublisher_, posePublisher_;
    ros::NodeHandle& nh_;
    std::string imuTopic_,poseTopic_;
    std::string frame_;
    imu_driver::Newton2Parser* par_;
};

}