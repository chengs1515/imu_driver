#pragma once

#include <ros/ros.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <proj_api.h>
#include <tf/transform_broadcaster.h>

namespace imu_driver{

class Newton2Parser
{
public:
    Newton2Parser(ros::Publisher& imu_pub,ros::Publisher& pose_pub,std::string& frame);
    bool is_open_ = false;
    bool readPort();
private:
    void connect();
    void openDevice();
    bool configPort();
    void prase(std::string message);
    void prepareMessage();

    char *device_name_;
    speed_t baud_rate_;
    static constexpr size_t BUFFER_SIZE = 2048;
    uint8_t buffer_[BUFFER_SIZE] = {0};
    int fd_;
    const uint8_t* data_;
    const uint8_t* data_end_;
    size_t header_length_;
    size_t total_length_;
    std::vector<uint8_t> data_buffer_;
    double gyro_scale_ = 1.0850694444444445e-05;
    double accel_scale_ = 1.52587890625e-04;
    double sampling_rate_hz_ = 100.0;
    float imu_span_ = 1.0/sampling_rate_hz_;
    float time_pre_ = -1;
    projPJ wgs84pj_source_;
    projPJ utm_target_;
    tf::TransformBroadcaster br_;
    double roll_ = 0;
    double pitch_ = 0;
    double yaw_ = 0;

    ros::Publisher imu_pub_,pose_pub_;
    std::string frame_;
    sensor_msgs::Imu msg_imu_;
    geometry_msgs::Pose msg_pose_;
    uint32_t seq_ = 0;
};
}




