#include "imu_driver/novatel_messages.h"
#include <vector>
#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include "tf/transform_datatypes.h"
#include "imu_driver/Newton2Parase.h"
#include <proj_api.h>
//#include "imu_driver/bestpose.h"
//#include <ros/time.h>


using std::cout;
using std::endl;
using std::string;
using namespace std::chrono;
using namespace std::this_thread;

namespace imu_driver{
// device connection

Newton2Parser::Newton2Parser(ros::Publisher& imu_pub,ros::Publisher& pose_pub,std::string& frame)
{
    frame_ = frame;
    imu_pub_ = imu_pub;
    pose_pub_ = pose_pub;
    device_name_ = "/dev/ttyACM0";
    baud_rate_ = B115200;
    const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";
    const char *proj4_text = "+proj=utm +zone=51 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
    utm_target_ = pj_init_plus(proj4_text);

    connect();
}

void Newton2Parser::connect()
{
    if(!is_open_)
    {
        openDevice();
        if(!is_open_)
        {
            cout<<"open device failed because : "<<strerror(errno)<<endl;
            return;
        }
    }
    /*   从apollo上看到这个只是配置命令，只需要要在配置的时候发送一次，并不需要每次启动都发送
    std::vector<std::string> login_commands{
    "SETWHEELPARAMETERS 100 1 1\r\n",
    "UNLOGALL THISPORT\r\n",
    "LOG COM2 GPRMC ONTIME 1.0 0.25\r\n",
    "EVENTOUTCONTROL MARK2 ENABLE POSITIVE 999999990 10\r\n",
    "EVENTOUTCONTROL MARK1 ENABLE POSITIVE 50000000 50000000\r\n",
    "LOG NCOM1 GPGGA ONTIME 1.0\r\n",
    "log bestgnssposb ontime 0.5\r\n",
    "log bestposb ontime 1.0 0.2\r\n",
    "log inspvaxb ontime 1 0.7\r\n",
    "log inspvab ontime 0.01 0\r\n",
    "log corrimudatab ontime 0.01 0\r\n",
    "log RAWIMUB onnew 0 0\r\n",
    "log mark1pvab onnew 0 0\r\n",
    "log rangecmpb ontime 1.0\r\n",
    "log rawephemb onchanged\r\n",
    "log gloephemerisb onchanged\r\n",
    "log bdsephemerisb onchanged\r\n",
    "log imutoantoffsetsb once\r\n",
    "log vehiclebodyrotationb onchanged\r\n",
    "log headingb onchanged\r\n",
    "log rangeb ontime 0.2\r\n",
    "log bdsephemerisb\r\n",
    "log gpsephemb\r\n",
    "log gloephemerisb\r\n",
    "log bdsephemerisb ontime 15\r\n",
    "log gpsephemb ontime 15\r\n",
    "log gloephemerisb ontime 15\r\n"
    };
    int senddata = 0;
    for(const auto& command:login_commands)
    {
        int sendtimes = 0;
        
        while(sendtimes<5)
        {
            sleep_for(milliseconds(20));//不能直接发送，需要有一定的延时，否则会发送不成功
            ssize_t nsent = ::write(fd_,reinterpret_cast<const uint8_t*>(command.data()),command.size());
            if(nsent<0)
            {
                cout<<"sent data failed because: "<<strerror(errno)<<endl;
            }
            else if(nsent>0) break;

            sendtimes++;
        }
        if(sendtimes == 5)
            cout<<"This command : "<<command<<"failed to send"<<endl;
    }
    */
}

void Newton2Parser::openDevice()
{
    fd_ = ::open(device_name_,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd_ == -1)
    {
        cout<<"There is an error in "<<device_name_<<" because: "<<strerror(errno)<<endl;//通过报错知道是什么原因，可能是因为没有权限，也可能是太多文件
        return;
    }
    
    if(!configPort())
    {
        ::close(fd_);
        return;
    }
    is_open_ = true;
}

bool Newton2Parser::configPort()
{
    if(fd_<0)
    {
        return false;
    }

    struct termios options;  // The options for the file descriptor
    if (tcgetattr(fd_, &options) == -1) {
        cout<< "tcgetattr failed."<<endl;;
        return false;
    }
    

    // set up raw mode / no echo / binary
    options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                    ISIG | IEXTEN);  // |ECHOPRT

    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

    #ifdef IUCLC
    options.c_iflag &= (tcflag_t)~IUCLC;
    #endif

    #ifdef PARMRK
    options.c_iflag &= (tcflag_t)~PARMRK;
    #endif

    #ifdef BSD_SOURCE_  // depend glibc
    ::cfsetspeed(&options, baud_rate_);
    #else
    ::cfsetispeed(&options, baud_rate_);
    ::cfsetospeed(&options, baud_rate_);
    #endif

    // setup char len
    options.c_cflag &= (tcflag_t)~CSIZE;

    // eight bits
    options.c_cflag |= CS8;

    // setup stopbits:stopbits_one
    options.c_cflag &= (tcflag_t) ~(CSTOPB);

    // setup parity: parity_none
    options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);

    // setup flow control : flowcontrol_none
    // xonxoff
    #ifdef IXANY
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
    #else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
    #endif

    // rtscts
    #ifdef CRTSCTS
    options.c_cflag &= static_cast<uint64_t>(~(CRTSCTS));
    #elif defined CNEW_RTSCTS
    options.c_cflag &= static_cast<uint64_t>(~(CNEW_RTSCTS));
    #else
    #error "OS Support seems wrong."
    #endif

    // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
    // this basically sets the read call up to be a polling read,
    // but we are using select to ensure there is data available
    // to read before each call, so we should never needlessly poll
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    // activate settings
    ::tcsetattr(fd_, TCSANOW, &options);

    return true;
}

//data reading

bool Newton2Parser::readPort()
{
    ssize_t rdle = ::read(fd_,buffer_,BUFFER_SIZE);
    if(rdle<0)
    {
      cout<<" fail to read the port ttyACM0, please check the port and start the driver again"<<endl;
      return false;;
    }
    sleep_for(milliseconds(1));//等待读取 等待10ms
    string msg;
    msg.assign(reinterpret_cast<const char*>(buffer_),reinterpret_cast<const char*>(buffer_ + rdle));
    prase(msg);
    return true;
}

void Newton2Parser::prase(string msg)
{
    data_ = reinterpret_cast<const uint8_t*>(msg.data());
    data_end_ = data_ + msg.length();
    header_length_ = 0;
    total_length_ = 0;
    if(data_buffer_.size() !=0)
    {
        data_buffer_.clear();
    }
    
    while (data_ < data_end_) {
      //cout<<buffer_.size()<<"  "<<total_length_<<"  "<<header_length_<<endl;
      //数据格式参见OEM6 FW-20000129.book  22页
    if (data_buffer_.size() == 0) {  // Looking for SYNC0
      if (*data_ == novatel::SYNC_0) {
        data_buffer_.push_back(*data_);
      }
      ++data_;
    } else if (data_buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == novatel::SYNC_1) {
        data_buffer_.push_back(*data_++);
      } else {
        data_buffer_.clear();
      }
    } else if (data_buffer_.size() == 2) {  // Looking for SYNC2
      switch (*data_) {
        case novatel::SYNC_2_LONG_HEADER:
          data_buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          data_buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          data_buffer_.clear();
      }
    } else if (header_length_ > 0) {  // Working on header.
      if (data_buffer_.size() < header_length_) {
        data_buffer_.push_back(*data_++);
      } else {
        if (header_length_ == sizeof(novatel::LongHeader)) {
          total_length_ = header_length_ + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::LongHeader*>(data_buffer_.data())
                              ->message_length;
        } else if (header_length_ == sizeof(novatel::ShortHeader)) {
          total_length_ =
              header_length_ + novatel::CRC_LENGTH +
              reinterpret_cast<novatel::ShortHeader*>(data_buffer_.data())
                  ->message_length;
        } else {
          cout << "Incorrect header_length_. Should never reach here.";
          data_buffer_.clear();
        }
        header_length_ = 0;
      }
    } else if (total_length_ > 0) {
      if (data_buffer_.size() < total_length_) {  // Working on body.
        data_buffer_.push_back(*data_++);
        continue;
      }

      prepareMessage();
      
      data_buffer_.clear();
      total_length_ = 0;
        }
    }
}

void Newton2Parser::prepareMessage()
{
    uint8_t* message = nullptr;
    novatel::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;

    if (data_buffer_[2] == novatel::SYNC_2_LONG_HEADER) {
    auto header = reinterpret_cast<const novatel::LongHeader*>(data_buffer_.data());
    message = data_buffer_.data() + sizeof(novatel::LongHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
    } else {
    auto header = reinterpret_cast<const novatel::ShortHeader*>(data_buffer_.data());
    message = data_buffer_.data() + sizeof(novatel::ShortHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
    }

    //分配数据
    //TODO:由于数据暂时比较少，如增加RTK的话，可以看一下RTK是否有bestpose信号，以后如果信号多起来了，可以增加更多的信号
    if(message_id == novatel::RAWIMU)
    {
        seq_++;
        const novatel::RawImu* imu = reinterpret_cast<novatel::RawImu*>(message);
        double time = imu->gps_week*60 * 60 * 24 * 7 +imu->gps_seconds;
        time_pre_ = time;
        
        msg_imu_.header.stamp = ros::Time::now();
        msg_imu_.header.frame_id = frame_;
        msg_imu_.header.seq = seq_;
        msg_imu_.linear_acceleration.y = -imu->x_velocity_change*accel_scale_;
        msg_imu_.linear_acceleration.x = -imu->y_velocity_change_neg*accel_scale_;
        msg_imu_.linear_acceleration.z = -imu->z_velocity_change*accel_scale_;
        msg_imu_.angular_velocity.y = -imu->x_angle_change*gyro_scale_;
        msg_imu_.angular_velocity.x = -imu->y_angle_change_neg*gyro_scale_;
        msg_imu_.angular_velocity.z = imu->z_angle_change*gyro_scale_;

        imu_pub_.publish(msg_imu_);
    }
    else if(message_id == novatel::INSPVA)
    {
        const novatel::InsPva* inspva = reinterpret_cast<novatel::InsPva*>(message);
        double time = inspva->gps_week*60 * 60 * 24 * 7 +inspva->gps_seconds;
        double posx = inspva->longitude*M_PI/180;
        double posy = inspva->latitude*M_PI/180;
        pj_transform(wgs84pj_source_, utm_target_, 1, 1, &posx, &posy, NULL);

        msg_pose_.position.x = posx;
        msg_pose_.position.y = posy;
        msg_pose_.position.z = inspva->height;
        roll_ = inspva->roll *3.141592653/180.0;
        pitch_ = -inspva->pitch *3.141592653/180.0;
        yaw_= (90.0 - inspva->azimuth)*3.141592653/180.0;
        msg_pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_,
        pitch_,yaw_);

        pose_pub_.publish(msg_pose_);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(posy,posx,inspva->height));
        tf::Quaternion quaternion;
        quaternion.setRPY(roll_, pitch_, yaw_);
        transform.setRotation(quaternion);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gps"));
    }

}

}