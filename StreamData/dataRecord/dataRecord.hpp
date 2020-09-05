#pragma once

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

class serialstream
{
public:
    serialstream();
    size_t dataspin(uint8_t*,int);
    bool is_open_ = false;
    static constexpr size_t BUFFER_SIZE = 2048;
    uint8_t buffer_[BUFFER_SIZE] = {0};
    int fd_;
private:
    void Connect();
    
    uint8_t buffer_rtk_[BUFFER_SIZE] = {0};
    void opendevice();
    bool configport();
    char *device_name_;
    speed_t baud_rate_;
    
    

};