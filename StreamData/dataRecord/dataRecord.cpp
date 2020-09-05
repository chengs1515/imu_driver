#include "dataRecord.hpp"
#include <vector>
#include <unistd.h>
#include "novatel_messages.h"
#include <chrono>
#include <thread>

using std::cout;
using std::endl;
using namespace std::chrono;
using namespace std::this_thread;


serialstream::serialstream()
{
    device_name_ = "/dev/ttyACM0";
    baud_rate_ = B115200;
    Connect();
}

void serialstream::Connect()//这里的connect包括了login操作
{
    if(!is_open_)
    {
        opendevice();
        if(!is_open_)
        {
            cout<<"open device failed because : "<<strerror(errno)<<endl;
            return;
        }
    }

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
            sleep_for(milliseconds(10));//不能直接发送，需要有一定的延时，否则会发送不成功,参见129.book 第193页
            ssize_t nsent = ::write(fd_,reinterpret_cast<const uint8_t*>(command.data()),command.size());
            if(nsent<0)
            {
                cout<<"sent data failed because: "<<strerror(errno)<<endl;
            }
            else if(nsent>0) break;

            sendtimes++;
        }
        if(sendtimes == 5)
            cout<<command<<endl;
    }
}

bool serialstream::configport()
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

void serialstream::opendevice()
{
    fd_ = ::open(device_name_,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd_ == -1)
    {
        cout<<"There is an error in "<<device_name_<<" because: "<<strerror(errno)<<endl;//通过报错知道是什么原因，可能是因为没有权限，也可能是太多文件
        return;
    }
    
    if(!configport())
    {
        ::close(fd_);
        return;
    }
    is_open_ = true;

}

size_t serialstream::dataspin(uint8_t* buffer,int max_length)
{
    if(!is_open_)
    {
        Connect();
    }
    int rdlen = -1;

    ssize_t bytes_read = 0;
    ssize_t bytes_current_read = 0;

    sleep(2);  // wait 10ms
    //cout<<1<<endl;
    while(max_length>0)
    {
        rdlen = ::read(fd_, buffer, max_length);
        if (rdlen < 0) 
        {
            cout<<"Error from read: "<<rdlen<<" because: "<< strerror(errno)<<endl;
        }
        else if(rdlen == 0)
        {
            char data = 0;
            ssize_t nsent = ::write(fd_, &data, 0);
            if (nsent < 0) {
                cout << "Serial stream detect write failed, error: " << strerror(errno);
            break;
        }
        }
        cout<<rdlen<<endl;
        max_length -= rdlen;
        buffer += rdlen;
        bytes_read += rdlen;
    }
    return bytes_read;
}

int main()
{
    serialstream ss;
    std::ofstream ofile("gnssData.bin",std::ios::binary|std::ios::app);
    int times = 0;
    while(times<10000)
    {
        ssize_t rdle = ::read(ss.fd_, ss.buffer_, ss.BUFFER_SIZE);
        uint8_t* buff = ss.buffer_;
        ofile.write(reinterpret_cast<char*>(buff),rdle);
        sleep_for(milliseconds(10));
        times++;
    }
    ofile.close();
}