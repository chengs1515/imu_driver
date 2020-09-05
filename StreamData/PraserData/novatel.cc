#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include "novatel_messages.h"
#include <unordered_map>
using namespace std;

vector<int> record(2000,0);

vector<uint8_t> buffer_;
int times = 0;
void PrepareMessage()
{
    uint8_t* message = nullptr;
    novatel::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;

    if (buffer_[2] == novatel::SYNC_2_LONG_HEADER) {
    auto header = reinterpret_cast<const novatel::LongHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::LongHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
    } else {
    auto header = reinterpret_cast<const novatel::ShortHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::ShortHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
    }
    record[message_id]++;
    //cout<<message_id<<endl;
    // if(message_id == novatel::RAWIMU||message_id == novatel::RAWIMU)
    // {
    //         const novatel::RawImu* imu = reinterpret_cast<novatel::RawImu*>(message);
    //         cout<<"imu message: "<<imu-><<"  "<<imu->z_angle_change<<"  "<<imu->gps_seconds<<endl;
    // }
    // else if(message_id == novatel::HEADING)
    // {
    //     const novatel::Heading* headi = reinterpret_cast<novatel::Heading*>(message);
    //     cout<<"heading message: "<<(uint32_t)headi->position_type<<"   "<<headi->heading<<endl;
    // }

}

void prase(string msg)
{
    const uint8_t* data_ = reinterpret_cast<const uint8_t*>(msg.data());
    const uint8_t* data_end_ = data_ + msg.length();
    size_t header_length_ = 0;
    size_t total_length_ = 0;

    while (data_ < data_end_) {
      //cout<<buffer_.size()<<"  "<<total_length_<<"  "<<header_length_<<endl;
      //数据格式参见OEM6 FW-20000129.book  22页
    if (buffer_.size() == 0) {  // Looking for SYNC0
      if (*data_ == novatel::SYNC_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == novatel::SYNC_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for SYNC2
      switch (*data_) {
        case novatel::SYNC_2_LONG_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          buffer_.clear();
      }
    } else if (header_length_ > 0) {  // Working on header.
      if (buffer_.size() < header_length_) {
        buffer_.push_back(*data_++);
      } else {
        if (header_length_ == sizeof(novatel::LongHeader)) {
          total_length_ = header_length_ + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::LongHeader*>(buffer_.data())
                              ->message_length;
        } else if (header_length_ == sizeof(novatel::ShortHeader)) {
          total_length_ =
              header_length_ + novatel::CRC_LENGTH +
              reinterpret_cast<novatel::ShortHeader*>(buffer_.data())
                  ->message_length;
        } else {
          cout << "Incorrect header_length_. Should never reach here.";
          buffer_.clear();
        }
        header_length_ = 0;
      }
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }

      PrepareMessage();
      
      buffer_.clear();
      //cout<<"run this:"<<"  "<<buffer_.size()<<endl;
      total_length_ = 0;
        }
    }
    //cout<<"while over"<<endl;
}

int main()
{
    std::ios::sync_with_stdio(false);
    ifstream f("gnssData.bin",std::ios::binary|std::ios::in);
    size_t buffer_size = 128;
    char b[buffer_size];
    while(f)
    {
        f.read(b,buffer_size);
        buffer_.clear();
        string msg;
        msg.assign(reinterpret_cast<const char*>(b), f.gcount());
        prase(msg);
    }
    f.close();
    for(int i = 0;i<record.size();i++)
    {
      if(record[i]!=0)
        cout<<i<<"   "<<record[i]<<endl;
    }

}