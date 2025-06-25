#ifndef INCLUDE_IMUSDK_H
#define INCLUDE_IMUSDK_H

#include <sensor_msgs/Imu.h>
#include "lidar_types.h"

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <boost/optional.hpp>
#include <ros/time.h>
#include <condition_variable>

namespace hesai
{
namespace lidar
{

class ImuSDK
{
public:
    ImuSDK(std::string imu_path);
    ~ImuSDK();

    void LoadAllImuData();
    bool SkipToTimestamp(double start_time);
    boost::optional<LidarImuData> GetImuPacket();

    std::condition_variable imu_cv;

private:
    void ReadIMUData();  // Internal function for reading data
    std::string imu_path;
    std::thread imu_thread;
    std::atomic<bool> running;
    bool imu_file_has_gps;
    std::vector<LidarImuData> all_imu_data;    // how to define an empty queue
    std::mutex imu_mutex;
    size_t playback_index = 0;
};
}
}


#endif // INCLUDE_PANDARGENERAL_H_
