#ifndef INCLUDE_IMUSDK_H
#define INCLUDE_IMUSDK_H

#include <sensor_msgs/Imu.h>
#include "lidar_types.h"

#include <vector>
#include <cmath>
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


inline double safeStod(const std::string& str) {
    try {
        return std::stod(str); // Try to convert the string to double
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Invalid value '" << str << "' encountered during conversion." << std::endl;
        return NAN; // Return NaN to indicate invalid data
    }
}

class ImuSDK
{
public:
    ImuSDK(std::string imu_path, bool roll_correct);
    ~ImuSDK();

    void LoadAllImuData();
    bool SkipToTimestamp(double start_time);
    boost::optional<LidarImuData> GetImuPacket();
    boost::optional<std::vector<LidarImuData*>> GetImuPackets(double start_time, double end_time);

    std::condition_variable imu_cv;

private:
    void ReadIMUData();  // Internal function for reading data
    std::string imu_path;
    std::thread imu_thread;
    bool roll_correct;
    std::atomic<bool> running;
    bool imu_file_has_gps;
    std::vector<LidarImuData> all_imu_data;    // how to define an empty queue
    std::mutex imu_mutex;
    size_t playback_index = 0;
};
}
}


#endif // INCLUDE_PANDARGENERAL_H_
