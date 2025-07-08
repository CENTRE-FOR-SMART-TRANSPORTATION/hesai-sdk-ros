#ifndef INCLUDE_GPSSDK_H
#define INCLUDE_GPSSDK_H

#include <sensor_msgs/NavSatFix.h> // Corrected include
#include "lidar_types.h"

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <boost/optional.hpp>
#include <ros/time.h>
#include <condition_variable>

namespace hesai
{
namespace lidar
{

class GPSSDK
{
public:
    GPSSDK(std::string GPS_path);
    ~GPSSDK();

    void LoadAllGPSData();
    bool SkipToTimestamp(double start_time);
    boost::optional<LidarGPSData> GetGPSPacket();
    boost::optional<std::vector<LidarGPSData*>> GetGPSPackets(double start_time, double end_time);

    std::condition_variable GPS_cv;

private:
    void ReadGPSData();  // Internal function for reading data
    std::string GPS_path;
    std::thread GPS_thread;
    std::atomic<bool> running;
    bool GPS_file_has_gps;
    std::vector<LidarGPSData> all_GPS_data; // This is already an empty vector
    std::mutex GPS_mutex;
    size_t playback_index = 0;
};

} // namespace lidar
} // namespace hesai

#endif // INCLUDE_GPSSDK_H
