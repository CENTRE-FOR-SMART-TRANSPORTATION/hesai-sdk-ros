#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>

#define EARTH_RADIUS 6371000
using namespace hesai::lidar;

// Constructor
ImuSDK::ImuSDK(std::string imu_path)
    : imu_path(imu_path), running(false) {}

// Destructor
ImuSDK::~ImuSDK() {}

// Convert degrees to radians
double degtorad(double degree) {
    return degree * M_PI / 180.0;
}

double safeStod(const std::string& str) {
    try {
        return std::stod(str); // Try to convert the string to double
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Invalid value '" << str << "' encountered during conversion." << std::endl;
        return NAN; // Return NaN to indicate invalid data
    }
}

void ImuSDK::LoadAllImuData() {
    std::ifstream file(imu_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open IMU file " << imu_path << std::endl;
        return;
    }

    std::string line;
    std::getline(file, line);  // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        LidarImuData packet;
        std::string value;

        std::getline(ss, value, ','); 
        std::getline(ss, value, ','); packet.timestamp = safeStod(value);  // time
        std::getline(ss, value, ','); 
        
        std::getline(ss, value, ','); 
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');
        std::getline(ss, value, ','); packet.imu_accel_x = safeStod(value);
        std::getline(ss, value, ','); packet.imu_accel_y = safeStod(value);
        std::getline(ss, value, ','); packet.imu_accel_z = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_x = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_y = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_z = safeStod(value);
        std::getline(ss, value, ','); packet.imu_quat_x = safeStod(value);
        std::getline(ss, value, ','); packet.imu_quat_y = safeStod(value);
        std::getline(ss, value, ','); packet.imu_quat_z = safeStod(value);
        std::getline(ss, value, ','); packet.imu_quat_w = safeStod(value);

        if (std::isnan(packet.timestamp) ||
            std::isnan(packet.imu_accel_x) || std::isnan(packet.imu_accel_y) || std::isnan(packet.imu_accel_z) ||
            std::isnan(packet.imu_ang_vel_x) || std::isnan(packet.imu_ang_vel_y) || std::isnan(packet.imu_ang_vel_z) ||
            std::isnan(packet.imu_quat_x) || std::isnan(packet.imu_quat_y) || std::isnan(packet.imu_quat_z) || std::isnan(packet.imu_quat_w))
        {
            std::cerr << "Warning: Skipping invalid IMU row.\n";
            continue;
        }

        packet.timestamp /= 1000.0;  // Convert ms to seconds
        all_imu_data.push_back(packet);
    }

    file.close();
}

bool ImuSDK::SkipToTimestamp(double start_time) {
    playback_index = 0;
    while (playback_index < all_imu_data.size() &&
           all_imu_data[playback_index].timestamp < start_time) {
        playback_index++;
    }

    if (playback_index >= all_imu_data.size()) {
        std::cerr << "Error: No IMU data found after start time.\n";
        return false;
    }

    return true;
}

boost::optional<LidarImuData> ImuSDK::GetImuPacket() {
    if (playback_index < all_imu_data.size()){
        LidarImuData packet = all_imu_data[playback_index];
        playback_index++;
        return packet;
    } else {
        return boost::none;  // Stop was called
    }
}