#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

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
        
        std::getline(ss, value, ','); packet.roll = safeStod(value);
        std::getline(ss, value, ','); packet.pitch = safeStod(value);
        std::getline(ss, value, ','); packet.yaw = safeStod(value);
        std::getline(ss, value, ','); packet.imu_accel_x = safeStod(value);
        std::getline(ss, value, ','); packet.imu_accel_y = safeStod(value);
        std::getline(ss, value, ','); packet.imu_accel_z = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_x = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_y = safeStod(value);
        std::getline(ss, value, ','); packet.imu_ang_vel_z = safeStod(value);
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');

        if (std::isnan(packet.timestamp) ||
            std::isnan(packet.imu_accel_x) || std::isnan(packet.imu_accel_y) || std::isnan(packet.imu_accel_z) ||
            std::isnan(packet.imu_ang_vel_x) || std::isnan(packet.imu_ang_vel_y) || std::isnan(packet.imu_ang_vel_z) ||
            std::isnan(packet.roll) || std::isnan(packet.pitch) || std::isnan(packet.yaw))
        {
            std::cerr << "Warning: Skipping invalid IMU row.\n";
            continue;
        }

        // Convert rpy to quats
        tf2::Quaternion q;
        q.setRPY(packet.roll, packet.pitch, packet.yaw);
        packet.imu_quat_x = q.x();
        packet.imu_quat_y = q.y();
        packet.imu_quat_z = q.z();
        packet.imu_quat_w = q.w();

        packet.timestamp /= 1000.0;  // Convert ms to seconds
        all_imu_data.push_back(packet);
    }
    std::cout << "All IMU data parsed\n";

    file.close();
}

bool ImuSDK::SkipToTimestamp(double start_time) {
    playback_index = 0;
    std::cout << "Length of IMU data: " << all_imu_data.size() << std::endl;
    while (playback_index < all_imu_data.size() &&
           all_imu_data[playback_index].timestamp < start_time) {
        playback_index++;
    }

    if (playback_index >= all_imu_data.size()) {
        std::cerr << "Error: No IMU data found after start time.\n";
        return false;
    }
    std::cout << "Playback index after skipping: " << playback_index << std::endl;
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


boost::optional<std::vector<LidarImuData*>> ImuSDK::GetImuPackets(double start_time, double end_time) {
    std::vector<LidarImuData*> packets;

    while (playback_index < all_imu_data.size()) {
        LidarImuData& packet = all_imu_data[playback_index];
        if (packet.timestamp >= start_time && packet.timestamp < end_time) {
            packets.push_back(&packet);
            playback_index++;
        } else if (packet.timestamp < start_time) {
            playback_index++;
        } else {
            break;
        }
    }

    if (!packets.empty()) {
        return packets;
    } else {
        return boost::none;
    }
}

