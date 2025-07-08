#include "gps.h"
#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>

#define EARTH_RADIUS 6371000
using namespace hesai::lidar;

// Constructor
GPSSDK::GPSSDK(std::string GPS_path)
    : GPS_path(GPS_path), running(false) {}

// Destructor
GPSSDK::~GPSSDK() {}

void GPSSDK::LoadAllGPSData() {
    std::ifstream file(GPS_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open GPS file " << GPS_path << std::endl;
        return;
    }

    std::string line;
    std::getline(file, line);  // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        LidarGPSData packet;
        std::string value;

        std::getline(ss, value, ','); 
        std::getline(ss, value, ','); packet.timestamp = safeStod(value);  // time
        std::getline(ss, value, ','); 
        std::getline(ss, value, ','); 

        
        std::getline(ss, value, ','); packet.lat = safeStod(value);
        std::getline(ss, value, ','); packet.lon = safeStod(value);
        std::getline(ss, value, ','); packet.alt = safeStod(value);
        std::getline(ss, value, ',');
        std::getline(ss, value, ','); 
        std::getline(ss, value, ','); packet.fix = safeStod(value);
        std::getline(ss, value, ','); packet.sip = safeStod(value);
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');
        std::getline(ss, value, ','); packet.h2acc = safeStod(value);
        std::getline(ss, value, ','); packet.v2acc = safeStod(value);
        std::getline(ss, value, ','); packet.t3acc = safeStod(value);

        if (std::isnan(packet.timestamp) ||
            std::isnan(packet.lat) || std::isnan(packet.lon) || std::isnan(packet.alt) ||
            std::isnan(packet.fix) || std::isnan(packet.hdop) || std::isnan(packet.h2acc) ||
            std::isnan(packet.v2acc))
        {
            std::cerr << "Warning: Skipping invalid GPS row.\n";
            continue;
        }
        packet.timestamp = packet.timestamp + 21600.00;
        packet.covariance = std::vector<double>{
            (packet.h2acc * packet.h2acc) / 2, 0.0, 0.0,
            0.0, (packet.h2acc * packet.h2acc) / 2, 0.0,
            0.0, 0.0, (packet.v2acc * packet.v2acc) / 2
        };

        
        all_GPS_data.push_back(packet);
    }
    std::cout << "All GPS data parsed\n";

    file.close();
}

bool GPSSDK::SkipToTimestamp(double start_time) {
    playback_index = 0;
    std::cout << "Length of GPS data: " << all_GPS_data.size() << std::endl;
    while (playback_index < all_GPS_data.size() &&
           all_GPS_data[playback_index].timestamp < start_time) {
        playback_index++;
    }

    if (playback_index >= all_GPS_data.size()) {
        std::cerr << "Error: No GPS data found after start time.\n";
        return false;
    }
    std::cout << "Playback index after skipping: " << playback_index << std::endl;
    return true;
}

boost::optional<LidarGPSData> GPSSDK::GetGPSPacket() {
    if (playback_index < all_GPS_data.size()){
        LidarGPSData packet = all_GPS_data[playback_index];
        playback_index++;
        return packet;
    } else {
        return boost::none;  // Stop was called
    }
}


boost::optional<std::vector<LidarGPSData*>> GPSSDK::GetGPSPackets(double start_time, double end_time) {
    std::vector<LidarGPSData*> packets;

    while (playback_index < all_GPS_data.size()) {
        LidarGPSData& packet = all_GPS_data[playback_index];
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

