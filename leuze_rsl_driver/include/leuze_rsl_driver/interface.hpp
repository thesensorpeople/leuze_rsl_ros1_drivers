// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_INTERFACE_H
#define LEUZE_INTERFACE_H

#include "leuze_rsl_driver/communication.hpp"
#include "leuze_rsl_driver/hardware_interface.hpp"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "std_msgs/String.h"
class Interface : public HardwareInterface<UDPConnection>, DataParser
{
public:
    Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh);
    ~Interface();

    void connect();
    void disconnect();
    int parseBuffer(std::basic_string<unsigned char> buffer);

protected:
    virtual void resetDefault() = 0;
    bool checkScan();
    virtual void publishScan() = 0;
    virtual void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) = 0;
    DatagramMeasurementDataType parseScanData(std::basic_string<unsigned char> buffer, Frame* frame);
    uint16_t convertBytesToUint16(unsigned char low_byte, unsigned char high_byte);
    bool compareTwoFloats(float a, float b,float epsilon = 0.0001);

    ros::NodeHandle nh_;
    ros::Publisher pub_scan_;
    ros::Publisher pub_status_;
    ros::Publisher pub_debug_;
    std::string header_frame_;
    bool configuration_received_;
    bool debug_on;

    sensor_msgs::LaserScan laser_scan_;
    int scan_number_;
    int configuration_type_; // type 3 = Distance + Intensity / type 6 = Distance
    int measure_counter_;
    int block_counter_;
    int scan_size_;
    std::vector<DatagramMeasurementDataType> scan_data_;

    void LogBufferToDebug(std::basic_string<unsigned char> buffer);
};

#endif
