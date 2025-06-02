// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/rsl200_interface.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <ros/console.h>
#include <string>

Interface::Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh):
  HardwareInterface(address, port, this),
  nh_(*nh)
{
  // No action needed
}

Interface::~Interface()
{
  disconnect();
}

void Interface::connect()
{
  HardwareInterface<UDPConnection>::connect();
  ROS_INFO_STREAM("[Laser Scanner] Listening to data");
}


void Interface::disconnect()
{
  HardwareInterface<UDPConnection>::disconnect();
  ROS_INFO_STREAM("[Laser Scanner] RSL Disconnected");
}

int Interface::parseBuffer(std::basic_string<unsigned char> buffer)
{
  if (debug_on)
  {
    LogBufferToDebug(buffer);
  }
  Frame *frame = reinterpret_cast<Frame *>(const_cast<unsigned char *>(buffer.c_str()));

  // To get the scanner type (0 for RSL400, 1 for RSL200) extract the highest 8 bits of the received fram ID:
  // Note: This is commented out because we do not use automatic device type detection in this driver
  // uint8_t scanner_type = static_cast<uint8_t>(frame->id >> 8);

  // From now on, we only use the remaining (lowest) 8 bits for frame ID:
  frame->id = static_cast<uint8_t>(frame->id);

  if (frame->id == 1)
    // Extended status profile. Status profile + measurement contour descritpion. Pg8 3.3.1.
  {
    parseExtendedStatusProfile(buffer);
  }
  else if ( (frame->id == 3) || (frame->id == 6) )
  {
    if (configuration_received_ == false)
    {
       ROS_WARN_STREAM("[Laser Scanner] Scan data header not received, skipping measurement.");
    }
    else
    {
      if (frame->scan_number != scan_number_)
      {
        ROS_WARN_STREAM("[Laser Scanner] Unexpected Scan Data id, skipping measurement.");
      }
      else
      {
        scan_data_[frame->block] = parseScanData(buffer, frame);
      }
    }
  }
  else if (frame->id == 0)
  {
    return frame->id;
  }
  else
  {
    ROS_ERROR_STREAM("[Laser Scanner] Unknown ID : " << frame->id);
    return -1;
  }
  if (measure_counter_ == scan_size_)
  {
    if (checkScan())
    {
      publishScan();
    }
  }
  if (measure_counter_ >= scan_size_)
  {
    ROS_WARN_STREAM("[Laser Scanner] Scan measure counter overflowed, resetting");
    configuration_received_ = false;
    resetDefault();
  }

  return frame->id;
}



DatagramMeasurementDataType Interface::parseScanData(std::basic_string<unsigned char> buffer,
                                                           Frame* frame)
{
  DatagramMeasurementDataType mdt;
  int length = 0;
  mdt.frame = frame;
  if (frame->id == 3)
  {
    length = (frame->h1.total_length - 20) / 4;
    // Capturing 4 bytes at a time - 2 for distance and 2 for signal strength
    // as per UDP spec pg 14 table 3.6
    for (int i = 20; i < buffer.length(); i += 4 )
    {
      uint16_t distance = convertBytesToUint16(buffer[i], buffer[i+1]);
      uint16_t intensity = convertBytesToUint16(buffer[i+2], buffer[i+3]);
      mdt.data_distance.push_back(distance);
      mdt.data_signal_strength.push_back(intensity);
    }
  }
  else if (frame->id == 6)
  {
    length = (frame->h1.total_length - 20) / 2;
    for (int i = 20; i < buffer.length(); i += 2 )   // Capturing 2 bytes at a time for distance
    {
      uint16_t distance = convertBytesToUint16(buffer[i], buffer[i+1]);
      mdt.data_distance.push_back(distance);
      mdt.data_signal_strength.push_back(0.0);
    }
  }

  // Buffer length should match length declared by header
  if (mdt.frame->h1.total_length != buffer.length())
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect length "
                      << buffer.length()
                      << ", expected "
                      << mdt.frame->h1.total_length);
    return mdt;
  }
  // Number of distance/signal values is equal to data length in bytes/4 (because 4 bytes per value)
  // Data langth is total length of datagram - 20 (which is fixed frame size)
  // Refer UDP specs pg 14 3.3.2.2. This gives number of "measurement data values"/"scan values".
  if (mdt.data_distance.size() != length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect number of data values "
                    << mdt.data_distance.size() << ", expected " << length);
    return mdt;
  }
  if (mdt.data_signal_strength.size() != length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect number of "
                    << "signal strength values "
                    << mdt.data_signal_strength.size()
                    << ", expected " << length);
    return mdt;
  }

  measure_counter_ += length;
  block_counter_ = mdt.frame->block+1;
  return mdt;
}


uint16_t Interface::convertBytesToUint16(unsigned char low_byte, unsigned char high_byte)
{
  return high_byte << 8 | low_byte;
}


bool Interface::compareTwoFloats(float a, float b, float epsilon)
{
  return fabs(a - b) < epsilon;
}


bool Interface::checkScan()
{
  int i_measure = 0;
  // Assemble data from block;
  for (int i_block = 0; i_block < block_counter_; i_block++)
  {
    if (scan_data_[i_block].data_distance.size() == 0)
    {
      ROS_INFO_STREAM("[Laser Scanner] Received scan data datagram with no distance values");
      return false;
    }
    else
    {
      for (int i_scan=0; i_scan< scan_data_[i_block].data_distance.size(); i_scan++)
      {
        laser_scan_.ranges[i_measure] =
          static_cast<float>(scan_data_[i_block].data_distance[i_scan]) / 1000.0;
        laser_scan_.intensities[i_measure] =
          static_cast<float>(scan_data_[i_block].data_signal_strength[i_scan]);
        i_measure++;
      }
    }
  }
  laser_scan_.header.stamp = ros::Time::now();
  // Reverse the scans to match real world
  std::reverse(laser_scan_.ranges.begin(), laser_scan_.ranges.end());
  std::reverse(laser_scan_.intensities.begin(), laser_scan_.intensities.end());
  return true;
}


void Interface::LogBufferToDebug(std::basic_string<unsigned char> buffer)
{
  std::stringstream oss;
  oss << std::hex;
  for (int i = 0; i < buffer.length(); i++)
  {
    oss << std::setw(2) << std::setfill('0') << static_cast<u_int16_t>(buffer[i]);
  }
  std_msgs::String string_msg;
  string_msg.data = oss.str();
  pub_debug_.publish(string_msg);
}
