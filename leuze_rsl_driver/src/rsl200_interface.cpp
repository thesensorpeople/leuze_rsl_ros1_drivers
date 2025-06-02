// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/rsl200_interface.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <ros/console.h>
#include <string>

RSL200Interface::RSL200Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh):
  Interface(address, port, topic, nh)
{
  // -135/135 0.1
  pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(topic, 50);
  pub_status_ = nh_.advertise<leuze_msgs::ExtendedStatusProfileMsg_rsl200>("status", 50);
  ros::NodeHandle private_nh_("~");
  configuration_received_ = false;
  scan_size_ = 1351;  // [RSL400: Use 2700]
  scan_data_.resize(scan_size_);

  if (!private_nh_.getParam("scan_frame", header_frame_))
  {
    ROS_WARN_STREAM("[Laser Scanner] scan_frame param not found, loading default value of \"world\"");
    header_frame_ = "world";
  }
  if (!private_nh_.getParam("ls_debug", debug_on))
  {
    debug_on = false;
  }
  if (debug_on)
  {
    pub_debug_ = nh_.advertise<std_msgs::String>("/scan_raw_data", 50);
    ROS_WARN_STREAM("[Laser Scanner] Debug Mode is on. This might affect the performace.");
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
        ROS_DEBUG_STREAM("[Laser Scanner] Debug Stream Enabled");
    }
  }
  resetDefault();
}

RSL200Interface::~RSL200Interface()
{
  disconnect();
}


void RSL200Interface::resetDefault()
{
  laser_scan_.header.frame_id = header_frame_;
  // -2.35794; Default min value (-135.1 instead of -135 because we have one extra beam for 0):
  laser_scan_.angle_min = nh_.param("/angle_min", -2.35794);
  // 2.35794; Default min value (+135.1 instead of +135 because we have one extra beam for 0):
  laser_scan_.angle_max = nh_.param("/angle_max", 2.35794);
  // Default max resolution:
  laser_scan_.angle_increment = (laser_scan_.angle_max-laser_scan_.angle_min) /
                                  static_cast<float>(scan_size_);
  laser_scan_.scan_time = nh_.param("/scan_time", 0.025);    // 0.025; Default
  laser_scan_.range_min = nh_.param("/range_min", 0.001);    // 0.001; Default
  laser_scan_.range_max = nh_.param("/range_max", 25.0);     // 25.0; Max range 25m
  laser_scan_.ranges.resize(0);
  laser_scan_.ranges.resize(scan_size_);
  laser_scan_.intensities.resize(0);
  laser_scan_.intensities.resize(scan_size_);
  block_counter_ = 0;
  measure_counter_ = 0;
  scan_number_ = -1;  // Get last scan number
  ROS_INFO_STREAM("[Laser Scanner] Reset data");
}


void RSL200Interface::parseExtendedStatusProfile(std::basic_string<unsigned char> buffer)
{
  DatagramExtendedStatusProfile_rsl200 *esp =
        reinterpret_cast<DatagramExtendedStatusProfile_rsl200 *>(const_cast<unsigned char *>(buffer.c_str()));

  if (buffer.length() != esp->frame.h1.total_length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Extended Status Profile of incorrect length "
                    << buffer.length()
                    << ", expected "
                    << esp->frame.h1.total_length);
    return;
  }
  verifyConfiguration(*esp);

  status_msg_.header.frame_id = header_frame_;
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.image_type = esp->status_profile.imageType;
  status_msg_.op_mode = esp->status_profile.op_mode;
  status_msg_.error_bits = esp->status_profile.errorBits;
  status_msg_.detection_state_bits = esp->status_profile.detectionStateBits;
  status_msg_.triple_sel = esp->status_profile.triple_sel;
  status_msg_.aux = esp->status_profile.aux;
  status_msg_.input_bits = esp->status_profile.inputBits;
  status_msg_.output_bits = esp->status_profile.outputBits;
  status_msg_.volt = esp->status_profile.volt;
  status_msg_.temp = esp->status_profile.temp;
  status_msg_.level_x = esp->status_profile.level_x;
  status_msg_.level_y = esp->status_profile.level_y;
  status_msg_.scan_number = esp->status_profile.scan_number;
  status_msg_.safe_sig = esp->status_profile.safe_sig;
  status_msg_.err_data = esp->status_profile.errDw;

  status_msg_.start_index = esp->measurement_contour_descritption.start_index;
  status_msg_.stop_index = esp->measurement_contour_descritption.stop_index;
  status_msg_.index_interval = esp->measurement_contour_descritption.index_interval;
  status_msg_.reserved = esp->measurement_contour_descritption.reseved;
}


void RSL200Interface::verifyConfiguration(DatagramExtendedStatusProfile_rsl200 d_esp)
{
  // Devision by because the resolution is 0.2° (use /10 for RSL400):
  float min_angle_from_esp = (static_cast<float>(d_esp.measurement_contour_descritption.start_index)/5);
  // +1 here to account for the fact that internal calculations are for example
  // from -135° to +135° but actual represntation is from 0° to 269.9° (difference of 0.1°)
  float max_angle_from_esp = (static_cast<float>(d_esp.measurement_contour_descritption.stop_index + 1)/5);
  float avg_angle = (min_angle_from_esp+max_angle_from_esp)/2;

  // Adjust for example from -135° to +135° to 0° to 270°
  min_angle_from_esp =  angles::from_degrees(min_angle_from_esp - avg_angle);
  max_angle_from_esp =  angles::from_degrees(max_angle_from_esp - avg_angle);

  if (!compareTwoFloats(min_angle_from_esp, laser_scan_.angle_min))
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal minimum angle of "
                    << laser_scan_.angle_min
                    << " does not match the value received from the laser "
                    << min_angle_from_esp
                    << ". Adjusting internally");
    laser_scan_.angle_min = min_angle_from_esp;
  }

  if (!compareTwoFloats(max_angle_from_esp, laser_scan_.angle_max))
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal maximum angle of "
                    << laser_scan_.angle_max
                    << " does not match the value received from the laser "
                    << max_angle_from_esp
                    << ". Adjusting internally");
    laser_scan_.angle_max = max_angle_from_esp;
  }

  if (scan_size_ != d_esp.getBeamCount())
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal beam count of "
                    << scan_size_ << " does not match the value received from the laser "
                    << d_esp.getBeamCount()
                    << ". Adjusting internally");
    scan_size_ = d_esp.getBeamCount();
  }
  if (measure_counter_ != 0)
  {
    ROS_WARN_STREAM("[Laser Scanner] Received ExtendedProfile at unexcepted timing.");
    measure_counter_ = 0;
  }
  scan_data_.clear();
  scan_data_.resize(scan_size_);
  scan_number_ = d_esp.frame.scan_number;
  configuration_received_ = true;
  // Length 56 is fixed
  if (d_esp.frame.h1.total_length != 56)  // [RSL400] Use 48 for RSL400
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Extended Status Profile of incorrect length "
                      << d_esp.frame.h1.total_length
                      << ", expected "
                      << 56);  // [RSL400] Use 48 for RSL400
  }
}


void RSL200Interface::publishScan()
{
  pub_scan_.publish(laser_scan_);
  pub_status_.publish(status_msg_);
  measure_counter_ = 0;
}
