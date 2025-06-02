// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)f(
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/rsl400_interface.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <ros/console.h>
#include <string>

RSL400Interface::RSL400Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh):
  Interface(address, port, topic, nh)
{
  // -135/135 0.1
  pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(topic, 50);
  pub_status_ = nh_.advertise<leuze_msgs::ExtendedStatusProfileMsg_rsl400>("status", 50);
  ros::NodeHandle private_nh_("~");
  configuration_received_ = false;
  scan_size_ = 2700;
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


RSL400Interface::~RSL400Interface()
{
  disconnect();
}


void RSL400Interface::resetDefault()
{
  laser_scan_.header.frame_id = header_frame_;
  laser_scan_.angle_min = nh_.param("/angle_min", -2.35619449);   // Default min value (-135)
  laser_scan_.angle_max = nh_.param("/angle_max", 2.35619449);    // Default max value (+135)
  // Default max resolution:
  laser_scan_.angle_increment = (laser_scan_.angle_max-laser_scan_.angle_min) /
                                  static_cast<float>(scan_size_);
  laser_scan_.scan_time = nh_.param("/scan_time", 0.04);   // 0.04; Default
  laser_scan_.range_min = nh_.param("/range_min", 0.001);  // 0.001; Default
  laser_scan_.range_max = nh_.param("/range_max", 65.0);   // 65.0; Max range 65m
  laser_scan_.ranges.resize(0);
  laser_scan_.ranges.resize(scan_size_);
  laser_scan_.intensities.resize(0);
  laser_scan_.intensities.resize(scan_size_);
  block_counter_ = 0;
  measure_counter_ = 0;
  scan_number_ = -1;  // Get last scan number
  ROS_INFO_STREAM("[Laser Scanner] Reset data");
}


void RSL400Interface::parseExtendedStatusProfile(std::basic_string<unsigned char> buffer)
{
  DatagramExtendedStatusProfile_rsl400 *esp =
      reinterpret_cast<DatagramExtendedStatusProfile_rsl400 *>(const_cast<unsigned char *>(buffer.c_str()));

  if (buffer.length() != esp->frame.h1.total_length)
  {
    ROS_ERROR_STREAM(
        "[Laser Scanner] Parsing Extended Status Profile of incorrect length "
        << buffer.length() << ", expected " << esp->frame.h1.total_length);
    return;
  }
  verifyConfiguration(*esp);

  status_msg_.header.frame_id = header_frame_;
  status_msg_.header.stamp = ros::Time::now();

  status_msg_.byte_0 = esp->status_profile.byte_0;
  status_msg_.byte_1 = esp->status_profile.byte_1;
  status_msg_.msg_and_ossd_2 = esp->status_profile.msg_and_ossd_2;
  status_msg_.emergency_stop_3 = esp->status_profile.emergency_stop_3;
  status_msg_.electrical_signals_byte_4 = esp->status_profile.electrical_signals_byte_4;
  status_msg_.electrical_signals_byte_5 = esp->status_profile.electrical_signals_byte_5;
  status_msg_.electrical_signals_byte_6 = esp->status_profile.electrical_signals_byte_6;
  status_msg_.electrical_signals_byte_7 = esp->status_profile.electrical_signals_byte_7;
  status_msg_.scan_number = esp->status_profile.scan_number;
  status_msg_.protec_func_a_12 = esp->status_profile.protec_func_a_12;
  status_msg_.fp_sel_a_byte_13 = esp->status_profile.fp_sel_a_byte_13;
  status_msg_.fp_sel_a_byte_14 = esp->status_profile.fp_sel_a_byte_14;
  status_msg_.indic_a_15 = esp->status_profile.indic_a_15;
  status_msg_.protec_func_b_16 = esp->status_profile.protec_func_b_16;
  status_msg_.fp_sel_b_byte_17 = esp->status_profile.fp_sel_b_byte_17;
  status_msg_.fp_sel_b_byte_18 = esp->status_profile.fp_sel_b_byte_18;
  status_msg_.indic_b_19 = esp->status_profile.indic_b_19;

  status_msg_.start_index = esp->measurement_contour_descritption.start_index;
  status_msg_.stop_index = esp->measurement_contour_descritption.stop_index;
  status_msg_.index_interval = esp->measurement_contour_descritption.index_interval;
  status_msg_.reserved = esp->measurement_contour_descritption.reseved;
}


void RSL400Interface::verifyConfiguration(DatagramExtendedStatusProfile_rsl400 d_esp)
{
  float min_angle_from_esp = (static_cast<float>(d_esp.measurement_contour_descritption.start_index)/10);
  // +1 here to account for the fact that internal calculations are for example
  // from -135° to +135° but actual represntation is from 0° to 269.9° (difference of 0.1°)
  float max_angle_from_esp = (static_cast<float>(d_esp.measurement_contour_descritption.stop_index + 1)/10);
  float avg_angle = (min_angle_from_esp+max_angle_from_esp)/2;

  // Adjust for example from -135° to +135° to 0° to 270°
  min_angle_from_esp =  angles::from_degrees(min_angle_from_esp - avg_angle);
  max_angle_from_esp =  angles::from_degrees(max_angle_from_esp - avg_angle);

  if (!compareTwoFloats(min_angle_from_esp, laser_scan_.angle_min))
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal minimum angle of "
                    << laser_scan_.angle_min
                    << " does not match the value received from the laser "
                    << min_angle_from_esp << ". Adjusting internally");
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
  // Length 48 is fixed
  if (d_esp.frame.h1.total_length != 48)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Extended Status Profile of incorrect length "
                      << d_esp.frame.h1.total_length
                      << ", expected "
                      << 48);
  }
}


void RSL400Interface::publishScan()
{
  pub_scan_.publish(laser_scan_);
  pub_status_.publish(status_msg_);
  measure_counter_ = 0;
}
