#ifndef LEUZE_PHIDGET_DRIVER_H
#define LEUZE_PHIDGET_DRIVER_H

#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class LeuzePhidgetDriver
{
public:
  LeuzePhidgetDriver(ros::NodeHandle* nh, uint16_t ikInputsNo, uint16_t ikOutputsNo);

protected:
  void readInputStateCallback(const std_msgs::BoolConstPtr &msg, int i);
  void spawnInputSubscribers();
  void spawnOutputPublishers();
  void publishOutputState();
  ros::NodeHandle nh_;
  std::vector<int> input_state_;
  std::vector<int> output_state_;
  ros::Publisher pub_show_inputs_;
  std::vector<ros::Subscriber> input_subscribers_;
  std::vector<ros::Publisher> output_publishers_;
  ros::Subscriber sub_read_input_;
  ros::Subscriber sub_get_outputs_;

  const int _NUMBER_OF_IK_INPUTS_;
  const int _NUMBER_OF_IK_OUTPUTS_;
};

#endif // LEUZE_PHIDGET_DRIVER_H
