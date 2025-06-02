#include "leuze_phidget_driver.hpp"
#include <string>

LeuzePhidgetDriver::LeuzePhidgetDriver(ros::NodeHandle *nh, uint16_t ikInputsNo, uint16_t ikOutputsNo):
  nh_(*nh), _NUMBER_OF_IK_INPUTS_(ikInputsNo), _NUMBER_OF_IK_OUTPUTS_(ikOutputsNo)
{}

void LeuzePhidgetDriver::readInputStateCallback(const std_msgs::BoolConstPtr &msg, int i)
{
  input_state_[i] = msg->data?1:0;
  std::cout <<   "Received for input pin : " << i << " state : " << (msg->data?"True":"False") << ". Current input state : ";
  for(auto x : input_state_)
  {
    std::cout << " " << x;
  }
  std::cout << std::endl;
}

void LeuzePhidgetDriver::spawnInputSubscribers()
{
  std::cout << "MILAN _NUMBER_OF_IK_INPUTS_ " << _NUMBER_OF_IK_INPUTS_ << std::endl;
  for (int i=0; i<_NUMBER_OF_IK_INPUTS_; i++)
  {
    std::string topic_name = "/digital_input"+ ( i<=9 ? "0"+ std::to_string(i) : std::to_string(i)  );
    input_subscribers_[i] = nh_.subscribe<std_msgs::Bool>(topic_name, 10, boost::bind( &LeuzePhidgetDriver::readInputStateCallback, this, _1, i));
    ROS_INFO_STREAM("Spawned input topic subscriber " << i << " for : " << topic_name);
    ros::spinOnce();
  }
}

void LeuzePhidgetDriver::spawnOutputPublishers()
{
  for (int i=0; i<_NUMBER_OF_IK_OUTPUTS_; i++)
  {
    std::string topic_name = "/digital_output"+ ( i<=9 ? "0"+ std::to_string(i) : std::to_string(i)  );
    output_publishers_[i] = nh_.advertise<std_msgs::Bool>(topic_name, 50);
    ROS_INFO_STREAM("Spawned output topic publisher " << i << " for : " << topic_name);
    ros::spinOnce();
  }
}


void LeuzePhidgetDriver::publishOutputState()
{
  std_msgs::Bool msg;
  for (int i=0; i<_NUMBER_OF_IK_OUTPUTS_; i++)
  {
    msg.data = output_state_[i];
    output_publishers_[i].publish(msg);
  }
}



