#include "leuze_phidget_driver.hpp"
#include <string>
#include "leuze_phidget_driver_rsl200.hpp"

LeuzePhidgetDriverRsl200::LeuzePhidgetDriverRsl200(ros::NodeHandle *nh):
  LeuzePhidgetDriver(nh, 5, 7)
{
  input_state_.resize(_NUMBER_OF_IK_INPUTS_);
  input_subscribers_.resize(_NUMBER_OF_IK_INPUTS_);
  output_state_.resize(_NUMBER_OF_IK_OUTPUTS_);
  output_publishers_.resize(_NUMBER_OF_IK_OUTPUTS_);

  pub_show_inputs_ = nh_.advertise<leuze_msgs::PhidgetIKInputMsg_rsl200>("ik_show_inputs",50);
  sub_get_outputs_ = nh_.subscribe<leuze_msgs::PhidgetIKOutputMsg_rsl200>("ik_set_outputs",10, boost::bind(&LeuzePhidgetDriverRsl200::getOutputStateCallback, this, _1));

  spawnInputSubscribers();
  spawnOutputPublishers();

  ROS_INFO_STREAM("Waiting for inputs...");

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    publishInputState();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void LeuzePhidgetDriverRsl200::publishInputState()
{
  leuze_msgs::PhidgetIKInputMsg_rsl200 msg;
  msg.i_0_OSSD1= input_state_[0];
  msg.i_1_OSSD2= input_state_[1];
  msg.i_2_WF1VIO= input_state_[2];
  msg.i_3_WF2VIO= input_state_[3];
  msg.i_4_PFVIO= input_state_[4];
  msg.header.stamp = ros::Time::now();
  pub_show_inputs_.publish(msg);
}


void LeuzePhidgetDriverRsl200::getOutputStateCallback(const leuze_msgs::PhidgetIKOutputMsg_rsl200ConstPtr &msg)
{
  output_state_[0] = msg->o_0_RES1;
  output_state_[1] = msg->o_1_F1;
  output_state_[2] = msg->o_2_F2;
  output_state_[3] = msg->o_3_F3;
  output_state_[4] = msg->o_4_F4;
  output_state_[5] = msg->o_5_F5;
  output_state_[6] = msg->o_6_F6;
  publishOutputState();
  ros::spinOnce();
}


