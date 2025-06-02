#include "leuze_phidget_driver.hpp"
#include <string>
#include "leuze_phidget_driver_rsl400.hpp"

LeuzePhidgetDriverRsl400::LeuzePhidgetDriverRsl400(ros::NodeHandle *nh):
  LeuzePhidgetDriver(nh, 11, 14)
{
  input_state_.resize(_NUMBER_OF_IK_INPUTS_);
  input_subscribers_.resize(_NUMBER_OF_IK_INPUTS_);
  output_state_.resize(_NUMBER_OF_IK_OUTPUTS_);
  output_publishers_.resize(_NUMBER_OF_IK_OUTPUTS_);

  pub_show_inputs_ = nh_.advertise<leuze_msgs::PhidgetIKInputMsg_rsl400>("ik_show_inputs",50);
  sub_get_outputs_ = nh_.subscribe<leuze_msgs::PhidgetIKOutputMsg_rsl400>("ik_set_outputs",10, boost::bind(&LeuzePhidgetDriverRsl400::getOutputStateCallback, this, _1));

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

void LeuzePhidgetDriverRsl400::publishInputState()
{
  leuze_msgs::PhidgetIKInputMsg_rsl400 msg;
  msg.i_0_EA1 = input_state_[0];
  msg.i_1_OSSDA1= input_state_[1];
  msg.i_2_OSSDA2= input_state_[2];
  msg.i_3_MELD= input_state_[3];
  msg.i_4_A1= input_state_[4];
  msg.i_5_A2= input_state_[5];
  msg.i_6_A3= input_state_[6];
  msg.i_7_A4= input_state_[7];
  msg.i_8_EA2= input_state_[8];
  msg.i_9_OSSDB1= input_state_[9];
  msg.i_10_OSSDB2= input_state_[10];
  msg.header.stamp = ros::Time::now();
  pub_show_inputs_.publish(msg);
}


void LeuzePhidgetDriverRsl400::getOutputStateCallback(const leuze_msgs::PhidgetIKOutputMsg_rsl400ConstPtr &msg)
{
  output_state_[0] = msg->o_0_RES1;
  output_state_[1] = msg->o_1_F1;
  output_state_[2] = msg->o_2_F2;
  output_state_[3] = msg->o_3_F3;
  output_state_[4] = msg->o_4_F4;
  output_state_[5] = msg->o_5_F5;
  output_state_[6] = msg->o_6_SE1;
  output_state_[7] = msg->o_7_SE2;
  output_state_[8] = msg->o_8_F6;
  output_state_[9] = msg->o_9_F7;
  output_state_[10] = msg->o_10_F8;
  output_state_[11] = msg->o_11_F9;
  output_state_[12] = msg->o_12_F10;
  output_state_[13] = msg->o_13_RES2;
  publishOutputState();
  ros::spinOnce();
}

