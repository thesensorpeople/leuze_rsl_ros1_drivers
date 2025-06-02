#ifndef LEUZE_PHIDGET_DRIVER_RSL200_H
#define LEUZE_PHIDGET_DRIVER_RSL200_H

#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "leuze_phidget_driver.hpp"

#include "leuze_msgs/PhidgetIKInputMsg_rsl200.h"
#include "leuze_msgs/PhidgetIKOutputMsg_rsl200.h"

class LeuzePhidgetDriverRsl200: public LeuzePhidgetDriver
{
public:
  LeuzePhidgetDriverRsl200(ros::NodeHandle* nh);

protected:
  void getOutputStateCallback(const leuze_msgs::PhidgetIKOutputMsg_rsl200ConstPtr &msg);
  void publishInputState();
};

#endif // LEUZE_PHIDGET_DRIVER_RSL200_H
