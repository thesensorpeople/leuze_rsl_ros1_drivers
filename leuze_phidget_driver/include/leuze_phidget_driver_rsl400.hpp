#ifndef LEUZE_PHIDGET_DRIVER_RSL400_H
#define LEUZE_PHIDGET_DRIVER_RSL400_H

#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "leuze_phidget_driver.hpp"

#include "leuze_msgs/PhidgetIKInputMsg_rsl400.h"
#include "leuze_msgs/PhidgetIKOutputMsg_rsl400.h"

class LeuzePhidgetDriverRsl400: public LeuzePhidgetDriver
{
public:
  LeuzePhidgetDriverRsl400(ros::NodeHandle* nh);

protected:
  void getOutputStateCallback(const leuze_msgs::PhidgetIKOutputMsg_rsl400ConstPtr &msg);
  void publishInputState();
};

#endif // LEUZE_PHIDGET_DRIVER_RSL400_H
