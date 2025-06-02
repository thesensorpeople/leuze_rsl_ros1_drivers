// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL200_INTERFACE_H
#define LEUZE_RSL200_INTERFACE_H

#include "interface.hpp"

#include "leuze_msgs/ExtendedStatusProfileMsg_rsl200.h"

class RSL200Interface : public Interface
{
public:
    RSL200Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh);
    ~RSL200Interface();

protected:
    void resetDefault() override;
    void verifyConfiguration(DatagramExtendedStatusProfile_rsl200 d_esp);
    void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) override;
    void publishScan() override;

private:
    leuze_msgs::ExtendedStatusProfileMsg_rsl200 status_msg_;
};

#endif
