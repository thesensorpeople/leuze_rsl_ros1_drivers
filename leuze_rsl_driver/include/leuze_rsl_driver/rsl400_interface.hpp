// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL400_INTERFACE_H
#define LEUZE_RSL400_INTERFACE_H

#include "interface.hpp"

#include "leuze_msgs/ExtendedStatusProfileMsg_rsl400.h"

class RSL400Interface : public Interface
{
public:
    RSL400Interface(std::string address, std::string port, std::string topic, ros::NodeHandle* nh);
    ~RSL400Interface();

protected:
    void resetDefault() override;
    void verifyConfiguration(DatagramExtendedStatusProfile_rsl400 d_esp);
    void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) override;
    void publishScan() override;

private:
    leuze_msgs::ExtendedStatusProfileMsg_rsl400 status_msg_;
};

#endif
