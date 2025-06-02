// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include <ros/ros.h>
#include <string>

#if defined(RSL200)
  #include "leuze_rsl_driver/rsl200_interface.hpp"
#elif defined(RSL400)
  #include "leuze_rsl_driver/rsl400_interface.hpp"
#endif

int main(int argc, char *argv[])
{
#if defined(RSL200)
    ros::init(argc, argv, "leuze_driver_rsl200");
#elif defined(RSL400)
    ros::init(argc, argv, "leuze_driver_rsl200");
#endif
    ros::NodeHandle n;
    if (argc < 4)
    {
        std::cerr << "Not enough arguments!" << std::endl;
    }
    std::string address = argv[1];
    std::string port = argv[2];
    std::string topic = argv[3];

    std::cout << "address: " << address << std::endl;
    std::cout << "port: " << port << std::endl;
    std::cout << "topic: " << topic << std::endl;

#if defined(RSL200)
    RSL200Interface rsl_interface(address, port, topic, &n);
#elif defined(RSL400)
    RSL400Interface rsl_interface(address, port, topic, &n);
#endif
    rsl_interface.connect();

    ros::spin();
    return 0;
}
