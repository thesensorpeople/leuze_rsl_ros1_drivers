// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__UDP_SIM_HPP_
#define LEUZE_RSL_DRIVER__UDP_SIM_HPP_

#include <iostream>
#include <string>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>


class UdpSim
{
public:
  static void data_generator(
    boost::function<void(std::basic_string<unsigned char>)>
    handle_read);
};

#endif  //  LEUZE_RSL_DRIVER__UDP_SIM_HPP_