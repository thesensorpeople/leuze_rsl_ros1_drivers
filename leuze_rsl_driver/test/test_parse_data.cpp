#include "leuze_rsl_driver/rsl200_interface.hpp"
#include <gtest/gtest.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_parse_data");
    testing::InitGoogleTest(&argc, argv);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
