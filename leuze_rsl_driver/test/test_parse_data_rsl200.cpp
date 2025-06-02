#include "leuze_rsl_driver/rsl200_interface.hpp"
#include <gtest/gtest.h>


struct RSL200TestParseData : public testing::Test
{
    std::basic_string<unsigned char> test_buffer;
    ros::NodeHandle nh;
    RSL200Interface *interface;
    void SetUp()
    {
      nh.setParam("/test_parse_data/scanner_frame","test_frame");
      interface = new RSL200Interface("192.168.0.1","9990", "scan2", &nh);
    }
    void TearDown()
    {
        delete interface;
    }

};


TEST_F(RSL200TestParseData, test_id_1)
{
    test_buffer.resize(56);
    ros::NodeHandle nh;

    test_buffer = {
            0x38, 0x00,                            //Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    //Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,                //Header H2
            0x01, 0x00,                            //ID = 1 (Extended status profile)
            0x00, 0x00,                            //Block number
            0x6e ,0x29 ,0x05 ,0x00 ,               //Scan
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Status profile
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x46, 0x05,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00 //Reserved
            };  //Captured from wireshark, the datagram extended status profile

    EXPECT_EQ(interface->parseBuffer(test_buffer) , 1);
}


TEST_F(RSL200TestParseData, test_id_3)
{
    test_buffer.resize(56);
    ros::NodeHandle nh;

    test_buffer = {
            0x38, 0x00,                            //Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    //Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,                //Header H2
            0x03, 0x00,                            //ID = 3 (Meas data with both distances and amplitude)
            0x00, 0x00,                            //Block number
            0x6e ,0x29 ,0x05 ,0x00 ,               //Scan
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Status profile
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x46, 0x05,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00 //Reserved
            }; //Captured from wireshark, wrong type of datagram for ID 3, but we only test ID control flow here so it's okay

    EXPECT_EQ(interface->parseBuffer(test_buffer) , 3);
}


TEST_F(RSL200TestParseData, test_id_6)
{
    test_buffer.resize(56);
    ros::NodeHandle nh;
    test_buffer = {
            0x38, 0x00,                            //Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    //Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,                //Header H2
            0x06, 0x00,                            //ID = 6 (Meas data with distances only)
            0x00, 0x00,                            //Block number
            0x6e ,0x29 ,0x05 ,0x00 ,               //Scan
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Status profile
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x46, 0x05,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00 //Reserved
            }; //Captured from wireshark, wrong type of datagram for ID 3, but we only test ID control flow here so it's okay

    EXPECT_EQ(interface->parseBuffer(test_buffer) , 6);
}


//Unrecogized ID
TEST_F(RSL200TestParseData, test_id_4)
{
    test_buffer.resize(56);
    ros::NodeHandle nh;
    test_buffer = {
            0x38, 0x00,                            //Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    //Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,                //Header H2
            0x04, 0x00,                            //ID = 4 (Unknown ID)
            0x00, 0x00,                            //Block number
            0x6e ,0x29 ,0x05 ,0x00 ,               //Scan
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Status profile
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x46, 0x05,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00 //Reserved
            };  //Captured from wireshark, the datagram extended status profile

    EXPECT_EQ(interface->parseBuffer(test_buffer) , -1);
}


