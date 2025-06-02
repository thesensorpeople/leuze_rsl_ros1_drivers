// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/udp_sim.hpp"
#include <string>


void UdpSim::data_generator(boost::function<void(std::basic_string<unsigned char>)> handle_read)
{
  // This is a simulation => Generate dummy frames
  std::basic_string<unsigned char> test_buffer;

  uint32_t scan_number = 0;

  while (true)
  {
#if defined (RSL200)

        // State image (Frame 0 - first frame)
        test_buffer.resize(56);
        test_buffer =
        {
            0x38, 0x00,                            // Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    // Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,                // Header H2
            0x01, 0x00,                            // ID = 1 (Extended status profile)
            0x00, 0x00,                            // Block number
            // Scan number
            static_cast<unsigned char>(scan_number & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8)  & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),
            // Status profile
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11,
            0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x46, 0x05,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00   // Reserved
        };
        handle_read(test_buffer);

        // Meas data (Frames 1 to 4)
        for (unsigned char meas_frame_counter = 0; meas_frame_counter < 4; meas_frame_counter++)
        {
            test_buffer.clear();

            // Calculate number of beams in this frame:
            uint16_t beam_cout = 720;  // Maximum number of beams in a full frame
            if (meas_frame_counter == 3)
            {
                // This is the last beam which contains fewer beams:
                beam_cout = 542;
            }

            // Calculate total frame size
            // We must multiply the beams by 2 because each beam takes up 2 bytes;
            // 20 ist the size of the header, example: 1460 for a full frame
            uint16_t frame_size = beam_cout*2 + 20;


            // Fill header
            test_buffer =
            {
                static_cast<unsigned char>(frame_size & 0xFF),  // H1 (Frame size)
                static_cast<unsigned char>(frame_size >> 8),    // H1 (rest)
                0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,     // H1
                0x04, 0x15, 0x02, 0xc8,                 // H2
                0x03,  0x00,                            // ID
                meas_frame_counter, 0x00,               // Block number
                // Scan number
                static_cast<unsigned char>(scan_number & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8)  & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),
            };

            test_buffer.resize(frame_size);

            // Cast the buffer from 8-bit array into a 16-bit array, skip the header:
            uint16_t* test_buffer_as_shorts = reinterpret_cast<uint16_t*>(&test_buffer[20]);

            // Fill meas data with a simulation pattern:
            for (int i = 0; i < beam_cout; i++)
            {
                test_buffer_as_shorts[i] = 1000 + static_cast<int16_t>(100*sin(i + scan_number));
            }
            handle_read(test_buffer);
        }

        sleep(0.025);   // wait 25ms (real cycle time of RSL200)



#elif defined(RSL400)

        // State image (Frame 0 - first frame)
        test_buffer.resize(48);
        test_buffer =
        {
            0x30, 0x00,                          // Header H1 (Frame size)
            0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,  // Header H1 (rest)
            0x04, 0x15, 0x02, 0xc8,              // Header H2
            0x01, 0x00,                          // ID = 1 (Extended status profile)
            0x00, 0x00,                          // Block number
            // Scan number 0. - 3. byte:
            static_cast<unsigned char>(scan_number & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8) & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),
            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),
            // Status profile:
            0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11,
            0x00, 0xf0, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,  // Measurement Contour Description (start index)
            0x8b, 0x0a,  // Measurement Contour Description (stop index)
            0x01, 0x00,  // Measurement Contour Description (beam step)
            0x00, 0x00   // Reserved
        };

        handle_read(test_buffer);

        // Meas data (Frames 1 to 8)
        for (unsigned char meas_frame_counter = 0; meas_frame_counter < 8; meas_frame_counter++)
        {
            test_buffer.clear();

            // Calculate number of beams in this frame:
            uint16_t beam_cout = 720;  // Maximum number of beams in a full frame
            if (meas_frame_counter == 7)
            {
                // This is the last beam which contains fewer beams:
                beam_cout = 360;
            }

            // Calculate total frame size
            // We must multiply the beams by 2 because each beam takes up 2 bytes;
            // 20 ist the size of the header, example: 1460 for a full frame
            uint16_t frame_size = beam_cout * 2 + 20;

            // Fill header
            test_buffer =
            {
                // H1 (Frame size):
                static_cast<unsigned char>(frame_size & 0xFF),
                static_cast<unsigned char>(frame_size >> 8),
                0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,  // Header H1 (rest)
                0x04, 0x15, 0x02, 0xc8,              // Header H2
                0x03, 0x00,                          // ID = 3 (distances and amplitudes)
                meas_frame_counter, 0x00,            // Block number
                // Scan number 0. - 3. byte:
                static_cast<unsigned char>(scan_number & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8) & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),
                static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),
            };

            test_buffer.resize(frame_size);

            // cast the buffer from 8-bit array into a 16-bit array, skip the header:
            uint16_t * test_buffer_as_shorts = reinterpret_cast<uint16_t *>(&test_buffer[20]);

            // Fill meas data with a simulation pattern:
            for (int i = 0; i < beam_cout; i++)
            {
                test_buffer_as_shorts[i] = 1000 + static_cast<int16_t>(100 * sin(i + scan_number));
            }
            handle_read(test_buffer);
        }

        sleep(0.040);     // wait 40ms (real cycle time of RSL400)

#else
    #error "No scanner type defined! Cannot create simulation!"
#endif

        // Increase the scan number:
        scan_number++;
    }
}
