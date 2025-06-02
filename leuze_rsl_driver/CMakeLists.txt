cmake_minimum_required(VERSION 2.8.3)
project(leuze_rsl_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  leuze_msgs
  angles
  rosconsole
)

find_package(Boost REQUIRED)


include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs leuze_msgs rosconsole
)


#------------------SIMULATION---------------------------
set(SIMULATION "no")     #Set to "yes" for simulation mode



#------------------RSL400---------------------------
add_library( rsl400_def_lib INTERFACE )

if(SIMULATION STREQUAL "yes")
  target_compile_definitions( rsl400_def_lib INTERFACE -DRSL400 -DSIMULATION)
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp
                    src/interface.cpp
                    src/rsl400_interface.cpp
                    src/udp_sim.cpp
                    )
else()
  target_compile_definitions( rsl400_def_lib INTERFACE -DRSL400 )
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp
                    src/interface.cpp
                    src/rsl400_interface.cpp
                    )
endif()

target_link_libraries(leuze_rsl400_driver ${catkin_LIBRARIES} rsl400_def_lib)
add_dependencies(leuze_rsl400_driver ${catkin_EXPORTED_TARGETS})



#------------------RSL200---------------------------
add_library( rsl200_def_lib INTERFACE )

if(SIMULATION STREQUAL "yes")
  target_compile_definitions( rsl200_def_lib INTERFACE -DRSL200 -DSIMULATION)
  add_executable(leuze_rsl200_driver src/leuze_rsl_driver.cpp
                    src/interface.cpp
                    src/rsl200_interface.cpp
                    src/udp_sim.cpp
                    )
else()
  target_compile_definitions( rsl200_def_lib INTERFACE -DRSL200 )
  add_executable(leuze_rsl200_driver src/leuze_rsl_driver.cpp
                    src/interface.cpp
                    src/rsl200_interface.cpp
                    )
endif()

target_link_libraries(leuze_rsl200_driver ${catkin_LIBRARIES} rsl200_def_lib)
add_dependencies(leuze_rsl200_driver ${catkin_EXPORTED_TARGETS})




install(TARGETS leuze_rsl400_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS leuze_rsl200_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslint REQUIRED)

    add_rostest_gtest(test_parse_data
                      test/test_parse_data.test
                      test/test_parse_data.cpp
                      test/test_parse_data_rsl200.cpp
                      test/test_parse_data_rsl400.cpp
                      src/interface.cpp
                      src/rsl200_interface.cpp
                      src/rsl400_interface.cpp
                      )
    target_link_libraries(test_parse_data ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_parse_data ${catkin_EXPORTED_TARGETS})

    # We must add all .cpp and .hpp files, but currently .hpp files are ignored because roslint does not support them
    file(GLOB roslint_sources ./src/**.*  ./include/leuze_rsl_driver/**.*)
    SET( roslint_sources ${roslint_sources})
  
    roslint_cpp(${roslint_sources})
endif()
