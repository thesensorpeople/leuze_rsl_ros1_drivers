cmake_minimum_required(VERSION 2.8.3)
project(leuze_phidget_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  leuze_msgs
  std_msgs
)

find_package(Boost REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp leuze_msgs std_msgs
)


#------------------RSL400---------------------------
add_library( rsl400_phidget_def_lib INTERFACE )

target_compile_definitions( rsl400_phidget_def_lib INTERFACE -DRSL400 )
add_executable(leuze_phidget_driver_node_rsl400 src/leuze_phidget_driver_node.cpp
                                         src/leuze_phidget_driver.cpp
                                         src/leuze_phidget_driver_rsl400.cpp)


target_link_libraries(leuze_phidget_driver_node_rsl400 ${catkin_LIBRARIES} rsl400_phidget_def_lib)
add_dependencies(leuze_phidget_driver_node_rsl400 ${catkin_EXPORTED_TARGETS})

install(TARGETS leuze_phidget_driver_node_rsl400 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})



#------------------RSL200---------------------------
add_library( rsl200_phidget_def_lib INTERFACE )

target_compile_definitions( rsl200_phidget_def_lib INTERFACE -DRSL200 )
add_executable(leuze_phidget_driver_node_rsl200 src/leuze_phidget_driver_node.cpp
                                         src/leuze_phidget_driver.cpp
                                         src/leuze_phidget_driver_rsl200.cpp)


target_link_libraries(leuze_phidget_driver_node_rsl200 ${catkin_LIBRARIES} rsl200_phidget_def_lib)
add_dependencies(leuze_phidget_driver_node_rsl200 ${catkin_EXPORTED_TARGETS})

install(TARGETS leuze_phidget_driver_node_rsl200 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
