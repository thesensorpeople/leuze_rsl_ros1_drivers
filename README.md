# Leuze ROS1 drivers

This stack contains all packages of the ROS1 driver for the Leuze RSL 200 and RSL 400 laser scanners.

*This driver is based on / is a fork of the RSL ROS driver by Fraunhofer IPA (Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation) which can be found here:
https://gitlab.cc-asp.fraunhofer.de/led/leuze_ros_drivers*


## Installation (from source)

Activate the ROS1 Melodic installation with:
```
source /opt/ros/<distro>/setup.bash
```
where `<distro>` is your distribution of ROS *(kinetic/melodic)*. 
You need to repeat this command for each new terminal.

Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
```

Go to your catkin workspace and build the driver
```
cd ~/catkin_ws/src/
#Copy the complete source code of this driver into the current folder, for example by cloning it from github (Currently, you must copy the driver source code from the supplied zip file instead of git clone because the driver is not publicly available yet):
#git clone ...
#Navigate to the main workspace folder:
cd ..
#Install the required dependencies:
rosdep install --from-paths src --ignore-src -r -y
#Build the driver:
catkin_make
#Next, you need to source this workspace so that ROS2 can see the new packages:
source ~/catkin_ws/devel/setup.bash
```

## Simulation mode
You can activate the simulation mode for testing this driver without any connected real laser scanner by switching the option "SIMULATION" in leuze_rsl_driver/CMakeLists.txt from "no" to "yes":
```
#------------------SIMULATION---------------------------
set(SIMULATION "yes")     #Set to "yes" for simulation mode (default value: "no")
```

In RViz, the simulated laser scanner will appear as a dynamic sinusoidal contour within the defined topic name:
![RViz_RSL200_simulation](leuze_description/doc/RViz_RSL200_simulation.PNG?raw=true "RViz_RSL200_simulation")


## Scanner Setup

You can visit [this page](https://www.leuze.com/en/deutschland/produkte/produkte_fuer_die_arbeitssicherheit/optoelektronische_sicherheits_sensoren/sicherheits_laserscanner/rsl_4_5/selector.php?supplier_aid=53800144&grp_id=1411040450707&lang=eng#{%22tab%22:%223%22}) on the Leuze official website to download the *Sensor Studio* software tool and the *Quick Start Guide* document to help setup the communication settings of the scanner. Follow the instructions from the guide until you are successfully connected to the scanner.   

Now we need to setup the static IP address of the scanner as well as the receiving device. To do so, go to the *Settings* tab on the top and expand the *Communication* option to the left.   

![Alt text](leuze_description/doc/SensorStudio_IP1.PNG?raw=true "IP Settings")

Enter your desired static address for the scanner in the `IP address` field (this is the value you provide to the launch file as described in section *Bringup*) and the subnet mask in the `Subnet mask` field. The addresses assumed by default in this driver stack are `192.168.10.1` and `255.255.255.0` respectively.

Next, select the *Data telegrams* option to the left within the same tab. This allows us to setup the various settings for the UDP telegrams.   

![Alt text](leuze_description/doc/SensorStudio_IP2.PNG?raw=true "UDP Settings")

The various settings are :
* `UDP Telegram` : Ensure this is set to `Active` so we can actually receive data in the driver.   
* `Destination` : Should be set to `IP address`.   
* `IP address` : This refers to the IP address of the device receiving the datagrams (i.e. the device running this driver stack). You can set this to any desired value, the default assumed by this stack is `192.168.10.2`.  
* `Device name` : Enter any name you wish.   
* `Port` : You can enter any value you wish. The default assumed by this driver stack is `9990`.   
* `Measurement value transmission` : Ensure set to `Active`.   
* `Data type` : This allows you to select between `ID: 6` for Distance only or `ID: 3` for Distance+Signal Strength. Both are supported by this driver.
* The next 3 fields allow you to setup the scan area and resolution. Once you set these values, make sure to also update them in `leuze_rsl_driver/config/params.yaml` as well (albeit converted to radians). Failing to update the values only shows a warning during execution, but does not impair functionality. The default values can be seen in the image above and the *yaml* file respectively.


Once these settings have been updated, they need to be written to the device. You can do so by clicking on the small blue down arrow button at the top. Only the communication settings need to be updated, so select only this parameter in the ensuing dialog box. To upload any settings, you need to use the `Engineer` profile, and the password is `safety` be default. **PLEASE BE SURE OF ANY SETTINGS YOU UPLOAD THIS WAY.** Once you change the IPs, you may need to restart the scanner as well as the receiving device to reconnect.

## Receving device Setup

Once the IPs have been setup correctly on the scanner, setting up the receiving device is relatively straightforward. You only need to create a new Wired connection with a desired name. The IPv4 address should be the IP address you entered in *Settings>Data telegrams>IP address* in Sensor Studio, the default value assumed by this driver stack is `192.168.10.2`. The subnet mask would then similarly be `255.255.255.0` as previously setup in *Settings>Communication>LAN*.


## Packages description
`leuze_bringup` : Contains the launch files for starting the ROS driver, main point of entry to this stack.   
`leuze_description` : Contains the URDF of the scanner and the launch file for independently viewing it.   
`leuze_msgs` : Contains all the custom messages required for internal functionality.  
`leuze_phidget_driver` : Contains the Phidget IK driver package to read the I/Os of the scanner.   
`leuze_ros_drivers` : Metapackage of this stack.   
`leuze_rsl_driver` : Contains the main driver source code and its tests.   

## Bringup
You can start the Leuze RSL ROS driver for RSL200 by running :   
```
roslaunch leuze_bringup leuze_bringup_rsl200.launch sensor_ip:=<sensor ip> port:=<port> topic:=<topic name>
```

or (RSL400)
```
roslaunch leuze_bringup leuze_bringup_rsl400.launch sensor_ip:=<sensor ip> port:=<port> topic:=<topic name>
```

You can also run the RSL200 driver and RSL400 driver in parallel by using different topic names, for example:
```
roslaunch leuze_bringup leuze_bringup_rsl200.launch sensor_ip:=192.168.20.5 port:=9991 topic:=scan1
roslaunch leuze_bringup leuze_bringup_rsl400.launch sensor_ip:=192.168.20.7 port:=9992 topic:=scan2
```

If you want to run multiple instances of the driver for the same scanner type (e.g. two RSL200 laser scanners), you must create an additional .lauch file by cloning the existing one (e.g. leuze_bringup_rsl200_1) in the package leuze_bringup and use different node names in each .launch file (e.g. leuze_rsl200_node and leuze_rsl200_node_2).
Example:
```
roslaunch leuze_bringup leuze_bringup_rsl200.launch sensor_ip:=192.168.20.5 port:=9991 topic:=scan1
roslaunch leuze_bringup leuze_bringup_rsl200_1.launch sensor_ip:=192.168.20.7 port:=9992 topic:=scan2
```

#### Parameters
- `sensor_ip` : The IPv4 address of the laser scanner. This can be configured from the Sensor Studio software tool in Windows. The scanner also displays its currently configured IP during power on startup.   
- `port`: The port number of the scanner. Can be similarly configured on Sensor Studio, but not displayed on the sensor during startup.
- `topic`: The name of a topic the laser scanner should use to publish its measurement data (e.g. "scan1"). The driver appears under this name in the RViz tool.

> For more information on how to setup the IP and Port values of the scanner using Sensor Studio, see section *Scanner Setup*.   

Further parameters are defined in the corresponding .YAML files:

leuze_rsl_driver/config/params_rsl200.yaml
leuze_rsl_driver/config/params_rsl400.yaml

These additional parameters include:
- `scan_size`:  Number of beams in a single scann
- `angle_min`:  Lowest possible angle: -135° (-3*pi/4 rad)
- `angle_max`:  Highest possible angle: +135° (+3*pi/4 rad)
- `scan_time`:  Period of the laser scanner
- `range_min`:  Minimum measurable distance
- `range_max`:  Maximum measurable distance
- `scan_frame`: Laser scan frame ID for further transformations

*Note*
If you only want RViz to visualize the measured contour without defining any additional transformation, you must set the Fixed_frame property in RViz to the exact name of the Laser scan frame ID (scan_frame) defined above.

You can use RViz to see the the scanner contour. You'll need to select the scanner by its topic (e.g. "scan1") and set the Fixed Frame parameter to "scanner_laser" (or to whatever value specivied in the corresponding YAML file):
```
$ rviz
```


## Mount link
You can view a 3D model of a laser scanner directly in RViz by running the following command (Currently available for RSL400 only):
```
roslaunch leuze_description view_rsl400.launch
```

![RViz_RSL400_mount_link](leuze_description/doc/RViz_RSL400_mount_link.PNG?raw=true "RViz_RSL400_mount_link")

You can choose between two fixed frames:
- `scanner_laser`: Laser beams will be positioned at height 0.
- `scanner_mount_link `: Laser beams will be positioned at a certain height as if a real laser scanner were mounted on the floor.


## Phidget driver

You can use the Phidget Interface Kit drivers in order to interface with the I/Os of the scanner. It can be found [here](https://github.com/ros-drivers/phidgets_drivers). If you wish to utilize this feature, either install the Phidget driver from source by cloning it to the same workspace and building it, or install it directly as a Debian binary package:   

```
sudo apt install ros-<distro>-phidgets-ik
```
where `<distro>` is your distribution of ROS *(kinetic/melodic)*.   

You can make sure you have all other needed dependencies by running the following from your workspace directory:   
```
rosdep install --from-paths src --ignore-src -r -y
```

Launch the driver
```
# for RSL200
roslaunch leuze_bringup leuze_phidget_driver_rsl200.launch

# for RSL400
roslaunch leuze_bringup leuze_phidget_driver_rsl400.launch
```

You can view the topics registered by the driver using the following command (You should get 5 inputs and 7 outputs for RSL200, and 11 inputs and 14 outputs for RSL400):
```
rostopic list
```


## Unit test and code verification
Use the following commands to perform to run unit tests:
```
catkin_make run_tests
catkin_test_results build/test_results
```

Use the following commands to perform code verification (roslint): 
```
catkin_make roslint_leuze_rsl_driver
```