# How to Use the VDA-5050-Connector

## Prerequisites
* This project was developed and tested under Ubuntu 18.04 LTS and ROS Melodic
* We assume that you have installed and configured ROS as well created your first workspace (e. g. catkin_ws)
* install rosbridge: http://wiki.ros.org/mqtt_bridge
* configure rosbridge, e. g. change config endpoints, license storage locations and topics.

## Steps to Install the Connector
*go to yout catkin_ws and change to the src folder
```console
cd src
```
* Clone the repsonitory
```console
git clone https://github.com/idealworks/VDA-5050-Connector.git
```
* Clone the vda_5050 ros msg repository https://github.com/ipa320/vda5050_msgs (currently not in use, since needed msg are part of this repo, we will commit this to the msg repo)
```console
git clone https://github.com/ipa320/vda5050_msgs.git
```
* build and source the workspace 
```console
cd ..
catkin_make
source devel/setup.bash
```

## How to Start the Connector
* Have a look at the config and change parameters for your needs
  * Change endpoint parameters, add certificates etc.
* start the supervisor
* ```console
roslaunch vda_5050_connector supervisor.launch
```

## Configuration Options
* Connection
* Topic mapping
* ...

## About
The ROS-VDA-5050-Connector was developed by idealworks in cooperation with [TUM fml – future.meets.logistics](https://www.linkedin.com/company/tum-fml/).
