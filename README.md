# VDA-5050-Connector

developed with Ubuntu 18.04 LTS and ROS Melodic

install rosbridge: http://wiki.ros.org/mqtt_bridge

install AWSIoTPythonSDK https://github.com/aws/aws-iot-device-sdk-python

Reminder: There are two different SDKs, take care to install AWSIoTPythonSDK V1. Do not install V2

install aws-iot-mqtt-bridge https://github.com/aws-robotics/aws-iot-bridge-example. This is further called AWS example

for running the AWS example make sure to config the following files properly:

<your_catkin_ws>/src/aws-iot-bridge-example/aws_iot_mqtt_bridge/config/aws_iot_params.yaml
<your_catkin_ws>/src/aws-iot-bridge-example/aws_iot_mqtt_bridge/config/bridge.yaml

#TODO: give more information how to configure the configs

Depending on firewall configuration you need to modify the source code of the aws-iot-mqtt-bridge

originally the repo works with port 8883 for certificate based identifikation, but it also works with port 443

If port 8883 is blocked, change the config file in the aws-iot-mqtt-bridge repo to port=443
