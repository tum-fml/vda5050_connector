# How to Use the VDA-5050-Connector

## Aim of the Repository
The idea of the repository is to ensure an easy connection of the VDA 5050 with ROS. 
Unlike some other repos, we do not offer an abstract implementation, but allow an immediate integration into ROS.
The only thing that needs to be customized for use is a configuration file in which the respective ROS Topics can be individualized. 
![grafik](https://user-images.githubusercontent.com/36477083/177792573-6563f79d-95aa-4625-bcf2-bbb27b105b54.png)

## Prerequisites
* This project was developed and tested under Ubuntu 18.04 LTS and ROS Melodic.
* We assume that you have installed and configured ROS and created your first workspace (e.g. `catkin_ws`).
The communication needs to be bridged between ROS and MQTT, so you need a ROS node which passes on the messages in both directions. We suggest using the [ROS MQTT bridge](http://wiki.ros.org/mqtt_bridge) package for this, and provide a configuration (see below for details).

<details>
  <summary>Example installation of the ROS MQTT bridge</summary>

As an example, we describe how to configurate the MQTT bridge to work with AWS.

Clone the Aws IoT Bridge Example into your catkin_ws:
```console
cd src
git clone https://gitlab.lrz.de/vda5050-connector/aws-iot-bridge-example.git
```
Clone the mqtt-bridge (check, if you need the python 2.7 branch, depending of your ROS Distro)
```console
git clone https://gitlab.lrz.de/vda5050-connector/aws-iot-bridge-example.git
```
install the additional requirements if needed to run the bridge (see https://github.com/groove-x/mqtt_bridge#prerequisites).  
Now we have to modify several files to make the mqtt bridge run with AWS.  
In aws-io-bridge-example module:  
create a new config file (in our case "aws_iot_params.yaml") within the aws_iot_mqtt_bridge/config folder and add the needed tls configuration:
```
tls:
  ca_certs: <path_to_your_root_certificate>
  certfile: <path_to_your_key_certificate>
  keyfile: <your_path_to_private_key"
  tls_version: 5
  tls_insecure: false
connection:
  host: <your_hostname>
  port: 8883
  keepalive: 60
client:
  protocol: 4
  client_id: <your_client_id>
```
create an additional config file (in our case "bridge.yaml") within the aws_iot_mqtt_bridge/config folder and add the needed mqtt-bridge configuration:
```
- factory: mqtt_bridge.bridge:RosToMqttBridge
  msg_type: <your_msg_type>
  topic_from: <your_topic>
  topic_to: <your_topic>
- factory: mqtt_bridge.bridge:MqttToRosBridge
  msg_type: <your_msg_type>
  topic_from: <your_topic>
  topic_to: <your_topic>
  ...
```
Your folder aws_iot_mqtt_bridge/config should contain two files:
* aws_iot_params.yaml
* bridge.yaml


Finally, define your own launch file within the aws-io-bridge-example module and add the following
```
<launch>
  <arg name="bridge_params" />
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam command="load" ns="mqtt" file="$(find aws_iot_mqtt_bridge)/config/aws_iot_params.yaml" />
    <rosparam command="load" ns="bridge" file="$(find aws_iot_mqtt_bridge)/config/bridge.yaml" />
  </node>
</launch>
```
In your folder aws_iot_mqtt_bridge/launch shoud be one launch file:
* aws_iot_bridge.launch

Make and test the connection:
```console
cd ..
catkin_make
source devel/setup.bash
roslaunch aws_iot_mqtt_bridge aws_iot_bridge.launch
```
The output of you terminal should be something like this:
```console

started roslaunch server http://<your_server>

SUMMARY
========

PARAMETERS
 * /mqtt_bridge/bridge: [{'topic_from': '...
 * /mqtt_bridge/mqtt/client/client_id: <your_ID>
 * /mqtt_bridge/mqtt/client/protocol: 4
 * /mqtt_bridge/mqtt/connection/host: <your_hostname>
 * /mqtt_bridge/mqtt/connection/keepalive: 60
 * /mqtt_bridge/mqtt/connection/port: 8883
 * /mqtt_bridge/mqtt/tls/ca_certs: <your_root_path>
 * /mqtt_bridge/mqtt/tls/certfile: <your_cert_path>
 * /mqtt_bridge/mqtt/tls/keyfile: <your_private_keyfile_path>
 * /mqtt_bridge/mqtt/tls/tls_insecure: False
 * /mqtt_bridge/mqtt/tls/tls_version: 5
 * /rosdistro: melodic
 * /rosversion: 1.14.12

NODES
  /
    mqtt_bridge (mqtt_bridge/mqtt_bridge_node.py)

auto-starting new master
process[master]: started with pid [23084]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to cd2855b8-fd29-11ec-bfbb-080027dacaf0
process[rosout-1]: started with pid [23095]
started core service [/rosout]
process[mqtt_bridge-2]: started with pid [23098]
[INFO] [1657111653.836246]: MQTT connected
```
</details>
After you successfully installed and configured your MQTT bridge, lets head over to the VDA5050-Connector.

## Steps to Install the Connector
Go to your `catkin_ws` and switch to the src folder:
```console
cd src
```
Clone this repository:
```console
git clone https://github.com/idealworks/VDA-5050-Connector.git
```
Clone the `vda5050_msgs` repository:
```console
git clone https://github.com/ipa320/vda5050_msgs.git
```
**Note**
Currently, we did not send a merge request to the vda5050_msgs repository and some additional msgs are part of our repository. In order to use our msgs, do the following after cloning the vda5050_msgs repository:
* copy the .msg files found in /msg from VDA-5050-Connector (this repo) into the /msg folder of the vda5050_msgs repository
* replace the CmakeLists.txt in the vda5050_msgs repository with the one within the /msg/CmakeLists of the VDA-5050-Connector.

Build and source the workspace:
```console
cd ..
catkin_make
source devel/setup.bash
```

## How to Start the Connector
1. Have a look at the config file `config/daemons_params.yaml` and change parameters for your needs.
The config file is used to change the ROS topics you want to subscribe and publish.
```diff
- Do not change the key, only the value (see YAML specification if you are not sure)
```
2. Start the mqtt_bridge (or equivalent if you use another tool).
3. Start the supervisor:
```console
roslaunch vda_5050_connector supervisor.launch
```

Now you should see some output on the terminal which should look like this:
```console
[ INFO] [1656490735.862272168]: Using 1.1.03 for parameter ~AGV_Data/version
[ INFO] [1656490735.863479005]: Using template_1 for parameter ~AGV_Data/manufacturer
[ INFO] [1656490735.864071660]: Using AGV721 for parameter ~AGV_Data/serialNumber
[ INFO] [1656490735.865060948]: Using /error_1 as error topic
[ INFO] [1656490735.866858541]: for connection_daemon/topics_publish use:
[ INFO] [1656490735.868006928]: for connection_daemon/topics_subscribe use:
[ INFO] [1656490735.868029030]:     - parameter: /supervisor/connection_daemon/topics_subscribe/connectionState value: /connected
[ INFO] [1656490735.868595221]: Using uagv for parameter ~AGV_Data/interfaceName
[ INFO] [1656490735.869125729]: Using v2 for parameter ~AGV_Data/majorVersion
[ INFO] [1656490735.869653581]: Using /connected for parameter ~connection_daemon/topics_subscribe/connectionState
[ INFO] [1656490735.872009204]: Using 1.1.03 for parameter ~AGV_Data/version
[ INFO] [1656490735.872491504]: Using template_1 for parameter ~AGV_Data/manufacturer
[ INFO] [1656490735.872921588]: Using AGV721 for parameter ~AGV_Data/serialNumber
[ INFO] [1656490735.873329962]: Using /error_1 as error topic
[ INFO] [1656490735.874538560]: for state_daemon/topics_publish use:
[ INFO] [1656490735.874560377]:     - parameter: /supervisor/state_daemon/topics_publish/state value: /state
[ INFO] [1656490735.893055466]: for state_daemon/topics_subscribe use:
[ INFO] [1656490735.893084950]:     - parameter: /supervisor/state_daemon/topics_subscribe/actionStates value: /actionStates
[ INFO] [1656490735.893092672]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/deviationRange value: /deviationRange
[ INFO] [1656490735.893108816]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/localizationScore value: /localizationScore
[ INFO] [1656490735.893114964]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/mapDescription value: /mapDescription
[ INFO] [1656490735.893126128]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/mapId value: /mapId
[ INFO] [1656490735.893138406]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/pose value: /odometry
[ INFO] [1656490735.893146836]:     - parameter: /supervisor/state_daemon/topics_subscribe/agvPosition/positionInitialized value: /positionInitialized
[ INFO] [1656490735.893154395]:     - parameter: /supervisor/state_daemon/topics_subscribe/batteryState/batteryCharge value: /batteryCharge
[ INFO] [1656490735.893162173]:     - parameter: /supervisor/state_daemon/topics_subscribe/batteryState/batteryHealth value: /batteryHealth
[ INFO] [1656490735.893169512]:     - parameter: /supervisor/state_daemon/topics_subscribe/batteryState/batteryVoltage value: /batteryVoltage
[ INFO] [1656490735.893177634]:     - parameter: /supervisor/state_daemon/topics_subscribe/batteryState/charging value: /charging
[ INFO] [1656490735.893185112]:     - parameter: /supervisor/state_daemon/topics_subscribe/batteryState/reach value: /reach
[ INFO] [1656490735.893192605]:     - parameter: /supervisor/state_daemon/topics_subscribe/distanceSinceLastNode value: /distanceSinceLastNode
[ INFO] [1656490735.893200330]:     - parameter: /supervisor/state_daemon/topics_subscribe/driving value: /driving
[ INFO] [1656490735.893207638]:     - parameter: /supervisor/state_daemon/topics_subscribe/edgeStates value: /edgeState
[ INFO] [1656490735.893216187]:     - parameter: /supervisor/state_daemon/topics_subscribe/errors value: /errors
[ INFO] [1656490735.893224128]:     - parameter: /supervisor/state_daemon/topics_subscribe/information value: /information
[ INFO] [1656490735.893231756]:     - parameter: /supervisor/state_daemon/topics_subscribe/lastNodeId value: /lastNodeId
[ INFO] [1656490735.893239590]:     - parameter: /supervisor/state_daemon/topics_subscribe/lastNodeSequenceId value: /lastNodeSequenceId
[ INFO] [1656490735.893246911]:     - parameter: /supervisor/state_daemon/topics_subscribe/loads value: /loads
[ INFO] [1656490735.893255322]:     - parameter: /supervisor/state_daemon/topics_subscribe/newBaseRequest value: /newBaseRequest
[ INFO] [1656490735.893262908]:     - parameter: /supervisor/state_daemon/topics_subscribe/nodeStates value: /nodeState
[ INFO] [1656490735.893272540]:     - parameter: /supervisor/state_daemon/topics_subscribe/operatingMode value: /operatingMode
[ INFO] [1656490735.893280114]:     - parameter: /supervisor/state_daemon/topics_subscribe/orderId value: /orderId
[ INFO] [1656490735.893287369]:     - parameter: /supervisor/state_daemon/topics_subscribe/orderUpdateId value: /orderUpdateId
[ INFO] [1656490735.893296333]:     - parameter: /supervisor/state_daemon/topics_subscribe/paused value: /paused
[ INFO] [1656490735.893304115]:     - parameter: /supervisor/state_daemon/topics_subscribe/safetyState/eStop value: /eStop
[ INFO] [1656490735.893311934]:     - parameter: /supervisor/state_daemon/topics_subscribe/safetyState/fieldViolation value: /fieldViolation
[ INFO] [1656490735.893320184]:     - parameter: /supervisor/state_daemon/topics_subscribe/velocity value: /rosVelocity
[ INFO] [1656490735.893327443]:     - parameter: /supervisor/state_daemon/topics_subscribe/zoneSetId value: /zoneSetId
[ INFO] [1656490735.893811918]: Using uagv for parameter ~AGV_Data/interfaceName
[ INFO] [1656490735.894266591]: Using v2 for parameter ~AGV_Data/majorVersion
```
The output gives you an overview over all parameters read from the config file, so you can check if everything is as you want it.
If any parameters are not readable or not found in the parameter server, there is a warning output. Please check if you have a typo in your config file.

## Configuration Options
Please see the contents of `config\daemons_params.yaml` for configuration options.

## Known Issues
* Currently, the console ouput is done twice for some parameters due to the architecture.
* Sometimes the MQTT-Bridge has troubles to connect. The output is like this:
```console
   [INFO] [1657111653.836246]: MQTT disconnected
   [INFO] [1657111653.836246]: MQTT connected
```
Even there should be a connection, due to an internal bug the bridge is not working properly if the first connection is disconnected and connected afterwards.
The solution is to restart the MQTT bridge until there is noch disconnected before connected

## About
The ROS-VDA-5050-Connector was developed by idealworks in cooperation with [TUM fml – future.meets.logistics](https://www.linkedin.com/company/tum-fml/).

