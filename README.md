# How to Use the VDA-5050-Connector

## Aim of the Repository
The idea of the repository is to ensure an easy connection of the VDA 5050 with ROS. 
Unlike some other repos, we do not offer an abstract implementation, but allow an immediate integration into ROS.
The only thing that needs to be customized for use is a configuration file in which the respective ROS Topics can be individualized. 

## Prerequisites
* This project was developed and tested under Ubuntu 18.04 LTS and ROS Melodic
* We assume that you have installed and configured ROS as well created your first workspace (e. g. catkin_ws)
* install the ROS MQTT bridge: http://wiki.ros.org/mqtt_bridge
* configure the MQTT Bridge, e. g. change config endpoints, certificate storage location and topics.

## Steps to Install the Connector
go to your catkin_ws and switch to the src folder
```console
cd src
```
Clone the repository
```console
git clone https://github.com/idealworks/VDA-5050-Connector.git
```
Clone the vda_5050 ros msg repository https://github.com/ipa320/vda5050_msgs (currently not in use, since needed msg are part of this repo, we will commit this to the msg repo)
```console
git clone https://github.com/ipa320/vda5050_msgs.git
```
Build and source the workspace 
```console
cd ..
catkin_make
source devel/setup.bash
```

## How to Start the Connector
* Have a look at the config file (config/daemons_params.yaml) and change parameters for your needs
The config file is used to change the ROS topics you want to subscribe and publish.
```diff
- Do not change the key, only the value (see YAML Docu of you are not sure)
```
* start the mqtt_bridge (or equivalent if you use another tool)
* start the supervisor
```console
roslaunch vda_5050_connector supervisor.launch
```

Now you should see some output on the terminal which should looks like this:
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
Is some parameters are not readable or not found in the parameter server, there is a warning output. Please check if you have a typo in your config file

## Configuration Options
* Connection
* Topic mapping
* ...

## Known Bugs
* Currently, the console ouput is done twice for some parameters due to the architecture  

## About
The ROS-VDA-5050-Connector was developed by idealworks in cooperation with [TUM fml – future.meets.logistics](https://www.linkedin.com/company/tum-fml/).

