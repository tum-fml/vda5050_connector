# How to Use the VDA-5050-Connector

## Aim of the Repository

The idea of the repository is to ensure an easy connection of the VDA 5050 with ROS.
It is intended to ensure an immediate integration into ROS.
For ease of use, you can customize all interfaces by adapting the configuration files in the /config folder to match your requirements.

The VDA5050 connector only covers ROS communication, which means that all messages coming from the main control must be published to ROS topics. Since MQTT is widely used to communicate from the main control to AGVs, an example implementation to achieve compatibility between ROS and MQTT is shown below.

<img src="https://user-images.githubusercontent.com/44091826/190145636-37ef04e3-d28b-42da-961d-d2e8e919df73.png" width="500">

## Prerequisites

* This project was tested under Ubuntu 18.04 LTS + ROS Melodic (Python 2.7) and Ubuntu 20.04 LTS + ROS Noetic (Python 3.7).
* ROS must be installed and a workspace (e.g., `catkin_ws`) must be initialized.
* The python module Inject is required to run this project. Install by running the following command :
```
pip install Inject==3.5.4
```

Since the VDA5050 connector solely relies on ROS communication, any other communication protocol must be translated accordingly.
VDA5050 specifies the use of MQTT as it is widely used for communication between the main control and AGVs. In the following we will describe a method for translating MQTT messages to ROS messages.
For this, we use the [ROS MQTT bridge](http://wiki.ros.org/mqtt_bridge) to connect to a server via TLS.
Of course, any other solution can be used as well. Names of outgoing and incoming topics can be adapted accordingly.

## Installation of the ROS MQTT bridge

Clone the `mqtt_bridge` repository and install the additional requirements to run the bridge (see <https://github.com/groove-x/mqtt_bridge#prerequisites>).

```console
git clone https://github.com/groove-x/mqtt_bridge.git
```

>**NOTE**\
Be careful to choose the correct branch. If you plan to use ROS melodic, checkout the branch called "python2.7".\
>The master branch will only work with ROS noetic.

## Installation of the VDA5050 connector

Go to your catkin workspace and cd into the src folder:

```bash
cd ./src
```

Clone the `VDA-5050-Connector` repository:

```bash
git clone https://github.com/idealworks/VDA-5050-Connector.git
```

Clone the `vda5050_msgs` repository:

```bash
git clone https://github.com/ipa320/vda5050_msgs.git
```

>**NOTE**\
Although the `vda5050_msgs` repository provides most of the required message types, some additional message types must be defined in order to provide the full functionality of VDA5050.\
Since we have not send a merge request to the vda5050_msgs repository yet, do the following after cloning the vda5050_msgs repository:
>
>* copy all .msg files in /msg of the `VDA-5050-Connector` repository into the /msg folder of the `vda5050_msgs` repository
>
>* replace the CmakeLists.txt in the `vda5050_msgs` repository with the one within the /msg/CmakeLists of the `VDA-5050-Connector` repository

After cloning all required repositories, build your catkin workspace.

## Customization of the configuration

There are three distinct parts of the configuration that must be customized to fulfill your connection requirements.

1. ROS MQTT bridge server connection ("mqtt_bridge_tls.yaml")
2. ROS MQTT bridge topic configuration ("mqtt_bridge_topics.yaml")
3. VDA5050 connector topic configuration ("vda5050_connector_topics.yaml")

Each of them is represented by a single configuration file in the /config folder.
In the following sections we will go through them step by step.

>**NOTE**\
Since we wanted to use the ROS MQTT bridge out of the box with no further customization,\
all required parameters configuration files can be found in the /config folder in the `VDA-5050-Connector` repository (and not in the `mqtt_bridge` repository).

### ROS MQTT bridge server connection

To make the ROS MQTT bridge work with TLS, complete the "mqtt_bridge_tls.yaml" configuration file in the /config folder.

<details>

<summary>TLS configuration</summary>
```text
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

</details>

Specifically, add the paths to the required certificates and the private key and enter your host address as well as your client id. Do not forget to use the correct port in your network.

### ROS MQTT bridge topic configuration

Use the file "mqtt_bridge_topics.yaml" to adapt topic names and message types according to your needs.\
Make sure to use ROS and MQTT topic names at the correct position in the configuration.

<details>

<summary>Topic example</summary>

```text
##### bridge from ROS to MQTT#####
topic_from: <ROS topic>
topic to: <MQTT topic>

##### bridge from MQTT to ROS #####
topic_from: <MQTT topic>
topic to: <ROS topic>

```

</details>

```msg_type``` always describes the ROS message type.

After configuring the ROS MQTT bridge, we can go on with the VDA5050-Connector.

### VDA5050 connector topic configuration

<img src="https://user-images.githubusercontent.com/44091826/190145851-21905821-ce19-40af-8b82-f1853193f7fe.png" width="500">

The VDA5050 connector consists of five different daemons each represented by a single ROS node.
In order to allow easy access to the topics, you can change all subscribing (from master control and AGV) and publishing (to master control and AGV) topic names.

To do so, open the "vda5050_connector_topics.yaml" in the /config folder and change the topic names to fit your requirements.

>**NOTE**\
>Don't change the key. Only change the value.\
>Example:
>```yaml
>orderId: "/orderID"
>```
>```orderId``` is used for internal reference and must not be altered. To change the topic's name, change the value on the right ```"/orderId"```.

## Run the connector

### Launch the ROS MQTT bridge

Before starting the VDA5050 connector, the ROS MQTT bridge must be up and running.
To this end, open a terminal and launch the ROS MQTT bridge:

```bash
roslaunch vda5050_connector ros_mqtt_bridge.launch
```

The output should be similar to the following:

<details>

<summary>ROS MQTT bridge output</summary>

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

>**NOTE**\
>The ROS MQTT bridge tells if it is connected to the server by printing ```MQTT connected``` as last line. If anything went wrong, it prints ```MQTT disconnected```

### Launch the VDA5050 connector

As soon as the ROS MQTT bridge is connected to the server, the VDA5050 connector can be launched.
Open a new terminal and type:

```bash
roslaunch vda_5050_connector vda5050_connector.launch
```

If the VDA5050 connector was started properly, the output should be similar to:

<details>

<summary>VDA5050 connector output</summary>

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

</details>

The output gives an overview of all parameters read from the config file. Check if the topics are defined as required.
If any parameters are not readable or not found on the parameter server, there is a warning output. Please check if there is a typo in your config file.

## Interface Documentation
An overview of the currently available channels and required message types is available [here](doc/README.md).
## Known Issues

* Currently, the console output is done twice for some parameters due to the architecture.
* Sometimes the MQTT-Bridge cannot connect properly. If the output constantly changes between connected and disconnected, there is probably an error in the network configuration.

```console
   [INFO] [1657111653.836246]: MQTT disconnected
   [INFO] [1657111653.836246]: MQTT connected
   [INFO] [1657111653.836246]: MQTT disconnected
   [INFO] [1657111653.836246]: MQTT connected
```

## Comments on VDA 5050 specification
### Assumptions in unclear situations
**Situation:** Vehicle is processing an order and receives a new one (= differing order ID).

**Our Solution:** The new order is rejected if it doesn't begin at the end of the base of the previous (= currently running) order.

**Alternative:** The new order could be accepted if it begins at the end of the horizon of the previous order. Would lead to potential detours if the destination of the current order changes in the meantime.


**Situation:** The order message definition in the vda_msgs repository contains a boolean field called "replace". According to the commentary, this should be used if the base of an order is to be replaced by a new set of nodes/edges. However, the guideline does not define this procedure.

**Our Solution:** We do not implement the base replacement procedure. **TODO:** Throw a warning if the flag is set.

**Alternative:** A replacement procedure could be added to the routine of receiving a message on the order topic. See the according documentation and flow diagrams.


**Situation:** A new order is received. The guideline tells us to "validate" the order, but does not state how to achieve this.

**Our Solution:** We postpone the implementation of validations and expect the fleet controller to send conform order messages.

**Alternative:** Order messages should be validated to avoid undeterministic behavior, waiting and errors. The validation should at least check if the number of edges equals the number of nodes minus one.


## About

The ROS-VDA-5050-Connector was developed by idealworks in cooperation with [TUM fml – future.meets.logistics](https://www.linkedin.com/company/tum-fml/).
