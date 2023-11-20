# How to Use the ROS-VDA5050-Connector

## Aim of the Repository

The idea of the repository is to ensure an easy connection of the VDA 5050 with ROS.
It is intended to ensure an immediate integration into ROS.
For ease of use, you can customize all interfaces by adapting the configuration files in the /config folder to match your requirements.

The ROS-VDA5050-Connector only covers ROS communication, which means that all messages coming from the main control must be published to ROS topics. Since MQTT is widely used to communicate from the main control to AGVs, an example implementation to achieve compatibility between ROS and MQTT is shown below.

<img src="https://user-images.githubusercontent.com/44091826/190145636-37ef04e3-d28b-42da-961d-d2e8e919df73.png" width="500">

## Prerequisites

* This project was tested under Ubuntu 18.04 LTS + ROS Melodic (Python 2.7) and Ubuntu 20.04 LTS + ROS Noetic (Python 3.7).
* ROS must be installed and a workspace (e.g., `catkin_ws`) must be initialized.
* The python module Inject is required to run this project. Install by running the following command :

``` bash
pip install Inject==3.5.4
```

Since the VDA5050 connector solely relies on ROS communication, any other communication protocol must be translated accordingly.
VDA5050 specifies the use of MQTT as it is widely used for communication between the main control and AGVs. In the following we will describe a method for translating MQTT messages to ROS messages.
For this, we use the [ROS MQTT bridge](http://wiki.ros.org/mqtt_bridge) to connect to a server via TLS.
Of course, any other solution can be used as well. Names of outgoing and incoming topics can be adapted accordingly.

## Installation of the ROS MQTT bridge

Go to your catkin workspace and cd into the src directory:

```bash
cd ./src
```

Clone the `mqtt_bridge` repository.

```console
git clone https://github.com/tum-fml/mqtt_bridge.git
```

>**NOTE**\
Be careful to choose the correct branch. If you plan to use ROS melodic, checkout the branch "python2.7".\
>The master branch will only work with ROS noetic.

## Installation of the VDA5050 connector

In the same src folder where you cloned the MQTT Bridge, clone the `ROS-VDA-5050-Connector` repository:

```bash
git clone https://github.com/tum-fml/ros_vda5050_connector.git
```

Then clone the `vda5050_msgs` repository:

```bash
git clone https://github.com/tum-fml/vda5050_msgs.git
```

After cloning all required repositories, head to the parent folder of the src folder, and build your catkin workspace by running :

```console
catkin_make
```

## Customization of the configuration

There are distinct parts of the configuration that must be customized to make the connector function properly. Each configuration file is named after the node that it affects, with one common configuration file. The config files being :

1. ROS MQTT bridge.
2. VDA 5050 Connector.
3. Action Client.
4. AGV Data.

In the following sections we will go through them step by step.

>**NOTE**\
Since we wanted to use the ROS MQTT bridge out of the box with no further customization,\
all required parameters configuration files can be found in the /config folder in the `ROS-VDA5050-Connector` repository (and not in the `mqtt_bridge` repository).

### ROS MQTT Bridge Configuration

To make the ROS MQTT bridge work with TLS, complete the "mqtt_bridge.yaml" configuration file in the /config folder.

<details>

<summary>MQTT Bridge configuration</summary>

```text
mqtt:
  # TLS parameters.
  tls:
    ca_certs: <path_to_your_root_certificate>
    certfile: <path_to_your_key_certificate>
    keyfile: <your_path_to_private_key>
    tls_version: 5
    tls_insecure: false
    
  # Connection parameters.
  connection:
    host: <your_hostname>
    port: 8883
    keepalive: 60

  # MQTT parameters.
  client:
    protocol: 4
    client_id: <your_client_id>

bridge:
  # Bridge from ROS to MQTT.
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: vda5050_msgs.msg:State
    topic_from: /state
    topic_to: qa/<your_client_id>/state
    
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: vda5050_msgs.msg:Visualization
    topic_from: /visualization
    topic_to: qa/<your_client_id>/visualization

  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: vda5050_msgs.msg:Connection
    topic_from: /connection
    topic_to: qa/<your_client_id>/connection
    
  # Bridge from MQTT to ROS.
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: vda5050_msgs.msg:Action
    topic_from: qa/<your_client_id>/instantActions
    topic_to: /instantActions
    
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: vda5050_msgs.msg:Order
    topic_from: qa/<your_client_id>/order
    topic_to: /order_from_mc
```

</details>
Specifically, add the paths to the required certificates and the private key and enter your host address as well as your client id. Do not forget to use the correct port in your network.

Adapt topic names and message types according to your needs.

### VDA5050 connector topic configuration

<img src="https://user-images.githubusercontent.com/44091826/190145851-21905821-ce19-40af-8b82-f1853193f7fe.png" width="500">

The VDA5050 connector consists of 4 different nodes each represented by a single ROS node.
In order to allow easy access to the topics, you can change all subscribing (from master control and AGV) and publishing (to master control and AGV) topic names.

To do so, open the corresponding config file of the node in the /config folder and change the topic names to fit your requirements.

>**NOTE**\
>Don't change the key. Only change the value.\
>Example:
>
>```yaml
>orderId: "/orderID"
>```
>
>```orderId``` is used for internal reference and must not be altered. To change the topic's name, change the value on the right ```"/orderId"```.

Check the [Interface Section](#interface-documentation) section for more information about each node.

## Run the connector

The VDA5050 Connector launches the MQTT Bridge and the connector nodes together. To start the connector, run the following command :

```bash
roslaunch vda5050_connector vda5050_connector.launch 
```

If the VDA5050 connector was started properly, the output should be similar to:

<details>

<summary>VDA5050 connector output</summary>

```console
NODES
  /
    action_client (vda5050_connector/action_client)
    mqtt_bridge (mqtt_bridge/mqtt_bridge_node.py)
    vda5050_connector (vda5050_connector/vda5050_connector)

ROS_MASTER_URI=http://localhost:11311

process[mqtt_bridge-1]: started with pid [22954]
process[action_client-2]: started with pid [22955]
process[vda5050_connector-4]: started with pid [27631]
[INFO] [1678911895.153781] [/mqtt_bridge]: MQTT connected

```

</details>

The output also gives an overview of all parameters read from the config file. Check if the topics are defined as required.
If any parameters are not readable or not found on the parameter server, there is a warning output. Please check if there is a typo in your config file.

## Run the state mockup

The Connector includes a state mockup for testing. The mockup sends random speed, twist and battery values to the State Aggregator.
You can run the mockup with the following command :

``` bash
rosrun vda5050_connector state_mockup
```

## Interface Documentation

An overview of the node configuration, channels and required message types is available [here](doc/README.md).

## About

The ROS-VDA5050-Connector was developed by [idealworks](https://idealworks.com/) in cooperation with [TUM fml – future.meets.logistics](https://www.linkedin.com/company/tum-fml/).
