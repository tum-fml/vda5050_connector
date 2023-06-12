# VDA 5050 Connector Interface Documentation

This is a more in depth view of each available node.

## Currently Implemented Nodes

### [State Aggregator](nodes/State.md)

The State Aggregator handles the creation of VDA 5050 State messages, which are forwarded to the MQTT bridge, to be published to AnyFleet. This node builds the state message by receiving information about the robot's state on multiple topics. This node is also responsible for generating visualization messages to be sent to Master Control. The State Aggregator also informs the cloud about the connection status of the device. If the device wishes to disconnect gracefully, it informs the cloud before closing the connection.

### [Order Manager](nodes/Order.md)

The Order Manager takes care of handling orders from AnyFleet, and forwarding the actions that need to be executed to the AGV.

### [Action Client](nodes/Action.md)

The Action Client handles instantActions, and the state of the actions running on the AGV.

## Connections in the Connector

Currently, the application graph looks as follows :

![VDA 5050 Connector App Graph](images/App%20Graph.png)
