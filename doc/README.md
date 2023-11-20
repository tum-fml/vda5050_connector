# VDA 5050 Connector Interface Documentation

This is a more in depth view of each available node.

## Nodes

### [VDA 5050 Connector](nodes/VDA5050Connector.md)

The VDA 5050 Connector handles the creation of VDA 5050 State, Visualization and Connection state messages, which are forwarded to the MQTT bridge, to be published to AnyFleet. This node builds the state message by receiving information about the robot's state on multiple topics.

The VDA 5050 Connector takes care of handling orders from AnyFleet, and forwarding the actions that need to be executed to the vehicle.

### [Action Client](nodes/Action.md)

The Action Client handles instantActions, and the state of the actions running on the AGV.

## Connections in the Connector

Currently, the application graph looks as follows :

![VDA 5050 Connector App Graph](images/App%20Graph.png)
