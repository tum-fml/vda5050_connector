# VDA 5050 Connector Interface Documentation
This is a more in depth view of each available daemon.

## Currently Implemented Daemons
### [State Daemon](daemons/State.md)
The state daemon handles the creation of VDA 5050 State messages, which are forwarded to the MQTT bridge, to be published to AnyFleet. This daemon builds the state message by receiving information about the robot's state on multiple topics. 
### [Visualization Daemon](daemons/Visualization.md)
The purpose of the Visualization daemon, is to provide information to AnyFleet about the robot's position and speed, at a higher rate than the State daemon. 

### [Connection Daemon](daemons/Connection.md)
The Connection daemon informs the cloud about the connection status of the device. If the device wishes to disconnect gracefully, it informs the cloud before closing the connection.

### [Order Daemon](daemons/Order.md)
The order daemon takes care of handling orders from AnyFleet, and forwarding the actions that need to be executed to the AGV.

### [Action Daemon](daemons/Action.md)
The action daemon handles instantActions, and the state of the actions running on the AGV.

# Connections in the Connector
Currently, the application graph looks as follows :

![VDA 5050 Connector App Graph](images/App%20Graph.png)

