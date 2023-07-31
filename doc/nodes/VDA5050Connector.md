# VDA5050 Connector

## Current Status

The State Aggregator is a work in progress. It works by subscribing to multiple channels to get information about the state of the AGV, then it combines everything into one VDA5050 State message that is sent to Master Control.

The State Aggregator also builds and sends the visualization messages to MC, as well as the connection state messages.

This interface documentation will get updated as more channels get added with their proper type.

## Config Overview

### Subscribed Topics

* order_from_mc [vda5050_msgs::Order] : Order message from the master control.
* order_state [vda5050_msgs::State] : State message containing order related information. (OrderId, OrderUpdateId, LastNodeId, LastNodeSequenceId, NodeStates, EdgeStates, ActionStates)
* zoneSetId [std_msgs::String] : The ID of the zone set being used.
* agvPosition [geometry_msgs::Pose] : The current position of the AGV.
* velocity [geometry_msgs::Twist] : The current speed of the AGV.
* loads [vda5050_msgs::Loads] : Load state of the AGV.
* paused [std_msgs::Bool] : State of the AGV if it's paused or not.
* newBaseRequest [std_msgs::Bool] : Boolean indicating that the AGV is running out of base nodes and requires an update.
* distanceSinceLastNode [std_msgs::Float64] : Distance traversed by the AGV since the last node traversed.
* batteryState [sensor_msgs::BatteryState] : Battery state of the AGV.
* operatingMode [std_msgs::String] : The operating mode in the AGV. E.g. AUTOMATIC/MANUAL.
* errors [vda5050_msgs::Errors] : List of the errors produced by the AGV.
* information [vda5050_msgs::Information] : List of information messages produced by the AGV.
* safetyState [vda5050_msgs::SafetyState] : Current safety state of the AGV.

### Published Topics

* state [vda5050_msgs::State] : The state of the robot to be published to AnyFleet.
* visualization [vda5050_msgs::Visalization] : Real time visualization messages of the AGV to AnyFleet.
* connection [vda5050_msgs::Connection] : Connection state sent to Master Control.
