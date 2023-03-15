# State Aggregator

## Current Status

The State Aggregator is a work in progress. It works by subscribing to multiple channels to get information about the state of the AGV, then it combines everything into one VDA5050 State message that is sent to Master Control.

The State Aggregator also builds and sends the visualization messages to MC.

This interface documentation will get updated as more channels get added with their proper type.

## Config Overview

### Subscribed Topics

* orderId [std_msgs::String] : ID of the order executed.
* orderUpdateId [std_msgs::UInt32] : The update ID of the executed order.
* zoneSetId [std_msgs::String] : The ID of the zone set being used.
* lastNodeId [std_msgs::String] : The ID of the last node traversed by the AGV.
* lastNodeSequenceId [std_msgs::UInt32] : The Sequence ID of the last node traversed.
* nodeStates [vda5050_msgs::NodeStates] : State of the nodes of the current order.
* edgeStates [vda5050_msgs::EdgeStates] : State of the edges of the current order.
* agvPosition [vda5050_msgs::AGVPosition] : The current position of the AGV.
* velocity [vda5050_msgs::Velocity] : The current speed of the AGV.
* loads [vda5050_msgs::Loads] : Load state of the AGV.
* driving [std_msgs::Bool] : State of the AGV if it's driving at the moment or not.
* paused [std_msgs::Bool] : State of the AGV if it's paused or not.
* newBaseRequest [std_msgs::Bool] : Boolean indicating that the AGV is running out of base nodes and requires an update.
* distanceSinceLastNode [std_msgs::Float64] : Distance traversed by the AGV since the last node traversed.
* actionStates [vda5050_msgs::ActionState] : State of the actions of the current order.
* batteryState [vda5050_msgs::BatteryState] : Battery state of the AGV.
* operatingMode [std_msgs::String] : The operating mode in the AGV. E.g. AUTOMATIC/MANUAL.
* errors [vda5050_msgs::Errors] : List of the errors produced by the AGV.
* information [vda5050_msgs::Information] : List of information messages produced by the AGV.
* safetyState [vda5050_msgs::SafetyState] : Current safety state of the AGV.

### Published Topics

* state [vda5050_msgs::State] : The state of the robot to be published to AnyFleet.
* visualization [vda5050_msgs::Visalization] : Real time visualization messages of the AGV to AnyFleet.
