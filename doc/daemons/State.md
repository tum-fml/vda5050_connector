# State Daemon

### Current Status
The State daemon is a work in progress. This interface documentation will get updated as more channels get added with their proper type.

### Subscribed Topics

* /pose [nav_msgs::Odometry] : The current position of the AGV.
* /velocity [nav_msgs::Odometry] : The current speed of the AGV.  
* /mapId [std_msgs::String] : The ID of the map used by the AGV.  

### Published Topics

* /state [vda5050_msgs::State] : The state of the robot to be published to AnyFleet.

