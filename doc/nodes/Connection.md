# Connection Publisher

The connection publisher reports the connection state of the AGV to Master Control. If the AGV wishes to gracefully disconnect, it can inform Master Control with an "OFFLINE" connection state.

## Config Overview

### Subscribed Topics

* connectionState [std_msgs::Bool] : Connection state from the MQTT Bridge.

### Published Topics

* connectionState [vda5050_msgs::Connection] : Connection state sent to Master Control.
