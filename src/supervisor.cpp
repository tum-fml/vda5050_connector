/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 * 
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "ros/ros.h"
#include "vda5050_connector/state_daemon.h"
#include "vda5050_connector/connection_daemon.h"
#include "vda5050_connector/action_daemon.h"
#include "vda5050_connector/visualization_daemon.h"
#include <string>

using namespace std;
/*
 * TODO: write good comments
 */
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;

	// ConnectionDaemon connectionDaemon(&nh,"connection_daemon",15.0);
	// StateDaemon stateDaemon(&nh,"state_daemon");
	// VisDaemon visDaemon(&nh,"visualization_daemon");
	// ActionDaemon actionDaemon(&nh, "action_daemon");

	while(ros::ok())
	{
		// connectionDaemon.UpdateConnection();
		// stateDaemon.UpdateState();
		// visDaemon.UpdateVisualization();
		ros::spinOnce();
	}
	return 0;
}
