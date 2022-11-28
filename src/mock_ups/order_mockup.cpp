#include "ros/ros.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/Edge.h"

vda5050_msgs::Order create_example_order()
{
    vda5050_msgs::Order msg;
    // header
    msg.headerId = 0;
    msg.timestamp = "10/6/2022 10:16:37 AM";
    msg.version = "v1";
    msg.manufacturer;
    msg.serialNumber;

    msg.orderId = "f879f35e-a11f-4835-b13a-866c1bbc292a";

    msg.orderUpdateId = 0;

    msg.zoneSetId = " ";

    // nodes
    vda5050_msgs::Node node_1;
    node_1.nodeId = "9b5d075a-8757-4002-929d-e245c624881b";
    node_1.sequenceId = 0;
    node_1.nodeDescription = "Node based on device state: Header: 135";
    node_1.released = true;
    node_1.nodePosition.x = 10.0;
    node_1.nodePosition.y = 10.0;
    node_1.nodePosition.theta = 0.0;
    node_1.nodePosition.allowedDeviationXY = 100;
    node_1.nodePosition.allowedDeviationTheta = 100;
    node_1.nodePosition.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    node_1.nodePosition.mapDescription = "Id: 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    node_1.actions = std::vector<vda5050_msgs::Action>{};

    vda5050_msgs::Node node_2;
    node_2.nodeId = "6f61c844-e3d7-4018-85bf-c9d867fdd3d4";
    node_2.sequenceId = 2;
    node_2.nodeDescription = "NEntry node for PointOfInterest d13d3dc4-33f8-44f3-9631-72e0668aa55b WaitPoint";
    node_2.released = true;
    node_2.nodePosition.x = 16.413757;
    node_2.nodePosition.y = 19.216549;
    node_2.nodePosition.theta = 0.0;
    node_2.nodePosition.allowedDeviationXY = 0.7;
    node_2.nodePosition.allowedDeviationTheta = 0.0;
    node_2.nodePosition.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    node_2.nodePosition.mapDescription = "Map Id 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    node_2.actions = std::vector<vda5050_msgs::Action>{};

    msg.nodes = std::vector<vda5050_msgs::Node>{node_1, node_2};

    // edges
    vda5050_msgs::Edge edge_1;
    edge_1.edgeId = "8b4894b6-a03b-49cb-8cd4-a904bcf1f471";
    edge_1.sequenceId = 1;
    edge_1.edgeDescription = "Edge connecting 9b5d075a-8757-4002-929d-e245c624881b to 6f61c844-e3d7-4018-85bf-c9d867fdd3d4";
    edge_1.released = true;
    edge_1.startNodeId = "9b5d075a-8757-4002-929d-e245c624881b";
    edge_1.endNodeId = "6f61c844-e3d7-4018-85bf-c9d867fdd3d4";
    edge_1.maxSpeed = 0.0;
    edge_1.maxHeight = 0.0;
    edge_1.minHeight = 0.0;
    edge_1.orientation = 0.0;
    edge_1.direction = " ";
    edge_1.rotationAllowed = false;
    edge_1.maxRotationSpeed = 0.0;
    edge_1.trajectory.degree = 0.0;
    edge_1.trajectory.knotVector = std::vector<_Float64>{};
    edge_1.trajectory.controlPoints = std::vector<vda5050_msgs::ControlPoint>{};
    edge_1.length = 0.0;
    edge_1.actions = std::vector<vda5050_msgs::Action>{};

    msg.edges = std::vector<vda5050_msgs::Edge>{edge_1};

    return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "order_mockup");
    ros::NodeHandle nh;

    ros::Publisher order_pub = nh.advertise<vda5050_msgs::Order>("order_from_mc", 1000);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        vda5050_msgs::Order order_msg = create_example_order();
        order_pub.publish(order_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
