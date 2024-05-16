#include "ros/ros.h"
#include "agiros_msgs/Reference.h"
#include "agiros_msgs/Setpoint.h"
#include "agiros_msgs/QuadState.h"
#include "agiros_msgs/Command.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_creator");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub = nh.advertise<agiros_msgs::Reference>("/kingfisher/agiros_pilot/trajectory", 10);

    ros::Rate loop_rate(1); // Publish rate (1 Hz)

    while (ros::ok()) {
        // Create a Reference message
        agiros_msgs::Reference reference_msg;

        // Fill in the header
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "world";
        reference_msg.header = header;

        // Define waypoints
        std::vector<agiros_msgs::Setpoint> waypoints;

        // Waypoint 1
        agiros_msgs::Setpoint wp1;
        wp1.state.header = header;
        wp1.state.t = 0.0;
        wp1.state.pose.position.x = 0.0;
        wp1.state.pose.position.y = 0.0;
        wp1.state.pose.position.z = 1.0;
        wp1.state.pose.orientation.w = 1.0; // Neutral orientation
        waypoints.push_back(wp1);

        // Waypoint 2
        agiros_msgs::Setpoint wp2;
        wp2.state.header = header;
        wp2.state.t = 5.0; // Time to reach this waypoint
        wp2.state.pose.position.x = 1.0;
        wp2.state.pose.position.y = 1.0;
        wp2.state.pose.position.z = 1.0;
        wp2.state.pose.orientation.w = 1.0;
        waypoints.push_back(wp2);

        // Waypoint 3
        agiros_msgs::Setpoint wp3;
        wp3.state.header = header;
        wp3.state.t = 10.0; // Time to reach this waypoint
        wp3.state.pose.position.x = 2.0;
        wp3.state.pose.position.y = 0.0;
        wp3.state.pose.position.z = 1.0;
        wp3.state.pose.orientation.w = 1.0;
        waypoints.push_back(wp3);

        // Add waypoints to the Reference message
        reference_msg.points = waypoints;

        // Publish the trajectory
        trajectory_pub.publish(reference_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
