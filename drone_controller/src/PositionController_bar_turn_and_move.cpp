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

class TrajectoryCreator { // Class to create a trajectory
public:
    TrajectoryCreator() : nh("~"), loopRate(1), initialPose1Set(false), initialPose2Set(false) {
        // Constructor

        // Read parameters
        ReadParameters();

        // Initialize the Publisher
        trajectoryPub1 = nh.advertise<agiros_msgs::Reference>("/" + droneID1 + "/agiros_pilot/trajectory", 10);
        trajectoryPub2 = nh.advertise<agiros_msgs::Reference>("/" + droneID2 + "/agiros_pilot/trajectory", 10);

        // Initialize the Subscriber
        odometrySub1 = nh.subscribe("/" + droneID1 + "/agiros_pilot/odometry", 10, &TrajectoryCreator::OdometryCallback1, this);
        odometrySub2 = nh.subscribe("/" + droneID2 + "/agiros_pilot/odometry", 10, &TrajectoryCreator::OdometryCallback2, this);

        // Create a pose message
        geometry_msgs::Pose pose;
        pose.position.x = targetX;
        pose.position.y = targetY;
        pose.position.z = targetZ;
        pose.orientation.w = 1.0; // Neutral orientation

        // Send the pose to the drone
        // SendToPose(pose);

        // Send a triangle trajectory
        SendTriangleTrajectory();
    }

private:
    double targetX, targetY, targetZ;
    double targetTime;
    bool initialPose1Set, initialPose2Set;
    std::string droneID1, droneID2;
    ros::NodeHandle nh;
    ros::Publisher trajectoryPub1, trajectoryPub2;
    ros::Subscriber odometrySub1, odometrySub2;
    ros::Rate loopRate; // Publish rate (1 Hz)
    geometry_msgs::Pose currentPose1, currentPose2;
    geometry_msgs::Pose initialPose1, initialPose2;

    void ReadParameters() {
        nh.getParam("targetX", targetX);
        nh.getParam("targetY", targetY);
        nh.getParam("targetZ", targetZ);
        nh.getParam("targetTime", targetTime);
        nh.getParam("droneID1", droneID1);
        nh.getParam("droneID2", droneID2);

        ROS_INFO("C++ file launched with target position: (%f, %f, %f)", targetX, targetY, targetZ);
    }

    void OdometryCallback1(const agiros_msgs::QuadState::ConstPtr& msg) {
        if (!initialPose1Set) {
            initialPose1 = msg->pose;
            initialPose1Set = true;
        } else {
            currentPose1 = msg->pose;
        }
    }

    void OdometryCallback2(const agiros_msgs::QuadState::ConstPtr& msg) {
        if (!initialPose2Set) {
            initialPose2 = msg->pose;
            initialPose2Set = true;
        } else {
            currentPose2 = msg->pose;
        }
    }


    void SendToPose(geometry_msgs::Pose pose) {
        bool sentCommand = false; // Flag to track if the command has been sent
        while (ros::ok() && !sentCommand) {
            // Create a Reference message
            agiros_msgs::Reference referenceMsg;

            // Fill in the header
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "world";
            referenceMsg.header = header;

            // Define waypoints
            std::vector<agiros_msgs::Setpoint> waypoints;

            // First waypoint
            agiros_msgs::Setpoint wp1;
            wp1.state.header = header;
            wp1.state.t = 0.0;
            wp1.state.pose = initialPose1;
            waypoints.push_back(wp1);

            // Second waypoint
            agiros_msgs::Setpoint wp2;
            wp2.state.header = header;
            wp2.state.t = targetTime;
            wp2.state.pose = pose;
            waypoints.push_back(wp2);

            // Add waypoints to the Reference message
            referenceMsg.points = waypoints;

            // Publish the trajectory
            trajectoryPub1.publish(referenceMsg);
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    void GenerateTrajectory(geometry_msgs::Pose pose) {
        // Create a Reference message
        agiros_msgs::Reference referenceMsg;

        // Fill in the header
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "world";
        referenceMsg.header = header;

        // Define waypoints
        std::vector<agiros_msgs::Setpoint> waypoints;

        // First waypoint
        agiros_msgs::Setpoint wp1;
        wp1.state.header = header;
        wp1.state.t = 0.0;
        wp1.state.pose = initialPose1;
        waypoints.push_back(wp1);

        // Second waypoint
        agiros_msgs::Setpoint wp2;
        wp2.state.header = header;
        wp2.state.t = targetTime;
        wp2.state.pose = pose;
        waypoints.push_back(wp2);

        // Add waypoints to the Reference message
        referenceMsg.points = waypoints;

        // Publish the trajectory
        trajectoryPub1.publish(referenceMsg);
        ros::spinOnce();
        loopRate.sleep();

        
    }

    void SendTriangleTrajectory() {
        while (ros::ok()) {
            // Create a Reference message
            agiros_msgs::Reference referenceMsg;

            // Fill in the header
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "world";
            referenceMsg.header = header;

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
            wp3.state.t = targetTime; // Time to reach this waypoint
            wp3.state.pose.position.x = 2.0;
            wp3.state.pose.position.y = 0.0;
            wp3.state.pose.position.z = 1.0;
            wp3.state.pose.orientation.w = 1.0;
            waypoints.push_back(wp3);

            // Add waypoints to the Reference message
            referenceMsg.points = waypoints;

            // Publish the trajectory
            trajectoryPub1.publish(referenceMsg);

            ros::spinOnce();
            loopRate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectoryCreator");

    TrajectoryCreator trajectoryCreator;
    ros::spin();

    return 0;
}
