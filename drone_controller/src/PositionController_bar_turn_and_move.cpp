#include "ros/ros.h"
#include "agiros_msgs/Reference.h"
#include "agiros_msgs/Setpoint.h"
#include "agiros_msgs/QuadState.h"
#include "agiros_msgs/Command.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include "std_msgs/Empty.h"
#include <tf/tf.h>
#include <cmath>

class TrajectoryCreator { // Class to create a trajectory
public:
    TrajectoryCreator() : nh("~"), loopRate(1), initialPose1Set(false), initialPose2Set(false), initialPoseBarSet(false), deltaDistance(0.1), landingSequence(false){
        // Constructor

        // Read parameters
        ReadParameters();

        // Initialize the Publisher
        trajectoryPub1 = nh.advertise<agiros_msgs::Reference>("/" + droneID1 + "/agiros_pilot/trajectory", 10);
        trajectoryPub2 = nh.advertise<agiros_msgs::Reference>("/" + droneID2 + "/agiros_pilot/trajectory", 10);

        goToPosePub1 = nh.advertise<geometry_msgs::PoseStamped>("/" + droneID1 + "/agiros_pilot/go_to_pose", 10);
        goToPosePub2 = nh.advertise<geometry_msgs::PoseStamped>("/" + droneID2 + "/agiros_pilot/go_to_pose", 10);

        forceHoverPub1 = nh.advertise<std_msgs::Empty>("/" + droneID1 + "/agiros_pilot/force_hover", 1, true);
        forceHoverPub2 = nh.advertise<std_msgs::Empty>("/" + droneID2 + "/agiros_pilot/force_hover", 1, true);
        

        std::string subscribeTopic1 = "/" + droneID1 + "/agiros_pilot/odometry";
        std::string subscribeTopic2 = "/" + droneID2 + "/agiros_pilot/odometry";
        std::string subscribeTopicBar = "/" + barID + "/odometry_sensor1/odometry";

        // Initialize the Subscriber
        odometrySub1 = nh.subscribe(subscribeTopic1, 10, &TrajectoryCreator::OdometryCallback1, this);
        odometrySub2 = nh.subscribe(subscribeTopic2, 10, &TrajectoryCreator::OdometryCallback2, this);
        odometrySubBar = nh.subscribe(subscribeTopicBar, 10, &TrajectoryCreator::OdometryCallbackBar, this);
        
        ROS_INFO("Subscribed to odometry topics: %s, %s", subscribeTopic1.c_str(), subscribeTopic2.c_str());

        // Wait for initial pose to be set
        while (ros::ok() && !initialPose1Set && !initialPose2Set && !initialPoseBarSet) {
            ros::spinOnce();
            loopRate.sleep();
            ROS_INFO("Waiting for initial pose to be set, falcon1: %d, falcon2: %d", initialPose1Set, initialPose2Set);
        }

        if (!landingSequence) {
            double currentYaw = GetYawFromQuaternion(currentPose1.orientation);
            ROS_INFO("Current yaw: %f", currentYaw);
            geometry_msgs::Pose pose;
            pose.position.x = targetX;
            pose.position.y = targetY;
            pose.position.z = targetZ;
            pose.orientation = GetQuaternionFromYaw(targetYaw);
            pose = GeneratePoseFromYawAndPosition(pose, targetYaw);
            SendToPose(pose);
        } else {
            InitiateLandingSequence();
        }



        // CalculateBarEndpointPositions(currentPoseBar, GetYawFromQuaternion(currentPoseBar.orientation));


        // Send the pose to the drone
        // SendToPose(pose);

        // Send a triangle trajectory
        // SendTriangleTrajectory();
    }

private:
    double targetX, targetY, targetZ;
    double targetTime;
    double targetYaw;
    double radiusBar, cableLength, deltaZ, zBias;
    double deltaDistance;
    bool initialPose1Set, initialPose2Set, initialPoseBarSet;
    bool landingSequence;
    std::string droneID1, droneID2, barID;
    ros::NodeHandle nh;
    ros::Publisher trajectoryPub1, trajectoryPub2, forceHoverPub1, forceHoverPub2, goToPosePub1, goToPosePub2;
    ros::Subscriber odometrySub1, odometrySub2, odometrySubBar;
    ros::Rate loopRate; // Publish rate (1 Hz)
    geometry_msgs::Pose currentPose1, currentPose2, currentPoseBar;
    geometry_msgs::Pose initialPose1, initialPose2, initialPoseBar;

    void ReadParameters() {
        nh.getParam("targetX", targetX);
        nh.getParam("targetY", targetY);
        nh.getParam("targetZ", targetZ);
        nh.getParam("targetTime", targetTime);
        nh.getParam("droneID1", droneID1);
        nh.getParam("droneID2", droneID2);
        nh.getParam("targetYaw", targetYaw);
        nh.getParam("barID", barID);
        nh.getParam("radiusBar", radiusBar);
        nh.getParam("cableLength", cableLength);
        nh.getParam("deltaZ", deltaZ);
        nh.getParam("zBias", zBias);
        nh.getParam("landingSequence", landingSequence);
        targetYaw = DegreesToRadians(targetYaw);


        ROS_INFO("C++ file launched with target position: (%f, %f, %f)", targetX, targetY, targetZ);
    }

    void OdometryCallback1(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initialPose1Set) {
            initialPose1 = msg->pose.pose;
            initialPose1.position.z += deltaZ;
            initialPose1Set = true;
        } else {
            currentPose1 = msg->pose.pose;
        }
    }

    void OdometryCallback2(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initialPose2Set) {
            initialPose2 = msg->pose.pose;
            initialPose2.position.z += deltaZ;
            initialPose2Set = true;
        } else {
            currentPose2 = msg->pose.pose;
        }
    }

    void OdometryCallbackBar(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initialPoseBarSet) {
            initialPoseBar = msg->pose.pose;
            initialPoseBarSet = true;
            ROS_INFO("Initial pose of the bar: (%f, %f, %f)", initialPoseBar.position.x, initialPoseBar.position.y, initialPoseBar.position.z);
        } else {
            currentPoseBar = msg->pose.pose;
        }
    }


double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

void InitiateLandingSequence() {
    // Create PoseStamped messages for current pose of both drones
    geometry_msgs::PoseStamped currentPoseStamped1, currentPoseStamped2;
    currentPoseStamped1.header.stamp = ros::Time::now();
    currentPoseStamped1.header.frame_id = "world";
    currentPoseStamped1.pose = currentPose1;
    
    currentPoseStamped2.header.stamp = ros::Time::now();
    currentPoseStamped2.header.frame_id = "world";
    currentPoseStamped2.pose = currentPose2;

    currentPoseStamped1.pose.position.z = 0.2 * cableLength + deltaZ;
    currentPoseStamped2.pose.position.z = 0.2 * cableLength + deltaZ;

    currentPoseStamped1.pose.position.x += 0.5 * cableLength;
    currentPoseStamped2.pose.position.x -= 0.5 * cableLength;

    currentPoseStamped1.pose.position.y += 0.5 * cableLength;
    currentPoseStamped2.pose.position.y -= 0.5 * cableLength;
    
    std_msgs::Empty emptyMsg;
    bool commandSent = false;

    while (ros::ok() && !commandSent) {
        // Publish the current pose of both drones
        goToPosePub1.publish(currentPoseStamped1);
        goToPosePub2.publish(currentPoseStamped2);

        // ROS info
        ROS_INFO("Publishing current pose of drones");

        ros::spinOnce();
        loopRate.sleep();
        // Sleep for 1 second
        ros::Duration(5).sleep();

        // Publish force hover messages
        forceHoverPub1.publish(emptyMsg);
        forceHoverPub2.publish(emptyMsg);

        // ROS info
        ROS_INFO("Publishing force hover messages");
        commandSent = true;
    }
}

void SendToPose(geometry_msgs::Pose pose) {
    bool sentCommand = false; // Flag to track if the command has been sent

    // Adjust for CableLength and zBias
    pose.position.z += cableLength + zBias;

    geometry_msgs::Pose deltaPose1, deltaPose2;
    deltaPose1 = currentPose1;
    deltaPose2 = currentPose2;
    double initialZ = (initialPose1.position.z + initialPose2.position.z) / 2;

    double initialYaw1 = GetYawFromQuaternion(initialPose1.orientation);
    double initialYaw2 = GetYawFromQuaternion(initialPose2.orientation);
    double targetYaw = GetYawFromQuaternion(pose.orientation);

    // Calculate the target positions for the left and right drones based on the midpoint of the bar
    double targetXLeft = radiusBar * cos(targetYaw + M_PI / 2);
    double targetYLeft = radiusBar * sin(targetYaw + M_PI / 2);
    double targetXRight = radiusBar * cos(targetYaw - M_PI / 2);
    double targetYRight = radiusBar * sin(targetYaw - M_PI / 2);

    geometry_msgs::Pose poseLeft, poseRight;
    poseLeft.orientation = pose.orientation;
    poseRight.orientation = pose.orientation;

    // Adjust positions relative to the midpoint (target pose)
    poseLeft.position.x = pose.position.x + targetXLeft;
    poseLeft.position.y = pose.position.y + targetYLeft;
    poseLeft.position.z = pose.position.z;
    
    poseRight.position.x = pose.position.x + targetXRight;
    poseRight.position.y = pose.position.y + targetYRight;
    poseRight.position.z = pose.position.z;

    // Determine which drone should go to which position
    geometry_msgs::Point pointLeft, pointRight;
    pointLeft.x = currentPose1.position.x + radiusBar * cos(targetYaw + M_PI / 2);
    pointLeft.y = currentPose1.position.y + radiusBar * sin(targetYaw + M_PI / 2);
    pointLeft.z = currentPose1.position.z;

    pointRight.x = currentPose1.position.x + radiusBar * cos(targetYaw - M_PI / 2);
    pointRight.y = currentPose1.position.y + radiusBar * sin(targetYaw - M_PI / 2);
    pointRight.z = currentPose1.position.z;

    double dist1ToLeft = calculateDistance(currentPose1.position, pointLeft);
    double dist1ToRight = calculateDistance(currentPose1.position, pointRight);
    double dist2ToLeft = calculateDistance(currentPose2.position, pointLeft);
    double dist2ToRight = calculateDistance(currentPose2.position, pointRight);

    bool isDrone1Left = (dist1ToLeft + dist2ToRight < dist1ToRight + dist2ToLeft);

    while (ros::ok() && !sentCommand) {
        // Create a Reference message
        agiros_msgs::Reference referenceMsg1, referenceMsg2;
        
        // Fill in the header
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "world";
        referenceMsg1.header = header;
        referenceMsg2.header = header;

        // Define waypoints
        std::vector<agiros_msgs::Setpoint> waypoints1, waypoints2;

        // First waypoint left drone
        agiros_msgs::Setpoint wp1;
        wp1.state.header = header;
        wp1.state.t = 0.0;
        wp1.state.pose = initialPose1;
        wp1.state.pose.position.z = initialZ;
        waypoints1.push_back(wp1);

        // First waypoint right drone
        agiros_msgs::Setpoint wp3;
        wp3.state.header = header;
        wp3.state.t = 0.0;
        wp3.state.pose = initialPose2;
        wp3.state.pose.position.z = initialZ;
        waypoints2.push_back(wp3);

        // Assign paths based on relative positions
        geometry_msgs::Pose targetPose1 = isDrone1Left ? poseLeft : poseRight;
        geometry_msgs::Pose targetPose2 = isDrone1Left ? poseRight : poseLeft;

        // Calculate intermediate positions for left drone
        geometry_msgs::Pose intermediatePose1Left = initialPose1;
        intermediatePose1Left.position.x += (targetPose1.position.x - initialPose1.position.x) / 3;
        intermediatePose1Left.position.y += (targetPose1.position.y - initialPose1.position.y) / 3;
        intermediatePose1Left.position.z += (targetPose1.position.z - initialPose1.position.z) / 3;
        intermediatePose1Left.orientation = GetQuaternionFromYaw(interpolateYaw(initialYaw1, targetYaw, 1.0 / 3.0));

        geometry_msgs::Pose intermediatePose2Left = initialPose1;
        intermediatePose2Left.position.x += 2 * (targetPose1.position.x - initialPose1.position.x) / 3;
        intermediatePose2Left.position.y += 2 * (targetPose1.position.y - initialPose1.position.y) / 3;
        intermediatePose2Left.position.z += 2 * (targetPose1.position.z - initialPose1.position.z) / 3;
        intermediatePose2Left.orientation = GetQuaternionFromYaw(interpolateYaw(initialYaw1, targetYaw, 2.0 / 3.0));

        // Calculate intermediate positions for right drone
        geometry_msgs::Pose intermediatePose1Right = initialPose2;
        intermediatePose1Right.position.x += (targetPose2.position.x - initialPose2.position.x) / 3;
        intermediatePose1Right.position.y += (targetPose2.position.y - initialPose2.position.y) / 3;
        intermediatePose1Right.position.z += (targetPose2.position.z - initialPose2.position.z) / 3;
        intermediatePose1Right.orientation = GetQuaternionFromYaw(interpolateYaw(initialYaw2, targetYaw, 1.0 / 3.0));

        geometry_msgs::Pose intermediatePose2Right = initialPose2;
        intermediatePose2Right.position.x += 2 * (targetPose2.position.x - initialPose2.position.x) / 3;
        intermediatePose2Right.position.y += 2 * (targetPose2.position.y - initialPose2.position.y) / 3;
        intermediatePose2Right.position.z += 2 * (targetPose2.position.z - initialPose2.position.z) / 3;
        intermediatePose2Right.orientation = GetQuaternionFromYaw(interpolateYaw(initialYaw2, targetYaw, 2.0 / 3.0));

        // Intermediate waypoint 1 left drone
        agiros_msgs::Setpoint wpInt1Left;
        wpInt1Left.state.header = header;
        wpInt1Left.state.t = targetTime / 3;
        wpInt1Left.state.pose = intermediatePose1Left;
        waypoints1.push_back(wpInt1Left);

        // Intermediate waypoint 1 right drone
        agiros_msgs::Setpoint wpInt1Right;
        wpInt1Right.state.header = header;
        wpInt1Right.state.t = targetTime / 3;
        wpInt1Right.state.pose = intermediatePose1Right;
        waypoints2.push_back(wpInt1Right);

        // Intermediate waypoint 2 left drone
        agiros_msgs::Setpoint wpInt2Left;
        wpInt2Left.state.header = header;
        wpInt2Left.state.t = 2 * targetTime / 3;
        wpInt2Left.state.pose = intermediatePose2Left;
        waypoints1.push_back(wpInt2Left);

        // Intermediate waypoint 2 right drone
        agiros_msgs::Setpoint wpInt2Right;
        wpInt2Right.state.header = header;
        wpInt2Right.state.t = 2 * targetTime / 3;
        wpInt2Right.state.pose = intermediatePose2Right;
        waypoints2.push_back(wpInt2Right);

        // Second waypoint left drone
        agiros_msgs::Setpoint wp2;
        wp2.state.header = header;
        wp2.state.t = targetTime;
        wp2.state.pose = targetPose1;
        waypoints1.push_back(wp2);

        // Second waypoint right drone
        agiros_msgs::Setpoint wp4;
        wp4.state.header = header;
        wp4.state.t = targetTime;
        wp4.state.pose = targetPose2;
        waypoints2.push_back(wp4);

        // Add waypoints to the Reference messages
        referenceMsg1.points = waypoints1;
        referenceMsg2.points = waypoints2;

        ROS_INFO("Sending left drone to position: (%f, %f, %f)", targetPose1.position.x, targetPose1.position.y, targetPose1.position.z);
        ROS_INFO("Sending right drone to position: (%f, %f, %f)", targetPose2.position.x, targetPose2.position.y, targetPose2.position.z);

        // Check if both drones are within deltaDistance of their initial positions
        double distanceLeft = calculateDistance(deltaPose1.position, targetPose1.position);
        double distanceRight = calculateDistance(deltaPose2.position, targetPose2.position);
        ROS_INFO("Delta distance: %f", deltaDistance);

        ROS_INFO("Distance left: %f, distance right: %f", distanceLeft, distanceRight);
        if (distanceLeft >= deltaDistance && distanceRight >= deltaDistance) {
            sentCommand = true;
            ROS_INFO("Sent command is set to %d", sentCommand);
        }

        // Publish the trajectories
        trajectoryPub1.publish(referenceMsg1);
        trajectoryPub2.publish(referenceMsg2);
        ros::spinOnce();
        loopRate.sleep();
    }
}



double interpolateYaw(double yaw1, double yaw2, double t) {
    return yaw1 + t * (yaw2 - yaw1);
}





    geometry_msgs::Pose GeneratePoseFromYawAndPosition(geometry_msgs::Pose position, double yaw) {
        geometry_msgs::Pose pose;
        pose.position = position.position;
        pose.orientation = GetQuaternionFromYaw(yaw);
        return pose;
    }

    void GeneratePointFromMidpointBar(geometry_msgs::Pose poseMidpoint, double yawMidpoint) {

    }

    void GenerateTrajectory(geometry_msgs::Pose pose) {
        while (ros::ok()) {
            // Create a Reference message
            agiros_msgs::Reference referenceMsg1, referenceMsg2;


            // Fill in the header
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "world";
            referenceMsg1.header = header;
            referenceMsg2.header = header;

            // Define waypoints
            std::vector<agiros_msgs::Setpoint> waypoints1, waypoints2;

            // Get the bar endpoints

            // First waypoint left drone
            agiros_msgs::Setpoint wp1;
            wp1.state.header = header;
            wp1.state.t = 0.0;
            wp1.state.pose = initialPose1;
            waypoints1.push_back(wp1);

            // Second waypoint
            agiros_msgs::Setpoint wp2;
            wp2.state.header = header;
            wp2.state.t = targetTime;
            wp2.state.pose = pose;
            waypoints1.push_back(wp2);

            // Add waypoints to the Reference message
            referenceMsg1.points = waypoints1;

            // Publish the trajectory
            trajectoryPub1.publish(referenceMsg1);
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    std::vector<geometry_msgs::Pose> CalculateBarEndpointPositions(geometry_msgs::Pose midpointBar, double yawMidpointBar) {  
        std::vector<geometry_msgs::Pose> barEndpoints;

        // Calculate the target position for the drone to the left of the bar
        double targetYawLeft = yawMidpointBar + M_PI / 2; // 90 degrees to the left
        double targetXLeft = radiusBar * cos(targetYawLeft);
        double targetYLeft = radiusBar * sin(targetYawLeft);

        double targetYawRight = yawMidpointBar - M_PI / 2; // 90 degrees to the right
        double targetXRight = radiusBar * cos(targetYawRight);
        double targetYRight = radiusBar * sin(targetYawRight);

        ROS_INFO("Target position to the left of the bar: (%f, %f)", targetXLeft, targetYLeft);
        ROS_INFO("Target position to the right of the bar: (%f, %f)", targetXRight, targetYRight);

        geometry_msgs::Pose poseLeft, poseRight;

        poseLeft.orientation = GetQuaternionFromYaw(yawMidpointBar);
        poseLeft.position.x = targetXLeft + midpointBar.position.x;
        poseLeft.position.y = targetYLeft + midpointBar.position.y;
        poseLeft.position.z = midpointBar.position.z + cableLength;

        poseRight.orientation = GetQuaternionFromYaw(yawMidpointBar);
        poseRight.position.x = targetXRight + midpointBar.position.x;
        poseRight.position.y = targetYRight + midpointBar.position.y;
        poseRight.position.z = midpointBar.position.z + cableLength;

        barEndpoints.push_back(poseLeft);
        barEndpoints.push_back(poseRight);

        return barEndpoints;


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

    void CalculateMidpoint() {
        geometry_msgs::Pose midpoint;
        midpoint.position.x = (initialPose1.position.x + initialPose2.position.x) / 2;
        midpoint.position.y = (initialPose1.position.y + initialPose2.position.y) / 2;
        midpoint.position.z = (initialPose1.position.z + initialPose2.position.z) / 2;
        midpoint.orientation.w = 1.0;

        GenerateTrajectory(midpoint);
    
    }

    // Function to extract yaw from a geometry_msgs::Quaternion
    double GetYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    // Function to create a geometry_msgs::Quaternion from a yaw angle
    geometry_msgs::Quaternion GetQuaternionFromYaw(double yaw) {
        tf::Quaternion q;
        q.setRPY(0, 0, yaw); // Roll and pitch are set to zero
        geometry_msgs::Quaternion quat_msg;
        tf::quaternionTFToMsg(q, quat_msg);
        return quat_msg;
    }

    double DegreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectoryCreator");

    TrajectoryCreator trajectoryCreator;
    ros::spin();

    return 0;
}
