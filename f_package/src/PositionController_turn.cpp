#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <cmath>

class TurnHandler {
private:
    std::string drone_id1, drone_id2;
    double end_angle_degrees;
    ros::NodeHandle nh;
    ros::Publisher pose_pub_falcon1;
    ros::Publisher pose_pub_falcon2;
    ros::Subscriber odom_sub_falcon1;
    ros::Subscriber odom_sub_falcon2;
    geometry_msgs::Pose p1;  // Store the current pose of drone1
    geometry_msgs::Pose p2;  // Store the current pose of drone2
    geometry_msgs::PoseStamped midpoint;
    double radius;
    double angle;

public:
    TurnHandler(const std::string &_drone_id1, const std::string &_drone_id2, double _end_angle_degrees)
        : drone_id1(_drone_id1), drone_id2(_drone_id2), end_angle_degrees(_end_angle_degrees), nh("~"), radius(5.0), angle(0.0) {

        

        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id1 + "/agiros_pilot/go_to_pose", 10);
        pose_pub_falcon2 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id2 + "/agiros_pilot/go_to_pose", 10);

        odom_sub_falcon1 = nh.subscribe("/" + drone_id1 + "/agiros_pilot/odometry", 10, &TurnHandler::OdomCallbackFalcon1, this);
        odom_sub_falcon2 = nh.subscribe("/" + drone_id2 + "/agiros_pilot/odometry", 10, &TurnHandler::OdomCallbackFalcon2, this);

        GetParams();

        midpoint = CalculateMidpoint(p1, p2);
    }

    void GetParams() {
        nh.param("drone_id1", drone_id1, std::string("falcon"));
        nh.param("drone_id2", drone_id2, std::string("falcon1"));
        nh.param("end_angle_degrees", end_angle_degrees, 0.0);
        nh.param("radius", radius, 5.0);  // Optional: Allow setting radius via parameter
    }

    geometry_msgs::PoseStamped CalculateMidpoint(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
        geometry_msgs::PoseStamped midpoint;
    
        // Calculate midpoint of the positions
        midpoint.pose.position.x = (p1.position.x + p2.position.x) / 2.0;
        midpoint.pose.position.y = (p1.position.y + p2.position.y) / 2.0;
        midpoint.pose.position.z = (p1.position.z + p2.position.z) / 2.0;
    
        // Set orientation (assuming you might want to average the orientations as well)
        midpoint.pose.orientation.x = (p1.orientation.x + p2.orientation.x) / 2.0;
        midpoint.pose.orientation.y = (p1.orientation.y + p2.orientation.y) / 2.0;
        midpoint.pose.orientation.z = (p1.orientation.z + p2.orientation.z) / 2.0;
        midpoint.pose.orientation.w = (p1.orientation.w + p2.orientation.w) / 2.0;

        // Set the header
        midpoint.header.stamp = ros::Time::now();
        midpoint.header.frame_id = "world";  // Set the frame_id appropriately

        return midpoint;
    }

    void OdomCallbackFalcon1(const nav_msgs::Odometry::ConstPtr &msg) {
        p1 = msg->pose.pose;  // Update the current pose of Falcon1
    }

    void OdomCallbackFalcon2(const nav_msgs::Odometry::ConstPtr &msg) {
        p2 = msg->pose.pose;  // Update the current pose of Falcon2
    }

    void turn() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            // Increment the angle to make the drones rotate
            angle += degreesToRadians(5);  // Adjust the increment value as needed

            if (angle >= 2 * M_PI) {
                angle -= 2 * M_PI;
            }

            // Calculate new positions for the drones
            geometry_msgs::PoseStamped pose_falcon1;
            pose_falcon1.header.stamp = ros::Time::now();
            pose_falcon1.header.frame_id = "world";
            pose_falcon1.pose.position.x = midpoint.pose.position.x + radius * cos(angle);
            pose_falcon1.pose.position.y = midpoint.pose.position.y + radius * sin(angle);
            pose_falcon1.pose.position.z = midpoint.pose.position.z;  // Assuming rotation in the XY plane
            pose_falcon1.pose.orientation.w = 1.0;  // Keep the orientation unchanged

            geometry_msgs::PoseStamped pose_falcon2;
            pose_falcon2.header.stamp = ros::Time::now();
            pose_falcon2.header.frame_id = "world";
            pose_falcon2.pose.position.x = midpoint.pose.position.x + radius * cos(angle + M_PI);  // Opposite side
            pose_falcon2.pose.position.y = midpoint.pose.position.y + radius * sin(angle + M_PI);
            pose_falcon2.pose.position.z = midpoint.pose.position.z;  // Assuming rotation in the XY plane
            pose_falcon2.pose.orientation.w = 1.0;  // Keep the orientation unchanged

            // Publish new positions
            pose_pub_falcon1.publish(pose_falcon1);
            pose_pub_falcon2.publish(pose_falcon2);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "PositionController_turn");

    double end_angle_degrees;
    std::string drone_id1, drone_id2;

    ros::NodeHandle nh("~");

    nh.param("end_angle_degrees", end_angle_degrees, 90.0);
    nh.param("drone_id1", drone_id1, std::string("falcon"));
    nh.param("drone_id2", drone_id2, std::string("falcon1"));

    std::cout << "end_angle_degrees: " << end_angle_degrees << std::endl;

    TurnHandler turn_handler(drone_id1, drone_id2, end_angle_degrees);
    turn_handler.turn();

    return 0;
}
