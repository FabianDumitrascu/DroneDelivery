#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

class CirclePathController {
public:
    ros::NodeHandle nh;
    CirclePathController() : initial_angle_set(false), num_initial_positions_received(0) {
        

        // Initialize the end_angle_degrees from ROS parameter server
        if (!nh.getParam("/BEP/Position_turn_node/end_angle_degrees", end_angle_degrees)) {
            ROS_WARN("Could not retrieve 'end_angle_degrees' from the parameter server; defaulting to 90 degrees");
            end_angle_degrees = 90.0;  // Default value if not set
        }
        // Subscribe to odometry topics
        odom_sub_falcon = nh.subscribe("/falcon/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon, this);
        odom_sub_falcon1 = nh.subscribe("/falcon1/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon1, this);

        // Other initializations
        pose_pub_falcon = nh.advertise<geometry_msgs::PoseStamped>("/falcon/agiros_pilot/go_to_pose", 10);
        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/falcon1/agiros_pilot/go_to_pose", 10);
        timer = nh.createTimer(ros::Duration(1), &CirclePathController::updateCallback, this);
    }

private:
    ros::Publisher pose_pub_falcon, pose_pub_falcon1;
    ros::Subscriber odom_sub_falcon, odom_sub_falcon1;
    ros::Timer timer, shutdown_timer; 
    double angle_degrees, end_angle_degrees;
    bool initial_angle_set;
    int num_initial_positions_received;
    geometry_msgs::Point current_position_falcon, current_position_falcon1;

        void odometryCallbackFalcon(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_falcon = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
    }

    void odometryCallbackFalcon1(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_falcon1 = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
    }

    void calculateInitialAngles() {
        double centerX = 0.0, centerY = 0.0;
        double dx = current_position_falcon.x - centerX;
        double dy = current_position_falcon.y - centerY;
        angle_degrees = atan2(dy, dx) * 180.0 / M_PI;

        initial_angle_set = true;

        // Start de timer nadat de initiële hoek is ingesteld
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(1), &CirclePathController::updateCallback, this);
    }

    void updateCallback(const ros::TimerEvent&) {
        if (!initial_angle_set) {
            ROS_WARN("Initial angles not yet set. Skipping update.");
            return;
        }

        double radius = 1.0;
        double centerX = 0.0, centerY = 0.0, centerZ = 1.0;
        double angle_rad = degreesToRadians(angle_degrees);

        geometry_msgs::PoseStamped new_pose_falcon, new_pose_falcon1;
        new_pose_falcon.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_falcon.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_falcon.pose.position.z = centerZ;

        new_pose_falcon1.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_falcon1.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_falcon1.pose.position.z = centerZ;
        // Assuming angle_rad is the yaw rotation desired around the Z-axis
        double half_angle_rad = angle_rad / 2;
        new_pose_falcon.pose.orientation.x = 0.0;
        new_pose_falcon.pose.orientation.y = 0.0;
        new_pose_falcon.pose.orientation.z = sin(half_angle_rad);
        new_pose_falcon.pose.orientation.w = cos(half_angle_rad);

        new_pose_falcon1.pose.orientation.x = 0.0;
        new_pose_falcon1.pose.orientation.y = 0.0;
        new_pose_falcon1.pose.orientation.z = sin(half_angle_rad + M_PI); // Opposite direction for the second drone
        new_pose_falcon1.pose.orientation.w = cos(half_angle_rad + M_PI);
        pose_pub_falcon.publish(new_pose_falcon);
        pose_pub_falcon1.publish(new_pose_falcon1);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("Falcon position: x=%f, y=%f, z=%f", new_pose_falcon.pose.position.x, new_pose_falcon.pose.position.y, new_pose_falcon.pose.position.z);
        ROS_INFO("Falcon1 position: x=%f, y=%f, z=%f", new_pose_falcon1.pose.position.x, new_pose_falcon1.pose.position.y, new_pose_falcon1.pose.position.z);

        double next_angle = angle_degrees + 15.0;
        if (next_angle >= end_angle_degrees) {
            angle_degrees = end_angle_degrees;
            ROS_INFO("Reached end angle: %f degrees", angle_degrees);
            timer.stop(); // Stop the main timer to prevent further updates
            shutdown_timer = nh.createTimer(ros::Duration(2), [this](const ros::TimerEvent&) {
                ROS_INFO("Shutting down after delay.");
                ros::shutdown();
            }, true); // true makes it a one-shot timer
        } else if (next_angle > 360) {
            angle_degrees = next_angle - 360;
        } else {
            angle_degrees = next_angle;
        }
    }


    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(10)); // Wacht 10 seconden voor takeoff
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
