#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

class CirclePathController {
public:
    CirclePathController() : initial_angle_set(false), num_initial_positions_received(0), 
                             centerX(0.0), centerY(0.0), centerZ(1.0),
                             startX(0.0), startY(0.0), target_x(2.0), target_y(2.0), degree_increment(30.0) {
        // Initialize ROS NodeHandle
        nh = ros::NodeHandle("~");

        // Retrieve the end_angle_degrees from the ROS parameter server
        if (!nh.getParam("end_angle_degrees", end_angle_degrees)) {
            ROS_WARN("Could not retrieve 'end_angle_degrees' from the parameter server; defaulting to 90 degrees");
            end_angle_degrees = 90.0;  // Default value if not set
        }

        // Retrieve target coordinates
        nh.getParam("/BEP/Position_turn_and_move_node/target_x", target_x);
        nh.getParam("/BEP/Position_turn_and_move_node/target_y", target_y);

        // Subscribe to odometry topics
        odom_sub_falcon = nh.subscribe("/falcon/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon, this);
        odom_sub_falcon1 = nh.subscribe("/falcon1/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon1, this);

        // Publishers
        pose_pub_falcon = nh.advertise<geometry_msgs::PoseStamped>("/falcon/agiros_pilot/go_to_pose", 10);
        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/falcon1/agiros_pilot/go_to_pose", 10);

        // Timer setup
        timer = nh.createTimer(ros::Duration(0.5), &CirclePathController::updateCallback, this, false, false);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub_falcon, pose_pub_falcon1;
    ros::Subscriber odom_sub_falcon, odom_sub_falcon1;
    ros::Timer timer, shutdown_timer, initial_position_timer;
    double angle_degrees, end_angle_degrees;
    bool initial_angle_set;
    int num_initial_positions_received;
    double centerX, centerY, centerZ;
    double startX, startY; // Store the initial center coordinates
    double target_x, target_y; // Target coordinates
    double degree_increment;
    geometry_msgs::Point current_position_falcon, current_position_falcon1;

    void odometryCallbackFalcon(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_falcon = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
        // Update startX with the middle x coordinate
        startX = (current_position_falcon.x + current_position_falcon1.x) / 2;
    }

    void odometryCallbackFalcon1(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_falcon1 = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
        // Update startY with the middle y coordinate
        startY = (current_position_falcon.y + current_position_falcon1.y) / 2;
    }

    void calculateInitialAngles() {
        double dx = current_position_falcon.x - startX;
        double dy = current_position_falcon.y - startY;
        angle_degrees = atan2(dy, dx) * 180.0 / M_PI;

        initial_angle_set = true;

        // Publish the first position directly
        updateDronesPosition();

        // Start a one-time timer of 1 second before starting the regular updateCallback
        initial_position_timer = nh.createTimer(ros::Duration(1), &CirclePathController::startRegularTimerCallback, this, true);
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        // Start the regular timer after a delay of 1 second
        timer.start();
    }

    void updateCallback(const ros::TimerEvent&) {
        if (!initial_angle_set) {
            ROS_WARN("Initial angles not yet set. Skipping update.");
            return;
        }

        updateDronesPosition();
        
        double next_angle = angle_degrees + degree_increment;
        if (next_angle >= end_angle_degrees) {
            angle_degrees = end_angle_degrees;
            ROS_INFO("Reached end angle: %f degrees", angle_degrees);
            timer.stop(); // Stop the main timer to prevent further updates

            // Set a one-time timer to shut down after 10 seconds
            shutdown_timer = nh.createTimer(ros::Duration(11), [this](const ros::TimerEvent&) {
                ROS_INFO("Shutting down after delay.");
                ros::shutdown();
            }, true); // true makes it a one-shot timer
        } else if (next_angle > 360) {
            angle_degrees = next_angle - 360;
        } else {
            angle_degrees = next_angle;
        }
    }

    void updateDronesPosition() {
        double radius = 1.0;
        double angle_rad = degreesToRadians(angle_degrees);

        // Calculate increments for `centerX` and `centerY`
        double increment_x = (target_x - startX) / (end_angle_degrees / degree_increment);
        double increment_y = (target_y - startY) / (end_angle_degrees / degree_increment);

        // Update `centerX` and `centerY`
        centerX += increment_x;
        if (centerX >= target_x) {
            centerX = target_x;
            ROS_INFO("Reached end x: %f x", target_x);
        }
        centerY += increment_y;
        if (centerY >= target_y) {
            centerY = target_y;
            ROS_INFO("Reached end y: %f y", target_y);
        }

        geometry_msgs::PoseStamped new_pose_falcon, new_pose_falcon1;
        new_pose_falcon.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_falcon.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_falcon.pose.position.z = centerZ;
        new_pose_falcon1.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_falcon1.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_falcon1.pose.position.z = centerZ;
        
        // Orientation
        double half_angle_rad = angle_rad / 2;
        new_pose_falcon.pose.orientation.x = 0.0;
        new_pose_falcon.pose.orientation.y = 0.0;
        new_pose_falcon.pose.orientation.z = sin(half_angle_rad);
        new_pose_falcon.pose.orientation.w = cos(half_angle_rad);

        new_pose_falcon1.pose.orientation.x = 0.0;
        new_pose_falcon1.pose.orientation.y = 0.0;
        new_pose_falcon1.pose.orientation.z = sin(half_angle_rad + M_PI);
        new_pose_falcon1.pose.orientation.w = cos(half_angle_rad + M_PI);

        pose_pub_falcon.publish(new_pose_falcon);
        pose_pub_falcon1.publish(new_pose_falcon1);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("Center position: x=%f, y=%f", centerX, centerY);
        ROS_INFO("Falcon position: x=%f, y=%f, z=%f", new_pose_falcon.pose.position.x, new_pose_falcon.pose.position.y, new_pose_falcon.pose.position.z);
        ROS_INFO("Falcon1 position: x=%f, y=%f, z=%f", new_pose_falcon1.pose.position.x, new_pose_falcon1.pose.position.y, new_pose_falcon1.pose.position.z);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(6)); // Wait 6 seconds before takeoff
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
