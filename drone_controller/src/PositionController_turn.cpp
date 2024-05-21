#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

class CirclePathController {
public:
    CirclePathController() : initial_angle_set(false), num_initial_positions_received(0) {
        // Initialize ROS NodeHandle
        nh = ros::NodeHandle("~");

        // Initialize the end_angle_degrees from ROS parameter server
        if (!nh.getParam("end_angle_degrees", end_angle_degrees)) {
            ROS_WARN("Could not retrieve 'end_angle_degrees' from the parameter server; defaulting to 90 degrees");
            end_angle_degrees = 90.0;  // Default value if not set
        }

        // Subscribe to odometry topics
        odom_sub_flycrane = nh.subscribe("/flycrane/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackflycrane, this);
        odom_sub_flycrane1 = nh.subscribe("/flycrane1/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackflycrane1, this);

        // Publishers
        pose_pub_flycrane = nh.advertise<geometry_msgs::PoseStamped>("/flycrane/agiros_pilot/go_to_pose", 10);
        pose_pub_flycrane1 = nh.advertise<geometry_msgs::PoseStamped>("/flycrane1/agiros_pilot/go_to_pose", 10);

        // Timer setup
        timer = nh.createTimer(ros::Duration(0.5), &CirclePathController::updateCallback, this, false, false);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub_flycrane, pose_pub_flycrane1;
    ros::Subscriber odom_sub_flycrane, odom_sub_flycrane1;
    ros::Timer timer, shutdown_timer, initial_position_timer;
    double angle_degrees, end_angle_degrees;
    bool initial_angle_set;
    int num_initial_positions_received;
    geometry_msgs::Point current_position_flycrane, current_position_flycrane1;

    void odometryCallbackflycrane(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_flycrane = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
    }

    void odometryCallbackflycrane1(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_flycrane1 = msg->pose.pose.position;
        if (++num_initial_positions_received == 2 && !initial_angle_set) {
            calculateInitialAngles();
        }
    }

    void calculateInitialAngles() {
        double centerX = 0.0, centerY = 0.0;
        double dx = current_position_flycrane.x - centerX;
        double dy = current_position_flycrane.y - centerY;
        angle_degrees = atan2(dy, dx) * 180.0 / M_PI;

        initial_angle_set = true;

        // Publiceer direct de eerste positie
        updateDronesPosition();

        // Start de eenmalige timer van 1 seconde voordat de reguliere updateCallback begint
        initial_position_timer = nh.createTimer(ros::Duration(1), &CirclePathController::startRegularTimerCallback, this, true);
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        // Start de reguliere timer na een vertraging van 1 seconde
        timer.start();
    }

    void updateCallback(const ros::TimerEvent&) {
        if (!initial_angle_set) {
            ROS_WARN("Initial angles not yet set. Skipping update.");
            return;
        }

        updateDronesPosition();

        double next_angle = angle_degrees + 15.0;
        if (next_angle >= end_angle_degrees) {
            angle_degrees = end_angle_degrees;
            ROS_INFO("Reached end angle: %f degrees", angle_degrees);
            timer.stop(); // Stop de hoofd-timer om verdere updates te voorkomen

            // Stel een eenmalige timer in om na 2 seconden af te sluiten
            shutdown_timer = nh.createTimer(ros::Duration(2), [this](const ros::TimerEvent&) {
                ROS_INFO("Shutting down after delay.");
                ros::shutdown();
            }, true); // true maakt het een eenmalige timer
        } else if (next_angle > 360) {
            angle_degrees = next_angle - 360;
        } else {
            angle_degrees = next_angle;
        }
    }

    void updateDronesPosition() {
        double radius = 1.0;
        double centerX = 0.0, centerY = 0.0, centerZ = 1.0;
        double angle_rad = degreesToRadians(angle_degrees);

        geometry_msgs::PoseStamped new_pose_flycrane, new_pose_flycrane1;
        new_pose_flycrane.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_flycrane.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_flycrane.pose.position.z = centerZ;

        new_pose_flycrane1.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_flycrane1.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_flycrane1.pose.position.z = centerZ;

        // Orientation
        double half_angle_rad = angle_rad / 2;
        new_pose_flycrane.pose.orientation.x = 0.0;
        new_pose_flycrane.pose.orientation.y = 0.0;
        new_pose_flycrane.pose.orientation.z = sin(half_angle_rad);
        new_pose_flycrane.pose.orientation.w = cos(half_angle_rad);

        new_pose_flycrane1.pose.orientation.x = 0.0;
        new_pose_flycrane1.pose.orientation.y = 0.0;
        new_pose_flycrane1.pose.orientation.z = sin(half_angle_rad + M_PI);
        new_pose_flycrane1.pose.orientation.w = cos(half_angle_rad + M_PI);

        pose_pub_flycrane.publish(new_pose_flycrane);
        pose_pub_flycrane1.publish(new_pose_flycrane1);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("flycrane position: x=%f, y=%f, z=%f", new_pose_flycrane.pose.position.x, new_pose_flycrane.pose.position.y, new_pose_flycrane.pose.position.z);
        ROS_INFO("flycrane1 position: x=%f, y=%f, z=%f", new_pose_flycrane1.pose.position.x, new_pose_flycrane1.pose.position.y, new_pose_flycrane1.pose.position.z);
    
        double next_angle = angle_degrees + 10.0;
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
    std::this_thread::sleep_for(std::chrono::seconds(9)); // Wacht 9 seconden voor takeoff
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
