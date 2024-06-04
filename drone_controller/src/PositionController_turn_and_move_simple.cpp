#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include <cmath>
#include <thread>
#include <chrono>
#include <tf/tf.h>
#include <mutex>

class CirclePathController {
public:
    CirclePathController() : initial_positions_set(false), centerX(0.0), centerY(0.0), centerZ(1.0), startX(0.0), startY(0.0),
                             target_x(2.0), target_y(2.0), target_z(1.0), degree_increment(30.0), etha_yaw(0.0), etha_translation(0.0) {
        nh = ros::NodeHandle("~");
        std::string drone_id1, drone_id2;
        nh.getParam("drone_id1", drone_id1);
        nh.getParam("drone_id2", drone_id2);
        nh.getParam("/BEP/Position_turn_and_move_simple_node/target_x", target_x);
        nh.getParam("/BEP/Position_turn_and_move_simple_node/target_y", target_y);
        nh.getParam("/BEP/Position_turn_and_move_simple_node/target_z", target_z);
        nh.getParam("end_angle_degrees", end_angle_degrees);
        nh.getParam("increment_time", increment_time);
        nh.getParam("increment_degrees", increment_degrees);
        nh.getParam("etha_yaw", etha_yaw);
        nh.getParam("etha_translation", etha_translation);

        odom_sub_falcon1 = nh.subscribe("/" + drone_id1 + "/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon1, this);
        odom_sub_falcon2 = nh.subscribe("/" + drone_id2 + "/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon2, this);
        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id1 + "/agiros_pilot/go_to_pose", 2);
        pose_pub_falcon2 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id2 + "/agiros_pilot/go_to_pose", 2);
        timer = nh.createTimer(ros::Duration(increment_time), &CirclePathController::updateCallback, this, false, false);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub_falcon1, pose_pub_falcon2;
    ros::Publisher land_pub_1, land_pub_2;  // Add these as member variables
    ros::Subscriber odom_sub_falcon1, odom_sub_falcon2;
    ros::Timer timer, initial_position_timer, shutdown_timer;
    bool initial_positions_set;
    double centerX, centerY, centerZ;
    double startX, startY;
    double target_x, target_y, target_z;
    double angle_degrees, start_angle_degrees;
    double end_angle_degrees, increment_time, increment_degrees;
    double degree_increment, increment_x, increment_y;
    double roll, pitch, current_yaw;
    double etha_yaw, etha_translation;  // Declare these as member variables
    geometry_msgs::Point initial_position_falcon1, initial_position_falcon2, current_position_bar;
    std::mutex data_mutex; // Mutex for synchronizing data access

    void normalizeEndAngleDegrees() {
        if (end_angle_degrees > 180.0) {
            end_angle_degrees -= 360.0;
        } else if (end_angle_degrees < -180.0) {
            end_angle_degrees += 360.0;
        }
    }

    void odometryCallbackFalcon1(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initial_positions_set) {
            initial_position_falcon1 = msg->pose.pose.position;
            checkInitialPositionsSet();
        }
    }

    void odometryCallbackFalcon2(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initial_positions_set) {
            initial_position_falcon2 = msg->pose.pose.position;
            checkInitialPositionsSet();
        }
    }

    void checkInitialPositionsSet() {
        if (initial_position_falcon1.x != 0.0000000 && initial_position_falcon2.x != 0.0000000) {
            startX = (initial_position_falcon1.x + initial_position_falcon2.x) / 2;
            startY = (initial_position_falcon1.y + initial_position_falcon2.y) / 2;
            double dx = initial_position_falcon1.x - startX;
            double dy = initial_position_falcon1.y - startY;
            centerX = startX;
            centerY = startY;
            normalizeEndAngleDegrees();
            start_angle_degrees = atan2(dy, dx) * 180.0 / M_PI; // Deze is sowieso tussen -180 en 180 
            angle_degrees = start_angle_degrees;
            if (end_angle_degrees > start_angle_degrees && end_angle_degrees - start_angle_degrees > 180.0) {
                degree_increment = -increment_degrees;  //bvb van -90 naar 180 graden
            } else if (end_angle_degrees > start_angle_degrees && end_angle_degrees - start_angle_degrees <= 180.0) {
                degree_increment = increment_degrees;    // bvb can 0 naar 90 graden
            } else if (end_angle_degrees < start_angle_degrees && end_angle_degrees - start_angle_degrees > 180.0) {
                degree_increment = increment_degrees;    // bvb van 180 naar -90 graden
            } else if (end_angle_degrees < start_angle_degrees && end_angle_degrees - start_angle_degrees <= 180.0) {
                degree_increment = -increment_degrees;   // bvb van 90 naar 0 graden
            }
            if (end_angle_degrees - start_angle_degrees == 0.0) {
                increment_x = target_x - startX;
                increment_y = target_y - startY;
            } else {
                increment_x = (target_x - startX) / std::abs((end_angle_degrees - start_angle_degrees) / degree_increment);
                increment_y = (target_y - startY) / std::abs((end_angle_degrees - start_angle_degrees) / degree_increment);
            }

            initial_positions_set = true;
            ROS_INFO("Initial info: startX=%f, startY=%f, start angle=%f, start degree increment=%f, start increment x=%f, start increment y=%f", startX, startY, angle_degrees, degree_increment, increment_x, increment_y);

            // Start a one-time timer of 1 second before starting the regular updateCallback
            initial_position_timer = nh.createTimer(ros::Duration(0.1), &CirclePathController::startRegularTimerCallback, this, true);
        }
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        timer.start();
    }

    void updateCallback(const ros::TimerEvent&) {
        double next_angle = angle_degrees + degree_increment;

        if (end_angle_degrees > start_angle_degrees) {
            if (next_angle >= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
                
            } else {
                angle_degrees = next_angle;
            }
        } else {
            if (next_angle <= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
            } else {
                angle_degrees = next_angle;
            }
        }
        updateDronesPosition();
    }

    void updateDronesPosition() {
        double radius = 0.5;
        double angle_rad = degreesToRadians(angle_degrees);
        centerX += increment_x;
        centerY += increment_y;
        if (startX <= target_x && centerX >= target_x) {
            centerX = target_x;
        } 
        if (startX >= target_x && centerX <= target_x) {
            centerX = target_x;
        }
        if (startY <= target_y && centerY >= target_y) {
            centerY = target_y;
        }
        if (startY >= target_y && centerY <= target_y) {
            centerY = target_y;
        }
        
        centerZ = target_z;

        geometry_msgs::PoseStamped new_pose_falcon1, new_pose_falcon2;
        new_pose_falcon1.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_falcon1.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_falcon1.pose.position.z = centerZ + 1.0;
        new_pose_falcon2.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_falcon2.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_falcon2.pose.position.z = centerZ + 1.0;
        
        // Orientation
        double half_angle_rad = angle_rad / 2;
        new_pose_falcon1.pose.orientation.z = sin(half_angle_rad);
        new_pose_falcon1.pose.orientation.w = cos(half_angle_rad);
        new_pose_falcon2.pose.orientation.z = sin(half_angle_rad + M_PI);
        new_pose_falcon2.pose.orientation.w = cos(half_angle_rad + M_PI);

        pose_pub_falcon1.publish(new_pose_falcon1);
        pose_pub_falcon2.publish(new_pose_falcon2);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("Center position: x=%f, y=%f, z-%f", centerX, centerY, centerZ);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
