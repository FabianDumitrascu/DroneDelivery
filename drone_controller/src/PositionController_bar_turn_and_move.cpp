#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

class CirclePathController {
public:
    CirclePathController(double radius, double end_angle_degrees, int steps = -1) 
        : centerX(0.0), centerY(0.0), centerZ(1.0), startX(0.0), startY(0.0), 
          degree_increment(30.0), radius(radius), end_angle_degrees(end_angle_degrees),
          init_position_flycrane_set(false), init_position_flycrane1_set(false),
          target_x(2.0), target_y(2.0), target_z(1.0), angle_degrees(0.0), start_angle_degrees(0.0), steps(steps) {

        nh = ros::NodeHandle("~");

        if (!nh.getParam("end_angle_degrees", end_angle_degrees)) {
            ROS_WARN("Could not retrieve 'end_angle_degrees' from the parameter server; defaulting to 90 degrees");
            end_angle_degrees = 90.0;
        }

        if (!nh.getParam("target_x", target_x)) {
            ROS_WARN("Could not retrieve 'target_x' from the parameter server; defaulting to 2.0");
            target_x = 2.0;
        }

        if (!nh.getParam("target_y", target_y)) {
            ROS_WARN("Could not retrieve 'target_y' from the parameter server; defaulting to 2.0");
            target_y = 2.0;
        }

        if (!nh.getParam("target_z", target_z)) {
            ROS_WARN("Could not retrieve 'target_z' from the parameter server; defaulting to 1.0");
            target_z = 1.0;
        }

        if (steps == -1) {
            steps = calculateSteps();
        }

        odom_sub_flycrane = nh.subscribe("/flycrane/agiros_pilot/odometry", 1, &CirclePathController::OdometryCallbackFlycrane, this);
        odom_sub_flycrane1 = nh.subscribe("/flycrane1/agiros_pilot/odometry", 1, &CirclePathController::OdometryCallbackFlycrane1, this);

        pose_pub_flycrane = nh.advertise<geometry_msgs::PoseStamped>("/flycrane/agiros_pilot/go_to_pose", 2);
        pose_pub_flycrane1 = nh.advertise<geometry_msgs::PoseStamped>("/flycrane1/agiros_pilot/go_to_pose", 2);
        

        timer = nh.createTimer(ros::Duration(0.8), &CirclePathController::updateCallback, this, false, false);
        
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub_flycrane, pose_pub_flycrane1;
    ros::Subscriber odom_sub_flycrane, odom_sub_flycrane1;
    ros::Timer timer, shutdown_timer;
    bool init_position_flycrane_set, init_position_flycrane1_set;
    int steps;
    double radius;
    double centerX, centerY, centerZ;
    double startX, startY;
    double target_x, target_y, target_z;
    double angle_degrees, start_angle_degrees;
    double end_angle_degrees;
    double degree_increment;
    geometry_msgs::Point initial_position_flycrane, initial_position_flycrane1;


    void normalizeEndAngleDegrees() {
        // if (end_angle_degrees > 180.0) {
        //     end_angle_degrees -= 360.0;
        // } else if (end_angle_degrees < -180.0) {
        //     end_angle_degrees += 360.0;
        // }
    }

    void OdometryCallbackFlycrane(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!InitialPositionsSet()) {
            init_position_flycrane_set = true;
            initial_position_flycrane = msg->pose.pose.position;
            ROS_INFO("The initial position of flycrane is set at x=%f, y=%f, z=%f", initial_position_flycrane.x, initial_position_flycrane.y, initial_position_flycrane.z);
        }
    }

    void OdometryCallbackFlycrane1(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!InitialPositionsSet()) {
            init_position_flycrane1_set = true;
            initial_position_flycrane1 = msg->pose.pose.position;
            ROS_INFO("The initial position of flycrane1 is set at x=%f, y=%f, z=%f", initial_position_flycrane1.x, initial_position_flycrane1.y, initial_position_flycrane1.z);
        }
    }

    bool InitialPositionsSet() {
        return init_position_flycrane_set && init_position_flycrane1_set;
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        // Start the regular timer after a delay of 1 second
        timer.start();
    }

    void UpdatePositions() {
        // Update the center position
        double increment_x = (target_x - startX ) / static_cast<double>(steps);
        centerX += increment_x;

        pose_flycrane.pose.position.x = centerX; 
        pose_flycrane1.pose.position.x = centerX;
        pose_flycrane.pose.position.y = centerY + radius;
        pose_flycrane1.pose.position.y = centerY - radius;
        pose_flycrane.pose.position.z = centerZ;
        pose_flycrane1.pose.position.z = centerZ;
        pose_flycrane.pose.orientation.w = 1.0;
        pose_flycrane1.pose.orientation.w = 1.0;

        pose_pub_flycrane.publish(pose_flycrane);
        pose_pub_flycrane1.publish(pose_flycrane1);

        ROS_INFO("Updated positions: x=%f, y=%f, z=%f", centerX, centerY, centerZ);

        

    }

    void updateCallback(const ros::TimerEvent&) {
        // updateDronesPosition();
        
        // double next_angle = angle_degrees + degree_increment;
        // if (end_angle_degrees > start_angle_degrees) {
        //     if (next_angle >= end_angle_degrees) {
        //         angle_degrees = end_angle_degrees;
        //         ROS_INFO("Reached end angle: %f degrees", angle_degrees);
        //         timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
        //         updateDronesPosition();
        //         shutdown_timer = nh.createTimer(ros::Duration(10), [this](const ros::TimerEvent&) {
        //             ROS_INFO("Shutting down after delay.");
        //             ros::shutdown();
        //         }, true); // true makes it a one-shot timer
        //     } else {
        //         angle_degrees = next_angle;
        //     }
        // } else {
        //     if (next_angle <= end_angle_degrees) {
        //         angle_degrees = end_angle_degrees;
        //         ROS_INFO("Reached end angle: %f degrees", angle_degrees);
        //         timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
        //         updateDronesPosition();
        //         shutdown_timer = nh.createTimer(ros::Duration(10), [this](const ros::TimerEvent&) {
        //             ROS_INFO("Shutting down after delay.");
        //             ros::shutdown();
        //         }, true); // true makes it a one-shot timer
        //     } else {
        //         angle_degrees = next_angle;
        //     }
        // }
    }

    void updateDronesPosition() {
        // double radius = 1.0;
        // double angle_rad = degreesToRadians(angle_degrees);

        

        // // Update `centerX` and `centerY`
        // centerX += increment_x;
        // centerY += increment_y;
        
        // centerZ = target_z;

        // geometry_msgs::PoseStamped new_pose_flycrane, new_pose_flycrane1;
        // new_pose_flycrane.pose.position.x = centerX + radius * cos(angle_rad);
        // new_pose_flycrane.pose.position.y = centerY + radius * sin(angle_rad);
        // new_pose_flycrane.pose.position.z = centerZ+1.0;
        // new_pose_flycrane1.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        // new_pose_flycrane1.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        // new_pose_flycrane1.pose.position.z = centerZ+1.0;
        
        // // Orientation
        // double half_angle_rad = angle_rad / 2;
        // new_pose_flycrane.pose.orientation.z = sin(half_angle_rad);
        // new_pose_flycrane.pose.orientation.w = cos(half_angle_rad);
        // new_pose_flycrane1.pose.orientation.z = sin(half_angle_rad + M_PI);
        // new_pose_flycrane1.pose.orientation.w = cos(half_angle_rad + M_PI);

        // pose_pub_flycrane.publish(new_pose_flycrane);
        // pose_pub_flycrane1.publish(new_pose_flycrane1);

        // ROS_INFO("Updated angle: %f", angle_degrees);
        // ROS_INFO("Center position: x=%f, y=%f", centerX, centerY);
        // //ROS_INFO("flycrane position: x=%f, y=%f, z=%f", new_pose_flycrane.pose.position.x, new_pose_flycrane.pose.position.y, new_pose_flycrane.pose.position.z);
        // //ROS_INFO("flycrane1 position: x=%f, y=%f, z=%f", new_pose_flycrane1.pose.position.x, new_pose_flycrane1.pose.position.y, new_pose_flycrane1.pose.position.z);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    int calculateSteps() {        
        return 4
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wait 4 seconds before takeoff
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller(1, 90.0);
    ros::spin();
    return 0;
}
