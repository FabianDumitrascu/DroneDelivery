#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <thread>
#include <chrono>

class CirclePathController {
public:
    CirclePathController() : initial_positions_set(false), centerX(0.0), centerY(0.0), centerZ(1.0), startX(0.0), startY(0.0), startZ(1.0),
                             target_x(2.0), target_y(2.0), target_z(1.0), degree_increment(30.0) {
        nh = ros::NodeHandle("~");

        if (!nh.getParam("end_angle_degrees", end_angle_degrees)) {
            ROS_WARN("Could not retrieve 'end_angle_degrees' from the parameter server; defaulting to 90 degrees");
            end_angle_degrees = 90.0;
        }

        nh.getParam("/BEP/Position_turn_and_move_node/target_x", target_x);
        nh.getParam("/BEP/Position_turn_and_move_node/target_y", target_y);
        nh.getParam("/BEP/Position_turn_and_move_node/target_z", target_z);

        //odom_sub_bar = nh.subscribe("/bar/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackBar, this);
        //odom_sub_flycrane = nh.subscribe("/flycrane/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackflycrane, this);
        //odom_sub_flycrane1 = nh.subscribe("/flycrane1/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackflycrane1, this);

        pose_pub_flycrane = nh.advertise<geometry_msgs::PoseStamped>("/flycrane/agiros_pilot/go_to_pose", 2);
        pose_pub_flycrane1 = nh.advertise<geometry_msgs::PoseStamped>("/flycrane1/agiros_pilot/go_to_pose", 2);
        

        timer = nh.createTimer(ros::Duration(0.8), &CirclePathController::updateCallback, this, false, false);
        
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub_flycrane, pose_pub_flycrane1;
    ros::Subscriber odom_sub_flycrane, odom_sub_flycrane1;
    ros::Timer timer, initial_position_timer, shutdown_timer;
    bool initial_positions_set;
    double centerX, centerY, centerZ;
    double startX, startY, startZ;
    double target_x, target_y, target_z;
    double angle_degrees, start_angle_degrees;
    double end_angle_degrees;
    double degree_increment, increment_x, increment_y, increment_z;
    geometry_msgs::Point initial_position_bar;

    void normalizeEndAngleDegrees() {
        if (end_angle_degrees > 180.0) {
            end_angle_degrees -= 360.0;
        } else if (end_angle_degrees < -180.0) {
            end_angle_degrees += 360.0;
        }
    }

    //void odometryCallbackflycrane(const nav_msgs::Odometry::ConstPtr& msg) {
      //  if (initial_positions_set==false) {
        //    initial_position_flycrane = msg->pose.pose.position;
          //  checkInitialPositionsSet();
        //}
    //}

    //void odometryCallbackflycrane1(const nav_msgs::Odometry::ConstPtr& msg) {
      //  if (initial_positions_set==false) {
       //     initial_position_flycrane1 = msg->pose.pose.position;
         //   checkInitialPositionsSet();
        //}
    //}
    void odometryCallbackBar(const nav_msgs::Odometry::ConstPtr& msg) {
        if (initial_positions_set==false) {
            initial_position_bar = msg->pose.pose.position;
            checkInitialPositionsSet();
        }
    }

    void checkInitialPositionsSet() {
        if (initial_position_bar.x != 0.0000000) {
            startX = initial_position_bar.x;
            startY = initial_position_bar.y;
            startZ = initial_position_bar.z;
            start_angle_degrees = initial_position_bar.z;
            centerX = startX;
            centerY = startY;
            normalizeEndAngleDegrees();
            
            angle_degrees = start_angle_degrees;
            if (end_angle_degrees > start_angle_degrees and end_angle_degrees - start_angle_degrees > 180.0) {
                degree_increment = -30.0;  //bvb van -90 naar 180 graden
            } else if (end_angle_degrees > start_angle_degrees and end_angle_degrees - start_angle_degrees <= 180.0) {
                degree_increment = 30.0;    // bvb can 0 naar 90 graden
            } else if (end_angle_degrees < start_angle_degrees and end_angle_degrees - start_angle_degrees > 180.0) {
                degree_increment = 30.0;    // bvb van 180 naar -90 graden
            } else if (end_angle_degrees < start_angle_degrees and end_angle_degrees - start_angle_degrees <= 180.0) {
                degree_increment = -30.0;   // bvb van 90 naar 0 graden
            }
            if (end_angle_degrees-start_angle_degrees == 0.0) {
                increment_x = target_x - startX;
                increment_y = target_y - startY;
                increment_z = target_z - startZ;
            } else {
                increment_x = (target_x - startX) / std::abs((end_angle_degrees-start_angle_degrees) / degree_increment);
                increment_y = (target_y - startY) / std::abs((end_angle_degrees-start_angle_degrees) / degree_increment);
                increment_z = (target_z - startZ) / std::abs((end_angle_degrees-start_angle_degrees) / degree_increment);
            }

            initial_positions_set = true;
            ROS_INFO("Initial info: startX=%f, startY=%f, start angle=%f, start degree increment=%f, start increment x=%f, start increment y=%f", startX, startY, angle_degrees, degree_increment, increment_x, increment_y);

            // Start a one-time timer of 1 second before starting the regular updateCallback
            initial_position_timer = nh.createTimer(ros::Duration(0.1), &CirclePathController::startRegularTimerCallback, this, true);
        }
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        // Start the regular timer after a delay of 1 second
        timer.start();
    }

    void updateCallback(const ros::TimerEvent&) {
        updateDronesPosition();
        
        double next_angle = angle_degrees + degree_increment;
        if (end_angle_degrees > start_angle_degrees) {
            if (next_angle >= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
                shutdown_timer = nh.createTimer(ros::Duration(10), [this](const ros::TimerEvent&) {
                    ROS_INFO("Shutting down after delay.");
                    ros::shutdown();
                }, true); // true makes it a one-shot timer
            } else {
                angle_degrees = next_angle;
            }
        } else {
            if (next_angle <= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
                shutdown_timer = nh.createTimer(ros::Duration(10), [this](const ros::TimerEvent&) {
                    ROS_INFO("Shutting down after delay.");
                    ros::shutdown();
                }, true); // true makes it a one-shot timer
            } else {
                angle_degrees = next_angle;
            }
        }
    }

    void updateDronesPosition() {
        double radius = 1.0;
        double angle_rad = degreesToRadians(angle_degrees);

        // Update `centerX` and `centerY`
        centerX += increment_x;
        centerY += increment_y;
        centerZ += increment_z;
        if (startX <= target_x and centerX >= target_x) {
            centerX = target_x;
        } 
        if (startX >= target_x and centerX <= target_x) {
            centerX = target_x;
        }
        if (startY <= target_y and centerY >= target_y) {
            centerY = target_y;
        }
        if (startY >= target_y and centerY <= target_y) {
            centerY = target_y;
        }
        if (startZ <= target_z and centerZ >= target_z) {
            centerZ = target_z;
        }
        if (startZ >= target_z and centerZ <= target_z) {
        centerZ = target_z;
        }
        geometry_msgs::PoseStamped new_pose_flycrane, new_pose_flycrane1;
        new_pose_flycrane.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_flycrane.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_flycrane.pose.position.z = centerZ;
        new_pose_flycrane1.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_flycrane1.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_flycrane1.pose.position.z = centerZ;
        
        // Orientation
        double half_angle_rad = angle_rad / 2;
        new_pose_flycrane.pose.orientation.z = sin(half_angle_rad);
        new_pose_flycrane.pose.orientation.w = cos(half_angle_rad);
        new_pose_flycrane1.pose.orientation.z = sin(half_angle_rad + M_PI);
        new_pose_flycrane1.pose.orientation.w = cos(half_angle_rad + M_PI);

        pose_pub_flycrane.publish(new_pose_flycrane);
        pose_pub_flycrane1.publish(new_pose_flycrane1);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("Center position: x=%f, y=%f", centerX, centerY);
        //ROS_INFO("flycrane position: x=%f, y=%f, z=%f", new_pose_flycrane.pose.position.x, new_pose_flycrane.pose.position.y, new_pose_flycrane.pose.position.z);
        //ROS_INFO("flycrane1 position: x=%f, y=%f, z=%f", new_pose_flycrane1.pose.position.x, new_pose_flycrane1.pose.position.y, new_pose_flycrane1.pose.position.z);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wait 4 seconds before takeoff
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
