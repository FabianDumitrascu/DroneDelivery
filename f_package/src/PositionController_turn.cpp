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
    bool recievedPose1 = false;
    bool recievedPose2 = false;

public:
    TurnHandler(const std::string &_drone_id1, const std::string &_drone_id2, double _end_angle_degrees)
        : drone_id1(_drone_id1), drone_id2(_drone_id2), end_angle_degrees(_end_angle_degrees), nh("~"), radius(5.0), angle(0.0) {

        

        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id1 + "/agiros_pilot/go_to_pose", 10);
        pose_pub_falcon2 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id2 + "/agiros_pilot/go_to_pose", 10);

        odom_sub_falcon1 = nh.subscribe("/" + drone_id1 + "/agiros_pilot/odometry", 1, &TurnHandler::OdomCallbackFalcon1, this);
        odom_sub_falcon2 = nh.subscribe("/" + drone_id2 + "/agiros_pilot/odometry", 1, &TurnHandler::OdomCallbackFalcon2, this);

        GetParams();
    }

    void GetParams() {
        nh.param("drone_id1", drone_id1, std::string("falcon"));
        nh.param("drone_id2", drone_id2, std::string("falcon1"));
        nh.param("end_angle_degrees", end_angle_degrees, 0.0);
        nh.param("radius", radius, 5.0);  // Optional: Allow setting radius via parameter
    }

    void CalculateMidpoint() {
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
        ROS_INFO("Midpoint: (%f, %f, %f)", midpoint.pose.position.x, midpoint.pose.position.y, midpoint.pose.position.z);
        ROS_INFO("Orientation: (%f, %f, %f, %f)", midpoint.pose.orientation.x, midpoint.pose.orientation.y, midpoint.pose.orientation.z, midpoint.pose.orientation.w);
    }

    void CalculateAngle() {
        // Calculate the angle between the two drones
        double dx = p2.position.x - p1.position.x;
        double dy = p2.position.y - p1.position.y;
        angle = atan2(dy, dx);
        ROS_INFO("Distance dx: %f dy: %f", dx, dy);
        ROS_INFO("Angle between drones: %f", angle);
    }

    void OdomCallbackFalcon1(const nav_msgs::Odometry::ConstPtr &msg) {
        if(recievedPose1) {
            return;
        } else {
            p1 = msg->pose.pose;  // Update the current pose of Falcon1
            recievedPose1 = true;
            return;
        }
    }

    void OdomCallbackFalcon2(const nav_msgs::Odometry::ConstPtr &msg) {
        if(recievedPose2) {
            return;
        } else {
            p2 = msg->pose.pose;  // Update the current pose of Falcon2
            recievedPose2 = true;
            return;
        }
    }


    void turn() {
        ros::Rate loop_rate(10);
        geometry_msgs::PoseStamped command_falcon1, command_falcon2;
        double current_angle = angle;
        double target_angle = degreesToRadians(end_angle_degrees);
        double angular_step = degreesToRadians(10);  // Adjust this step for smoothness
        double epsilon = degreesToRadians(10.0);  // Set the epsilon value



        while (std::abs(current_angle - target_angle) > epsilon) {
            // Perform turning logic here
            command_falcon1.header.stamp = ros::Time::now();
            command_falcon1.header.frame_id = "world";
            command_falcon1.pose.position.x = midpoint.pose.position.x + radius * cos(current_angle);
            command_falcon1.pose.position.y = midpoint.pose.position.y + radius * sin(current_angle);
            command_falcon1.pose.position.z = midpoint.pose.position.z;
            command_falcon1.pose.orientation.x = 0.0;
            command_falcon1.pose.orientation.y = 0.0;
            command_falcon1.pose.orientation.z = sin(current_angle / 2.0);
            command_falcon1.pose.orientation.w = cos(current_angle / 2.0);
            pose_pub_falcon1.publish(command_falcon1);

            command_falcon2.header.stamp = ros::Time::now();
            command_falcon2.header.frame_id = "world";
            command_falcon2.pose.position.x = midpoint.pose.position.x + radius * cos(current_angle + M_PI);
            command_falcon2.pose.position.y = midpoint.pose.position.y + radius * sin(current_angle + M_PI);
            command_falcon2.pose.position.z = midpoint.pose.position.z;
            command_falcon2.pose.orientation.x = 0.0;
            command_falcon2.pose.orientation.y = 0.0;
            command_falcon2.pose.orientation.z = sin((current_angle + M_PI) / 2.0);
            command_falcon2.pose.orientation.w = cos((current_angle + M_PI) / 2.0);
            pose_pub_falcon2.publish(command_falcon2);
            
            // Calculate the difference between the current angle and the target angle
            double angle_difference = target_angle - current_angle;

            // Choose the direction with the smallest angle difference
            if (angle_difference > 0) {
                current_angle += angular_step;
            } else {
                current_angle -= angular_step;
            }

            // Sleep to maintain the loop rate
            loop_rate.sleep();
        }
    }

    bool PoseRecieved() {
        return recievedPose1 && recievedPose2;
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

    while (!turn_handler.PoseRecieved()) {
        ros::spinOnce();
    }

    turn_handler.CalculateMidpoint();
    turn_handler.CalculateAngle();
    turn_handler.turn();

    return 0;
}
