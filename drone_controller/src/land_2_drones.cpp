#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "land_2_drones");
    ros::NodeHandle nh("land");
    ROS_INFO("Current namespace: %s", nh.getNamespace().c_str());

    // Retrieve drone names from the parameter server or set to default
    std::string drone_name_1, drone_name_2;

    nh.getParam("drone_name_1", drone_name_1);
    nh.getParam("drone_name_2", drone_name_2);

    ROS_INFO("Retrieved 'drone_name1': %s", drone_name_1.c_str());
    ROS_INFO("Retrieved 'drone_name2': %s", drone_name_2.c_str());

    // Set up the publishers for the drone land commands
    ros::Publisher land_pub_1 = nh.advertise<std_msgs::Empty>("/" + drone_name_1 + "/agiros_pilot/land", 1, true);
    ros::Publisher land_pub_2 = nh.advertise<std_msgs::Empty>("/" + drone_name_2 + "/agiros_pilot/land", 1, true);
    ros::Publisher forcehover_pub_1 = nh.advertise<std_msgs::Empty>("/" + drone_name_1 + "/agiros_pilot/force_hover", 1, true);
    ros::Publisher forcehover_pub_2 = nh.advertise<std_msgs::Empty>("/" + drone_name_2 + "/agiros_pilot/force_hover", 1, true);
    ros::Publisher off_pub_1 = nh.advertise<std_msgs::Empty>("/" + drone_name_1 + "/agiros_pilot/off", 1, true);
    ros::Publisher off_pub_2 = nh.advertise<std_msgs::Empty>("/" + drone_name_2 + "/agiros_pilot/off", 1, true);
    // Ensure that the publishers are valid and the master has been notified of the advertisement
    if (!land_pub_1 || !land_pub_2) {
        ROS_ERROR("Failed to advertise on the land topics!");
        return -1;
    }

    // // Sleep to give ROS time to set up the connection
    ros::Duration(0.5).sleep(); 
    // ros::Duration(1).sleep(); 
    std_msgs::Empty msg;
    forcehover_pub_1.publish(msg);
    forcehover_pub_2.publish(msg);

    // ros::Duration(2).sleep(); 
    // Publish the land command
    land_pub_1.publish(msg);
    land_pub_2.publish(msg);

    // Sleep to ensure the messages have time to be sent
    // ros::Duration(8).sleep(); 

    // off_pub_1.publish(msg);
    // off_pub_2.publish(msg);

    // Sleep to ensure the messages have time to be sent
    ros::Duration(1).sleep(); 

    return 0;
}
