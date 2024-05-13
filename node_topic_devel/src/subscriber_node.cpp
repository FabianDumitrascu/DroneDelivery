#include "ros/ros.h"
#include "node_topic_devel/person_data.h"

void callback(const node_topic_devel::person_data &person_data){
    ROS_INFO("I heard name: [%s]", person_data.name.c_str());
    ROS_INFO("I heard age: [%d]", person_data.age);
    ROS_INFO("I heard color: [%s]", person_data.color.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("tutorial", 1000, callback);
    ros::spin();


    

    return 0;
}