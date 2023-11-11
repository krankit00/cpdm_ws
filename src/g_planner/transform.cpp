#include <bits/stdc++.h>
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/PositionCommand.h"


using namespace std;
ros::Publisher pub;

void transform_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    geometry_msgs::PoseStamped point;
    point.pose.position.x = msg->position.x;
    point.pose.position.y = msg->position.y;
    point.pose.position.z = msg->position.z;

    pub.publish(point);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "transform");
    ros::NodeHandle n;
    ros::Subscriber surface_feedback = n.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd",1,transform_cb);
    pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Rate loop_rate(20);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
