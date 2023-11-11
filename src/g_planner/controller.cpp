#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <g_planner/point.h>
#include <std_msgs/Bool.h>

mavros_msgs::State current_state;
ros::Publisher fb_pub;
ros::Publisher local_pos_pub;
// ros::Rate loop_rate(20);
std_msgs::Bool fb;


g_planner::point trajectory;
bool trajectory_updated = false;

float x_thresh = 0.01;
float y_thresh = 0.01;
float z_thresh = 0.01;

geometry_msgs::PoseStamped curr_pose;

bool reached(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped start){

    if (fabs(goal.pose.position.x - start.pose.position.x) < x_thresh &&
        fabs(goal.pose.position.y - start.pose.position.y) < y_thresh &&
        fabs(goal.pose.position.z - start.pose.position.z) < z_thresh){
            return true;
    }
    return false;
}

void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    curr_pose.pose.position.x = pose.pose.position.x;
    curr_pose.pose.position.y = pose.pose.position.y;
    curr_pose.pose.position.z = pose.pose.position.z;
}

void trajectory_cb(g_planner::point traj){
    trajectory = traj;
    fb.data = false;
    ros::Rate loop_rate(20);
    // trajectory_updated = true;
    std::cout<<"Size of trajectory is: "<<trajectory.points.size()<<std::endl;

    for (int i = 0; i < trajectory.points.size(); i++){
        if (!ros::ok()) break;
        geometry_msgs::PoseStamped point;
        point.pose.position.x = trajectory.points[i].x;
        point.pose.position.y = trajectory.points[i].y;
        point.pose.position.z = trajectory.points[i].z;

        std::cout<<"Published point "<<i<<std::endl;  
        while(!reached(point,curr_pose))      
            for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(point);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    fb.data = true;
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     fb_pub.publish(fb);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// void control(ros::Publisher pub, ros::Rate rate){

//     if (trajectory_updated){
//         trajectory_updated = false;
//         std::cout<<"Size of trajectory is: "<<trajectory.points.size()<<std::endl;

//         for (int i = 0; i < trajectory.points.size(); i++){
//             if (!ros::ok()) break;
//             geometry_msgs::PoseStamped point;
//             point.pose.position.x = trajectory.points[i].x;
//             point.pose.position.x = trajectory.points[i].y;
//             point.pose.position.x = trajectory.points[i].z;

//             std::cout<<"Published point"<<i<<std::endl;        
//              for(int i = 100; ros::ok() && i > 0; --i){
//                 pub.publish(point);
//                 ros::spinOnce();
//                 rate.sleep();
//             }


//         }
//     }else{
//         std::cout<<"Trajectory not available";
//     }

// }

int main(int argc, char **argv){

    ros::init(argc,argv, "controller");
    ros::NodeHandle n;
    fb_pub = n.advertise<std_msgs::Bool> ("feedback", 10);
    local_pos_pub = n.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    ros::Subscriber loc = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);
    ros::Subscriber trajectory = n.subscribe<g_planner::point> ("trajectory", 10, trajectory_cb);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

    ros::Rate loop_rate(20);



    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // while (!trajectory_updated || !ros::ok()){
    //     std::cout<< "Waiting for Trajectory\n";
    //     ros::spinOnce();
    //     loop_rate.sleep();

    //     if (!ros::ok()) break;
    // }
    fb.data = false;
    geometry_msgs::PoseStamped pose;

    std::cout<<"Controller is starting\n";

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for(int i = 0; i<100 && ros::ok(); i++)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    /** set mode to offboard **/
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
    }
    
    if( arming_client.call(arm_cmd) &&  arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }
    fb.data = true;
    

    while (ros::ok()){
        // control(local_pos_pub, loop_rate);

        for(int i = 100; ros::ok() && i > 0; --i){
            fb_pub.publish(fb);
            ros::spinOnce();
            loop_rate.sleep();
        }

        ros::spinOnce();
        loop_rate.sleep();      

        if(!ros::ok())
        break;
    }
    return 0;
}